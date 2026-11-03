use crate::config::{BatteryConfig, BatteryState, DroneConfig, EnvironmentConfig, MotorConfig};
use crate::domain_randomization::ResetGeneration;
use crate::motor_rpms::MotorRPMs;
use avian3d::prelude::*;
use bevy::prelude::*;

#[derive(Component)]
pub struct Drone;

#[derive(Clone, Copy, Debug, Default)]
pub struct MotorState {
    pub rpm: f32,
    pub target_rpm: f32,
}

/// Mirror of the authoritative single-drone MotorRPMs resource.
#[derive(Component, Clone, Debug, Default)]
pub struct Motors {
    pub states: [MotorState; 4],
}

impl Motors {
    pub fn new() -> Self {
        Self::default()
    }

    fn sync(&mut self, rpms: &MotorRPMs) {
        for i in 0..4 {
            self.states[i].rpm = rpms.rpms[i];
            self.states[i].target_rpm = rpms.target_rpms[i];
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct MotorCommand {
    pub rpms: [f32; 4],
}

#[derive(Clone, Copy, Debug, Default)]
pub enum DroneCommand {
    Motor(MotorCommand),
    /// Collective body +Y thrust (N), body torque (N*m); no tilt compensation.
    ThrustTorque {
        thrust: f32,
        torque: Vec3,
    },
    /// World velocity (m/s), world +Y heading rate (rad/s).
    /// Zero components capture and hold the corresponding world position axes.
    Velocity {
        vel: Vec3,
        yaw_rate: f32,
    },
    /// Y-up attitude: roll about Z, pitch about X, yaw about Y (YXZ).
    /// Thrust is collective body thrust (N), not a vertical force request.
    Attitude {
        roll: f32,
        pitch: f32,
        yaw: f32,
        thrust: f32,
    },
    Position {
        pos: Vec3,
        yaw: f32,
    },
    /// Capture current position and heading, then actively hold them (not disarm).
    #[default]
    None,
}

#[derive(Component, Default)]
pub struct CommandInput {
    pub command: DroneCommand,
}

/// Quaternion error and angular damping are both expressed in the current body
/// frame. Inertia-scaled gains bound angular acceleration on light airframes.
/// Desired angular velocity is world-frame, just like Avian AngularVelocity.
pub fn attitude_body_torque(
    cfg: &DroneConfig,
    current: Quat,
    desired: Quat,
    angular_velocity_world: Vec3,
    desired_angular_velocity_world: Vec3,
    dt: f32,
) -> Vec3 {
    let mut error = (current.inverse() * desired).normalize();
    if error.w < 0.0 {
        error = -error;
    }
    let rate_error = current.inverse() * (desired_angular_velocity_world - angular_velocity_world);
    // Keep bandwidth below actuator lag and discrete-time limits, rather than
    // imposing N*m gains that can explode as inertia is randomized downward.
    let lag = cfg
        .motors
        .iter()
        .map(|m| m.time_constant)
        .fold(0.02_f32, f32::max);
    let frequency = 6.0_f32.min(0.25 / lag.max(1e-4)).min(0.15 / dt.max(1e-4));
    let acceleration = error.to_scaled_axis() * frequency.powi(2) + rate_error * (2.0 * frequency);
    cfg.inertia.max(Vec3::splat(1e-6)) * acceleration.clamp(Vec3::splat(-60.0), Vec3::splat(60.0))
}

fn attitude_for_acceleration(acceleration: Vec3, gravity: f32, yaw: f32) -> Quat {
    let up = (acceleration + Vec3::Y * gravity).normalize_or(Vec3::Y);
    let heading = Quat::from_rotation_y(yaw) * Vec3::Z;
    let right = up.cross(heading).normalize_or(Vec3::X);
    let forward = right.cross(up).normalize();
    Quat::from_mat3(&Mat3::from_cols(right, up, forward))
}

const POSITION_GAIN: f32 = 1.0;
const VELOCITY_GAIN: f32 = 2.5;

/// Local, automatically initialized inner-loop memory. Like MotorRPMs, this
/// system currently supports one drone. No resource registration is required.
#[derive(Default)]
pub struct FlightControlState {
    reset_generation: u64,
    previous_position: Option<Vec3>,
    mode: Option<u8>,
    hold_position: Vec3,
    moving: [bool; 3],
    heading: f32,
    integral_acceleration: Vec3,
}

impl FlightControlState {
    fn reset_if_generation_changed(&mut self, generation: u64) {
        if self.reset_generation != generation {
            *self = Self {
                reset_generation: generation,
                ..default()
            };
        }
    }

    #[cfg(test)]
    fn wrench(
        &mut self,
        command: DroneCommand,
        cfg: &DroneConfig,
        env: &EnvironmentConfig,
        tf: &Transform,
        velocity: Vec3,
        angular_velocity: Vec3,
        dt: f32,
        max_thrust: f32,
    ) -> (f32, Vec3) {
        self.wrench_with_limits(
            command,
            cfg,
            env,
            tf,
            velocity,
            angular_velocity,
            dt,
            max_thrust,
            &crate::config::FlightControlLimits::default(),
        )
    }

    fn wrench_with_limits(
        &mut self,
        command: DroneCommand,
        cfg: &DroneConfig,
        env: &EnvironmentConfig,
        tf: &Transform,
        velocity: Vec3,
        angular_velocity: Vec3,
        dt: f32,
        max_thrust: f32,
        limits: &crate::config::FlightControlLimits,
    ) -> (f32, Vec3) {
        let dt = dt.max(0.0);
        let mode = match command {
            DroneCommand::None => 0,
            DroneCommand::Velocity { .. } => 1,
            DroneCommand::Position { .. } => 2,
            DroneCommand::Attitude { .. } => 3,
            _ => 4,
        };
        let teleported = self.previous_position.is_some_and(|previous| {
            (tf.translation - previous - velocity * dt).length() > 1.0 + velocity.length() * dt
        });
        if self.mode != Some(mode) || teleported {
            self.hold_position = tf.translation;
            self.heading = tf.rotation.to_euler(EulerRot::YXZ).0;
            self.moving = [false; 3];
            self.integral_acceleration = Vec3::ZERO;
        }
        self.previous_position = Some(tf.translation);
        self.mode = Some(mode);

        let (desired_velocity, yaw_rate) = match command {
            DroneCommand::Attitude {
                roll,
                pitch,
                yaw,
                thrust,
            } => {
                let desired = Quat::from_euler(EulerRot::YXZ, yaw, pitch, roll);
                return (
                    thrust.max(0.0),
                    attitude_body_torque(
                        cfg,
                        tf.rotation,
                        desired,
                        angular_velocity,
                        Vec3::ZERO,
                        dt,
                    ),
                );
            }
            DroneCommand::ThrustTorque { thrust, torque } => return (thrust, torque),
            DroneCommand::Motor(_) => return (0.0, Vec3::ZERO),
            DroneCommand::Position { pos, yaw } => {
                self.heading = yaw;
                (
                    (pos - tf.translation).clamp_length_max(limits.max_speed_m_s) * POSITION_GAIN,
                    0.0,
                )
            }
            DroneCommand::Velocity { vel, yaw_rate } => {
                let mut target = vel.clamp_length_max(limits.max_speed_m_s);
                for axis in 0..3 {
                    let moving = vel[axis].abs() > 1e-4;
                    if moving || self.moving[axis] {
                        // Capture the release position once, not on every idle tick.
                        self.hold_position[axis] = tf.translation[axis];
                    }
                    if !moving {
                        target[axis] = ((self.hold_position[axis] - tf.translation[axis])
                            * POSITION_GAIN)
                            .clamp(-3.0, 3.0);
                    }
                    self.moving[axis] = moving;
                }
                (target, yaw_rate.clamp(-2.0, 2.0))
            }
            DroneCommand::None => (
                (self.hold_position - tf.translation).clamp_length_max(limits.max_speed_m_s)
                    * POSITION_GAIN,
                0.0,
            ),
        };
        self.heading = (self.heading + yaw_rate * dt + std::f32::consts::PI)
            .rem_euclid(std::f32::consts::TAU)
            - std::f32::consts::PI;
        let velocity_error = desired_velocity - velocity;
        let gravity = env.gravity.max(0.0);
        let up_projection = (tf.rotation * Vec3::Y).y;
        let requested = velocity_error * VELOCITY_GAIN + self.integral_acceleration;
        let vertical = requested
            .y
            .clamp(-4.0, 4.0)
            .clamp(-limits.max_acceleration_m_s2, limits.max_acceleration_m_s2);
        let remaining_acceleration = (limits.max_acceleration_m_s2.powi(2) - vertical.powi(2))
            .max(0.0)
            .sqrt();
        let max_horizontal_acceleration = ((gravity + vertical).max(0.1)
            * limits.max_tilt_deg.to_radians().tan())
        .min(remaining_acceleration);
        let horizontal = Vec3::new(requested.x, 0.0, requested.z)
            .clamp_length_max(max_horizontal_acceleration.min(limits.max_acceleration_m_s2));
        let acceleration = horizontal + Vec3::Y * vertical;
        let desired = attitude_for_acceleration(acceleration, gravity, self.heading);
        // Compensate actual tilt, not merely commanded tilt. Limit recovery thrust
        // so a nearly sideways/inverted drone is not launched horizontally.
        let thrust = (cfg.mass * (gravity + vertical).max(0.0) / up_projection.max(0.25))
            .min(cfg.mass * gravity.max(1.0) * 2.5)
            .min(max_thrust);
        // Bounded world-frame integral rejects steady wind. Freeze it during
        // actuator/tilt/acceleration saturation (including low-voltage ceilings).
        if up_projection > 0.85
            && thrust < max_thrust * 0.95
            && (requested - acceleration).length() < 1e-3
            && velocity_error.length() < 2.0
        {
            self.integral_acceleration = (self.integral_acceleration + velocity_error * (0.4 * dt))
                .clamp(Vec3::splat(-2.0), Vec3::splat(2.0));
        }
        let torque = attitude_body_torque(
            cfg,
            tf.rotation,
            desired,
            angular_velocity,
            Vec3::Y * yaw_rate,
            dt,
        );
        (thrust, torque)
    }
}

fn voltage_max_rpms(cfg: &DroneConfig, batt: &BatteryState, battery: &BatteryConfig) -> [f32; 4] {
    // Voltage limits available speed; it must not multiply a hover RPM setpoint.
    let factor = (batt.voltage / battery.nominal_voltage.max(1e-3)).clamp(0.0, 1.0);
    std::array::from_fn(|i| cfg.motors[i].max_rpm.max(0.0) * factor)
}

pub fn command_mixer_system(
    time: Res<Time>,
    cfg: Res<DroneConfig>,
    flight_limits: Option<Res<crate::config::FlightControlLimits>>,
    env: Res<EnvironmentConfig>,
    batt: Res<BatteryState>,
    battery: Res<BatteryConfig>,
    reset_generation: Option<Res<ResetGeneration>>,
    mut state: Local<FlightControlState>,
    mut motor_rpms: ResMut<MotorRPMs>,
    mut query: Query<
        (
            &mut Motors,
            &CommandInput,
            &Transform,
            &LinearVelocity,
            &AngularVelocity,
        ),
        With<Drone>,
    >,
) {
    // The generation also detects resets too small for the teleport heuristic.
    state.reset_if_generation_changed(
        reset_generation
            .as_ref()
            .map_or(0, |generation| generation.0),
    );
    let flight_limits = flight_limits.as_deref().copied().unwrap_or_default();
    let rpm_limits = voltage_max_rpms(&cfg, &batt, &battery);
    let max_thrust: f32 = (0..4)
        .map(|i| rpm_force(&cfg.motors[i], &env, rpm_limits[i]))
        .sum();
    for (mut motors, input, tf, linvel, angvel) in &mut query {
        let (thrust, torque) = state.wrench_with_limits(
            input.command,
            &cfg,
            &env,
            tf,
            linvel.0,
            angvel.0,
            time.delta_secs(),
            max_thrust,
            &flight_limits,
        );
        let rpms = match input.command {
            DroneCommand::Motor(command) => std::array::from_fn(|i| {
                command.rpms[i].clamp(
                    cfg.motors[i].min_rpm.max(0.0).min(rpm_limits[i]),
                    rpm_limits[i],
                )
            }),
            _ => mix_with_limits(&cfg, &env, thrust, torque, rpm_limits),
        };
        motor_rpms.target_rpms = rpms;
        motors.sync(&motor_rpms);
    }
}

#[derive(Default)]
pub struct MotorDynamicsState {
    reset_generation: u64,
    rates: [f32; 4],
    previous_rpms: [f32; 4],
}

impl MotorDynamicsState {
    fn reset_if_generation_changed(&mut self, generation: u64) {
        if self.reset_generation != generation {
            *self = Self {
                reset_generation: generation,
                ..default()
            };
        }
    }
}

fn motor_step(mc: &MotorConfig, rpm: f32, target: f32, rate: &mut f32, dt: f32) -> f32 {
    if dt <= 0.0 {
        return rpm;
    }
    let tau = mc.time_constant.max(1e-4);
    let max_rate = mc.rate_limit_rpm_per_s.max(0.0);
    let delta = if mc.second_order {
        // Implicit damped spring: stable even when dt is comparable to tau.
        let omega = 1.0 / tau;
        *rate = (*rate + dt * omega * omega * (target - rpm))
            / (1.0 + 2.0 * mc.damping.max(0.05) * omega * dt + (omega * dt).powi(2));
        *rate = rate.clamp(-max_rate, max_rate);
        *rate * dt
    } else {
        (target - rpm) * (1.0 - (-dt / tau).exp())
    };
    let next = (rpm + delta.clamp(-max_rate * dt, max_rate * dt)).clamp(0.0, mc.max_rpm.max(0.0));
    if mc.second_order && (next <= 0.0 || next >= mc.max_rpm) {
        *rate = 0.0;
    }
    next
}

/// Fixed-time lag followed by a hard RPM/s slew limit, for both motor models.
/// A falling voltage ceiling limits the target, not instantaneous rotor speed.
pub fn motor_dynamics_system(
    time: Res<Time>,
    cfg: Res<DroneConfig>,
    batt: Res<BatteryState>,
    battery: Res<BatteryConfig>,
    reset_generation: Option<Res<ResetGeneration>>,
    mut state: Local<MotorDynamicsState>,
    mut motor_rpms: ResMut<MotorRPMs>,
    mut motors: Query<&mut Motors, With<Drone>>,
    mut aircraft_controls: Query<&mut crate::aircraft::AircraftControls, With<Drone>>,
) {
    // Do not carry second-order rotor momentum across an external reset, even
    // when the reset leaves the authoritative RPM resource numerically unchanged.
    state.reset_if_generation_changed(
        reset_generation
            .as_ref()
            .map_or(0, |generation| generation.0),
    );
    let limits = voltage_max_rpms(&cfg, &batt, &battery);
    for i in 0..4 {
        let mc = &cfg.motors[i];
        if motor_rpms.rpms[i] != state.previous_rpms[i] {
            state.rates[i] = 0.0;
        }
        let target = motor_rpms.target_rpms[i].clamp(mc.min_rpm.max(0.0).min(limits[i]), limits[i]);
        motor_rpms.target_rpms[i] = target;
        motor_rpms.rpms[i] = motor_step(
            mc,
            motor_rpms.rpms[i],
            target,
            &mut state.rates[i],
            time.delta_secs(),
        );
        state.previous_rpms[i] = motor_rpms.rpms[i];
    }
    for mut motors in &mut motors {
        motors.sync(&motor_rpms);
    }
    for mut controls in &mut aircraft_controls {
        controls.propulsion.resize(cfg.motors.len(), 0.0);
        for (index, motor) in cfg.motors.iter().enumerate() {
            controls.propulsion[index] =
                (motor_rpms.rpms[index] / motor.max_rpm.max(1.0)).clamp(0.0, 1.0);
        }
    }
}

fn thrust_coefficient(mc: &MotorConfig, env: &EnvironmentConfig) -> f32 {
    mc.thrust_coeff.max(0.0) * mc.voltage_efficiency.max(0.0) * (env.air_density / 1.225).max(0.0)
}

fn rpm_force(mc: &MotorConfig, env: &EnvironmentConfig, rpm: f32) -> f32 {
    thrust_coefficient(mc, env) * (rpm * std::f32::consts::TAU / 60.0).powi(2)
}

/// Body force/torque; cross(r, +Y F) = (-r.z F, 0, r.x F).
pub fn motor_forces_and_torques(
    cfg: &DroneConfig,
    env: &EnvironmentConfig,
    motor_rpms: &MotorRPMs,
) -> (Vec3, Vec3) {
    let mut force = Vec3::ZERO;
    let mut torque = Vec3::ZERO;
    for i in 0..4 {
        let mc = &cfg.motors[i];
        let rpm = motor_rpms.rpms[i].clamp(0.0, mc.max_rpm.max(0.0));
        let thrust = Vec3::Y * rpm_force(mc, env, rpm);
        force += thrust;
        torque += mc.position.cross(thrust);
        torque.y += mc.rotation_dir
            * mc.torque_coeff.max(0.0)
            * (rpm * std::f32::consts::TAU / 60.0).powi(2)
            * (env.air_density / 1.225).max(0.0);
    }
    (force, torque)
}

/// Compatibility helper for nominal sea-level density and nominal RPM ceilings.
/// Runtime callers should use the environmental variant (or the mixer system).
pub fn mix_thrust_torque(cfg: &DroneConfig, thrust: f32, torque: Vec3) -> [f32; 4] {
    mix_thrust_torque_in_environment(cfg, &EnvironmentConfig::default(), thrust, torque)
}

pub fn mix_thrust_torque_in_environment(
    cfg: &DroneConfig,
    env: &EnvironmentConfig,
    thrust: f32,
    torque: Vec3,
) -> [f32; 4] {
    mix_with_limits(
        cfg,
        env,
        thrust,
        torque,
        std::array::from_fn(|i| cfg.motors[i].max_rpm.max(0.0)),
    )
}

fn mix_with_limits(
    cfg: &DroneConfig,
    env: &EnvironmentConfig,
    thrust: f32,
    torque: Vec3,
    limits: [f32; 4],
) -> [f32; 4] {
    let columns: [Vec4; 4] = std::array::from_fn(|i| {
        let mc = &cfg.motors[i];
        let effective_kt = mc.thrust_coeff * mc.voltage_efficiency;
        let arm = mc.position - cfg.center_of_mass_offset;
        Vec4::new(
            1.0,
            -arm.z,
            mc.rotation_dir * mc.torque_coeff / effective_kt.max(1e-12),
            arm.x,
        )
    });
    let allocation = Mat4::from_cols(columns[0], columns[1], columns[2], columns[3]);
    let min_force: [f32; 4] = std::array::from_fn(|i| {
        rpm_force(
            &cfg.motors[i],
            env,
            cfg.motors[i].min_rpm.max(0.0).min(limits[i]),
        )
    });
    let max_force: [f32; 4] = std::array::from_fn(|i| rpm_force(&cfg.motors[i], env, limits[i]));
    let minimum: f32 = min_force.iter().sum();
    let maximum: f32 = max_force.iter().sum();
    let collective = thrust.clamp(minimum, maximum);
    let headroom = maximum - minimum;
    // This fallback always preserves achievable collective, including a singular
    // geometry or insufficient authority for a torque-neutral baseline.
    let mut base = Vec4::from_array(std::array::from_fn(|i| {
        min_force[i]
            + (max_force[i] - min_force[i]) * ((collective - minimum) / headroom.max(1e-12))
    }));
    let mut forces = base;
    if allocation.determinant().abs() > 1e-9 {
        let inverse = allocation.inverse();
        let neutral = inverse * Vec4::new(collective, 0.0, 0.0, 0.0);
        if (0..4).all(|i| neutral[i] >= min_force[i] && neutral[i] <= max_force[i]) {
            base = neutral;
        }
        let desired = inverse * Vec4::new(collective, torque.x, torque.y, torque.z);
        let mut differential = desired - base;
        // Remove roundoff in the collective row before scaling differential torque.
        differential -= Vec4::splat(differential.element_sum() * 0.25);
        let mut scale = 1.0_f32;
        for i in 0..4 {
            if differential[i] > 1e-8 {
                scale = scale.min((max_force[i] - base[i]) / differential[i]);
            } else if differential[i] < -1e-8 {
                scale = scale.min((min_force[i] - base[i]) / differential[i]);
            }
        }
        forces = base + differential * scale.clamp(0.0, 1.0);
    }
    std::array::from_fn(|i| {
        let kt = thrust_coefficient(&cfg.motors[i], env);
        if kt <= 0.0 {
            0.0
        } else {
            ((forces[i].max(0.0) / kt).sqrt() * 60.0 / std::f32::consts::TAU).clamp(0.0, limits[i])
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn forces(cfg: &DroneConfig, env: &EnvironmentConfig, rpms: [f32; 4]) -> (Vec3, Vec3) {
        motor_forces_and_torques(
            cfg,
            env,
            &MotorRPMs {
                rpms,
                target_rpms: rpms,
            },
        )
    }

    #[test]
    fn mixer_recovers_collective_and_signed_torques() {
        let cfg = DroneConfig::default();
        let env = EnvironmentConfig::default();
        let thrust = cfg.mass * env.gravity;
        for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
            for sign in [-1.0, 1.0] {
                let target = axis * sign * 0.15;
                let rpms = mix_thrust_torque(&cfg, thrust, target);
                let (force, torque) = forces(&cfg, &env, rpms);
                assert!((force.y - thrust).abs() < 1e-4, "{force:?}");
                assert!(
                    (torque - target).length() < 1e-4,
                    "{torque:?} != {target:?}"
                );
            }
        }
    }

    #[test]
    fn mixer_uses_each_motor_efficiency_geometry_and_air_density() {
        let mut cfg = DroneConfig::default();
        for i in 0..4 {
            cfg.motors[i].thrust_coeff *= 0.7 + i as f32 * 0.2;
            cfg.motors[i].voltage_efficiency = 0.65 + i as f32 * 0.1;
            cfg.motors[i].torque_coeff *= 1.2 - i as f32 * 0.1;
            cfg.motors[i].position.x *= 1.1;
        }
        let env = EnvironmentConfig {
            air_density: 0.8,
            gravity: 7.0,
            ..default()
        };
        // Stay within yaw authority even for the weakest, low-density motor.
        let target = Vec3::new(0.02, -0.03, 0.04);
        let thrust = cfg.mass * env.gravity;
        let rpms = mix_thrust_torque_in_environment(&cfg, &env, thrust, target);
        let (force, torque) = forces(&cfg, &env, rpms);
        assert!((force.y - thrust).abs() < 1e-4);
        assert!((torque - target).length() < 1e-4, "{torque:?}");
    }

    #[test]
    fn mixer_balances_hover_about_displaced_center_of_mass() {
        let cfg = DroneConfig {
            center_of_mass_offset: Vec3::new(0.02, 0.005, -0.015),
            ..default()
        };
        let env = EnvironmentConfig::default();
        let requested_torque = Vec3::new(0.02, -0.03, 0.04);
        let rpms =
            mix_thrust_torque_in_environment(&cfg, &env, cfg.mass * env.gravity, requested_torque);
        let (force, origin_torque) = forces(&cfg, &env, rpms);
        let com_torque = origin_torque - cfg.center_of_mass_offset.cross(force);
        assert!((force.y - cfg.mass * env.gravity).abs() < 1e-4);
        assert!(
            (com_torque - requested_torque).length() < 1e-4,
            "{com_torque:?}"
        );
    }

    #[test]
    fn saturation_preserves_collective_instead_of_clipping_each_motor() {
        let cfg = DroneConfig::default();
        let env = EnvironmentConfig::default();
        for thrust in [0.0, 1.0, 14.715, 30.0, 100.0] {
            let limits = [4000.0; 4];
            let rpms = mix_with_limits(&cfg, &env, thrust, Vec3::new(50.0, -30.0, 40.0), limits);
            assert!(rpms.iter().all(|rpm| *rpm >= 0.0 && *rpm <= 4000.0));
            let (force, _) = forces(&cfg, &env, rpms);
            let maximum: f32 = cfg.motors.iter().map(|m| rpm_force(m, &env, 4000.0)).sum();
            assert!((force.y - thrust.min(maximum)).abs() < 1e-4, "{force:?}");
        }
    }

    #[test]
    fn attitude_is_negative_feedback_in_body_frame_and_shortest_arc() {
        let cfg = DroneConfig::default();
        let heading = Quat::from_rotation_y(1.2);
        for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
            let current = heading * Quat::from_scaled_axis(axis * 0.2);
            let torque =
                attitude_body_torque(&cfg, current, heading, Vec3::ZERO, Vec3::ZERO, 1.0 / 240.0);
            assert!(torque.dot(axis) < 0.0, "{torque:?}");
            let equivalent =
                attitude_body_torque(&cfg, current, -heading, Vec3::ZERO, Vec3::ZERO, 1.0 / 240.0);
            assert!((equivalent - torque).length() < 1e-5);
            let damp = attitude_body_torque(
                &cfg,
                heading,
                heading,
                heading * axis,
                Vec3::ZERO,
                1.0 / 240.0,
            );
            assert!(damp.dot(axis) < 0.0);
        }
        let mut light = cfg.clone();
        light.inertia *= 0.01;
        let torque = attitude_body_torque(
            &light,
            Quat::IDENTITY,
            Quat::from_rotation_x(0.2),
            Vec3::ZERO,
            Vec3::ZERO,
            1.0 / 240.0,
        );
        assert!((torque.x / light.inertia.x - 7.2).abs() < 1e-4);
    }

    #[test]
    fn velocity_tilt_is_world_correct_at_arbitrary_heading() {
        let cfg = DroneConfig::default();
        let env = EnvironmentConfig::default();
        for yaw in [0.0, 1.1, -2.4] {
            let tf = Transform::from_rotation(Quat::from_rotation_y(yaw));
            for velocity in [Vec3::X, Vec3::Z, Vec3::NEG_X, Vec3::NEG_Z] {
                let mut state = FlightControlState::default();
                let (_, torque) = state.wrench(
                    DroneCommand::Velocity {
                        vel: velocity,
                        yaw_rate: 0.0,
                    },
                    &cfg,
                    &env,
                    &tf,
                    Vec3::ZERO,
                    Vec3::ZERO,
                    1.0 / 240.0,
                    100.0,
                );
                let angular_acceleration_world = tf.rotation * (torque / cfg.inertia);
                let up_acceleration = angular_acceleration_world.cross(Vec3::Y);
                assert!(
                    up_acceleration.dot(velocity) > 0.0,
                    "yaw={yaw} torque={torque:?}"
                );
            }
        }
    }

    #[test]
    fn idle_holds_release_position_altitude_heading_and_resets_on_teleport() {
        let cfg = DroneConfig::default();
        let env = EnvironmentConfig {
            gravity: 6.0,
            ..default()
        };
        let dt = 1.0 / 240.0;
        let mut state = FlightControlState::default();
        let mut tf = Transform::from_xyz(1.0, 4.0, -3.0);
        let idle = DroneCommand::Velocity {
            vel: Vec3::ZERO,
            yaw_rate: 0.0,
        };
        let (thrust, torque) =
            state.wrench(idle, &cfg, &env, &tf, Vec3::ZERO, Vec3::ZERO, dt, 100.0);
        assert!((thrust - cfg.mass * env.gravity).abs() < 1e-5);
        assert!(torque.length() < 1e-5);
        state.wrench(
            DroneCommand::Velocity {
                vel: Vec3::Y,
                yaw_rate: 0.7,
            },
            &cfg,
            &env,
            &tf,
            Vec3::ZERO,
            Vec3::ZERO,
            dt,
            100.0,
        );
        tf.translation.y += 0.2;
        state.wrench(idle, &cfg, &env, &tf, Vec3::ZERO, Vec3::ZERO, dt, 100.0);
        assert_eq!(state.hold_position, tf.translation);
        let release = state.hold_position;
        tf.translation.y -= 0.1;
        tf.translation.x += 0.1;
        let (thrust, torque) =
            state.wrench(idle, &cfg, &env, &tf, Vec3::ZERO, Vec3::ZERO, dt, 100.0);
        assert_eq!(state.hold_position, release);
        assert!(thrust > cfg.mass * env.gravity);
        assert!(torque.z > 0.0); // +Z torque tilts thrust toward -X.
        assert!(torque.y > 0.0); // Heading remains captured after Q/E release.
        tf.translation = Vec3::new(20.0, 8.0, -10.0);
        let (_, torque) = state.wrench(idle, &cfg, &env, &tf, Vec3::ZERO, Vec3::ZERO, dt, 100.0);
        assert_eq!(state.hold_position, tf.translation);
        assert!(state.integral_acceleration.length() < 1e-5);
        assert!(torque.length() < 1e-5);
    }

    #[test]
    fn position_command_holds_altitude_and_tracks_yaw_with_actual_tilt_compensation() {
        let cfg = DroneConfig::default();
        let env = EnvironmentConfig::default();
        let tf = Transform::from_rotation(Quat::from_rotation_x(0.3));
        let mut state = FlightControlState::default();
        let (thrust, torque) = state.wrench(
            DroneCommand::Position {
                pos: Vec3::ZERO,
                yaw: 0.5,
            },
            &cfg,
            &env,
            &tf,
            Vec3::ZERO,
            Vec3::ZERO,
            1.0 / 240.0,
            100.0,
        );
        assert!((thrust * (tf.rotation * Vec3::Y).y - cfg.mass * env.gravity).abs() < 1e-4);
        assert!(torque.x < 0.0 && torque.y > 0.0);
    }

    #[test]
    fn both_motor_models_obey_lag_and_rate_limit_in_both_directions() {
        for second_order in [false, true] {
            let mc = MotorConfig {
                second_order,
                rate_limit_rpm_per_s: 1000.0,
                ..default()
            };
            let mut rpm = 0.0;
            let mut rate = 0.0;
            assert_eq!(motor_step(&mc, rpm, 6000.0, &mut rate, 0.0), rpm);
            for target in [6000.0, 0.0] {
                for tick in 0..2400 {
                    let dt = if tick % 2 == 0 {
                        1.0 / 240.0
                    } else {
                        1.0 / 120.0
                    };
                    let next = motor_step(&mc, rpm, target, &mut rate, dt);
                    assert!((next - rpm).abs() <= mc.rate_limit_rpm_per_s * dt + 1e-3);
                    rpm = next;
                }
                assert!(
                    (rpm - target).abs() < 1.0,
                    "second_order={second_order} rpm={rpm}"
                );
            }
            let slow = MotorConfig {
                second_order,
                time_constant: 1.0,
                rate_limit_rpm_per_s: 1e9,
                ..default()
            };
            let next = motor_step(&slow, 0.0, 6000.0, &mut 0.0, 1.0 / 240.0);
            assert!(next > 0.0 && next < 100.0);
        }
    }

    #[test]
    fn closed_loop_rejects_wind_and_settles_after_release_with_motor_lag() {
        let cfg = DroneConfig::default();
        let env = EnvironmentConfig::default();
        let dt = 1.0 / 240.0;
        let mut tf = Transform::from_xyz(0.0, 4.0, 0.0).with_rotation(Quat::from_euler(
            EulerRot::YXZ,
            1.1,
            0.15,
            -0.1,
        ));
        let mut velocity = Vec3::ZERO;
        let mut body_rate = Vec3::ZERO;
        let mut state = FlightControlState::default();
        let mut rpms = mix_thrust_torque(&cfg, cfg.mass * env.gravity, Vec3::ZERO);
        let mut motor_rates = [0.0; 4];
        let mut release_position = Vec3::ZERO;
        for tick in 0..7200 {
            let moving = tick < 480;
            let command = DroneCommand::Velocity {
                vel: if moving { Vec3::Z } else { Vec3::ZERO },
                yaw_rate: 0.0,
            };
            let (thrust, torque) = state.wrench(
                command,
                &cfg,
                &env,
                &tf,
                velocity,
                tf.rotation * body_rate,
                dt,
                100.0,
            );
            if tick == 480 {
                release_position = state.hold_position;
            }
            let targets = mix_thrust_torque_in_environment(&cfg, &env, thrust, torque);
            for i in 0..4 {
                rpms[i] = motor_step(&cfg.motors[i], rpms[i], targets[i], &mut motor_rates[i], dt);
            }
            let (force, torque) = forces(&cfg, &env, rpms);
            let wind_acceleration = Vec3::new(0.4, 0.2, -0.3);
            let drag = tf.rotation * (-cfg.linear_drag * (tf.rotation.inverse() * velocity));
            velocity += ((tf.rotation * force + drag) / cfg.mass - Vec3::Y * env.gravity
                + wind_acceleration)
                * dt;
            body_rate +=
                (torque - cfg.angular_drag * body_rate - body_rate.cross(cfg.inertia * body_rate))
                    / cfg.inertia
                    * dt;
            // A physically consistent test plant: body angular velocity multiplies
            // the quaternion on the right, world angular velocity on the left.
            tf.rotation = (tf.rotation * Quat::from_scaled_axis(body_rate * dt)).normalize();
            tf.translation += velocity * dt;
            assert!(tf.translation.is_finite() && body_rate.is_finite());
        }
        assert!(
            (tf.translation - release_position).length() < 0.1,
            "release={release_position:?}, final={:?}",
            tf.translation
        );
        assert!(velocity.length() < 0.05, "velocity={velocity:?}");
        assert!(body_rate.length() < 0.05, "body_rate={body_rate:?}");
        assert!((tf.rotation.to_euler(EulerRot::YXZ).0 - 1.1).abs() < 0.03);
    }

    #[test]
    fn generation_reset_recaptures_hold_after_sub_meter_teleport() {
        let mut app = App::new();
        let mut time = Time::<()>::default();
        time.advance_by(std::time::Duration::from_secs_f32(1.0 / 240.0));
        app.insert_resource(time)
            .init_resource::<DroneConfig>()
            .init_resource::<EnvironmentConfig>()
            .init_resource::<BatteryState>()
            .init_resource::<BatteryConfig>()
            .init_resource::<ResetGeneration>()
            .init_resource::<MotorRPMs>()
            .add_systems(Update, command_mixer_system);
        let drone = app
            .world_mut()
            .spawn((
                Drone,
                Motors::default(),
                CommandInput {
                    command: DroneCommand::Velocity {
                        vel: Vec3::ZERO,
                        yaw_rate: 0.5,
                    },
                },
                Transform::from_xyz(2.0, 3.0, -1.0),
                LinearVelocity(Vec3::NEG_Y * 0.1),
                AngularVelocity::default(),
            ))
            .id();
        // Build nonzero heading and integral memory without changing control mode.
        for _ in 0..20 {
            app.update();
        }
        app.world_mut()
            .get_mut::<CommandInput>(drone)
            .unwrap()
            .command = DroneCommand::Velocity {
            vel: Vec3::ZERO,
            yaw_rate: 0.0,
        };
        app.world_mut().get_mut::<LinearVelocity>(drone).unwrap().0 = Vec3::ZERO;
        for generation in [1, 2] {
            {
                let mut tf = app.world_mut().get_mut::<Transform>(drone).unwrap();
                tf.translation += Vec3::new(0.25, 0.15, 0.0);
                tf.rotation = Quat::from_rotation_y(generation as f32 * -0.2);
            }
            app.update();
            let cfg = app.world().resource::<DroneConfig>();
            let env = app.world().resource::<EnvironmentConfig>();
            let (_, stale_torque) =
                forces(cfg, env, app.world().resource::<MotorRPMs>().target_rpms);
            assert!(
                stale_torque.length() > 1e-3,
                "unchanged generation must preserve hold"
            );

            app.world_mut().resource_mut::<ResetGeneration>().0 = generation;
            app.update();
            let cfg = app.world().resource::<DroneConfig>();
            let env = app.world().resource::<EnvironmentConfig>();
            let (force, torque) = forces(cfg, env, app.world().resource::<MotorRPMs>().target_rpms);
            assert!(
                (force.y - cfg.mass * env.gravity).abs() < 1e-4,
                "old integral/altitude leaked across reset: {force:?}"
            );
            assert!(
                torque.length() < 1e-4,
                "old position/heading leaked across reset: {torque:?}"
            );
        }
    }

    #[test]
    fn generation_reset_clears_second_order_memory_without_rewriting_rpms() {
        let mut cfg = DroneConfig::default();
        for motor in &mut cfg.motors {
            motor.second_order = true;
        }
        let mut app = App::new();
        let mut time = Time::<()>::default();
        time.advance_by(std::time::Duration::from_secs_f32(1.0 / 240.0));
        app.insert_resource(time)
            .insert_resource(cfg)
            .init_resource::<BatteryState>()
            .init_resource::<BatteryConfig>()
            .init_resource::<ResetGeneration>()
            .insert_resource(MotorRPMs {
                rpms: [2000.0; 4],
                target_rpms: [6000.0; 4],
            })
            .add_systems(Update, motor_dynamics_system);
        let drone = app.world_mut().spawn((Drone, Motors::default())).id();
        for generation in [1, 2] {
            app.world_mut().resource_mut::<MotorRPMs>().target_rpms = [6000.0; 4];
            for _ in 0..10 {
                app.update();
            }
            let before = app.world().resource::<MotorRPMs>().rpms;
            app.world_mut().resource_mut::<MotorRPMs>().target_rpms = before;
            app.update();
            let spinning = app.world().resource::<MotorRPMs>().rpms;
            assert!(
                spinning[0] > before[0],
                "same generation should retain rotor momentum"
            );
            app.world_mut().resource_mut::<MotorRPMs>().target_rpms = spinning;
            app.world_mut().resource_mut::<ResetGeneration>().0 = generation;
            app.update();
            let rpms = app.world().resource::<MotorRPMs>();
            assert_eq!(
                rpms.rpms, spinning,
                "generation must clear second-order rates"
            );
            let motors = app.world().get::<Motors>(drone).unwrap();
            for i in 0..4 {
                assert_eq!(motors.states[i].rpm, rpms.rpms[i]);
                assert_eq!(motors.states[i].target_rpm, rpms.target_rpms[i]);
            }
        }
    }

    #[test]
    fn voltage_only_caps_targets_and_motors_mirror_resource() {
        let mut app = App::new();
        let mut time = Time::<()>::default();
        time.advance_by(std::time::Duration::from_secs_f32(1.0 / 240.0));
        let mut cfg = DroneConfig::default();
        for motor in &mut cfg.motors {
            motor.voltage_efficiency = 0.7;
        }
        app.insert_resource(time)
            .insert_resource(cfg)
            .init_resource::<BatteryConfig>()
            .insert_resource(BatteryState {
                voltage: 16.8,
                ..default()
            })
            .insert_resource(MotorRPMs {
                rpms: [4000.0; 4],
                target_rpms: [4000.0; 4],
            })
            .add_systems(Update, motor_dynamics_system);
        let entity = app.world_mut().spawn((Drone, Motors::default())).id();
        app.update();
        assert_eq!(app.world().resource::<MotorRPMs>().rpms, [4000.0; 4]);
        app.world_mut().resource_mut::<BatteryState>().voltage = 3.0;
        app.update();
        let rpms = app.world().resource::<MotorRPMs>();
        let motors = app.world().get::<Motors>(entity).unwrap();
        for i in 0..4 {
            assert!(rpms.target_rpms[i] < 2000.0);
            assert!(rpms.rpms[i] >= 4000.0 - 50000.0 / 240.0 - 1e-3);
            assert_eq!(motors.states[i].rpm, rpms.rpms[i]);
            assert_eq!(motors.states[i].target_rpm, rpms.target_rpms[i]);
        }
    }
}
