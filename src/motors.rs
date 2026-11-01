use crate::config::{BatteryState, DroneConfig, EnvironmentConfig};
use crate::motor_rpms::MotorRPMs;
use avian3d::prelude::*;
use bevy::prelude::*;

/// Marker for drone entity.
#[derive(Component)]
pub struct Drone;

/// Motor state for a single motor.
#[derive(Clone, Copy, Debug)]
pub struct MotorState {
    pub rpm: f32,
    pub target_rpm: f32,
}

impl Default for MotorState {
    fn default() -> Self {
        Self {
            rpm: 0.0,
            target_rpm: 0.0,
        }
    }
}

/// Component holding 4 motor states (kept for compatibility, but RPMs now in resource).
#[derive(Component, Clone, Debug, Default)]
pub struct Motors {
    pub states: [MotorState; 4],
}

impl Motors {
    pub fn new() -> Self {
        Self::default()
    }
}

/// Low-level motor command: target RPM per motor.
#[derive(Clone, Copy, Debug, Default)]
pub struct MotorCommand {
    pub rpms: [f32; 4],
}

/// High-level command interface.
#[derive(Clone, Copy, Debug, Default)]
pub enum DroneCommand {
    /// Direct motor RPMs.
    Motor(MotorCommand),
    /// Thrust (N) + body torque (N*m).
    ThrustTorque { thrust: f32, torque: Vec3 },
    /// Desired velocity (world frame) + yaw rate.
    Velocity { vel: Vec3, yaw_rate: f32 },
    /// Desired attitude (roll, pitch, yaw in rad) + thrust.
    Attitude {
        roll: f32,
        pitch: f32,
        yaw: f32,
        thrust: f32,
    },
    /// Desired world position + yaw.
    Position { pos: Vec3, yaw: f32 },
    #[default]
    None,
}

/// Component receiving commands (after network delay).
#[derive(Component, Default)]
pub struct CommandInput {
    pub command: DroneCommand,
}

/// System: translate high/low-level commands into motor target RPMs.
pub fn command_mixer_system(
    cfg: Res<DroneConfig>,
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
    for (mut motors, input, tf, linvel, angvel) in &mut query {
        let cmd = input.command;
        let rpms = match cmd {
            DroneCommand::Motor(m) => m.rpms,
            DroneCommand::ThrustTorque { thrust, torque } => {
                mix_thrust_torque(&cfg, thrust, torque)
            }
            DroneCommand::Attitude {
                roll,
                pitch,
                yaw,
                thrust,
            } => {
                let (cur_roll, cur_pitch, cur_yaw) = tf.rotation.to_euler(EulerRot::XYZ);
                let angvel_body = tf.rotation.inverse() * angvel.0;
                let torque_x = 2.0 * (cur_roll - roll) + 2.0 * angvel_body.x;
                let torque_z = 2.0 * (cur_pitch - pitch) + 2.0 * angvel_body.z;
                let torque_y = 1.0 * (cur_yaw - yaw) + 1.0 * angvel_body.y;
                mix_thrust_torque(&cfg, thrust, Vec3::new(torque_x, torque_y, torque_z))
            }
            DroneCommand::Velocity { vel, yaw_rate } => {
                let err = vel - linvel.0;
                let pitch = (err.z * 0.5).clamp(-0.4, 0.4);
                let roll = (-err.x * 0.5).clamp(-0.4, 0.4);
                // Increase thrust to compensate for tilt: thrust = mass * g / cos(pitch)cos(roll) approx
                let tilt_comp = (pitch * pitch + roll * roll).sqrt();
                let thrust = (cfg.mass * 9.81 / (1.0 - 0.5 * tilt_comp * tilt_comp)
                    + 8.0 * (vel.y - linvel.0.y)
                    - 2.0 * linvel.0.y)
                    .clamp(0.0, cfg.mass * 9.81 * 2.0);
                let (cur_roll, cur_pitch, _) = tf.rotation.to_euler(EulerRot::XYZ);
                let angvel_body = tf.rotation.inverse() * angvel.0;
                let torque_x = 8.0 * (roll - cur_roll) - 2.0 * angvel_body.x;
                let torque_z = 8.0 * (pitch - cur_pitch) - 2.0 * angvel_body.z;
                let torque_y = 2.0 * (yaw_rate - angvel_body.y);
                mix_thrust_torque(&cfg, thrust, Vec3::new(torque_x, torque_y, torque_z))
            }
            DroneCommand::Position { pos, yaw: _ } => {
                let err = pos - tf.translation;
                let vel = err.clamp_length_max(3.0);
                let pitch = (vel.z * 0.1).clamp(-0.3, 0.3);
                let roll = (-vel.x * 0.1).clamp(-0.3, 0.3);
                let thrust = cfg.mass * 9.81;
                let (cur_roll, cur_pitch, _) = tf.rotation.to_euler(EulerRot::XYZ);
                let angvel_body = tf.rotation.inverse() * angvel.0;
                let torque_x = 4.0 * (roll - cur_roll) - 1.5 * angvel_body.x;
                let torque_z = 4.0 * (pitch - cur_pitch) - 1.5 * angvel_body.z;
                mix_thrust_torque(&cfg, thrust, Vec3::new(torque_x, 0.0, torque_z))
            }
            DroneCommand::None => mix_thrust_torque(&cfg, cfg.mass * 9.81, Vec3::ZERO),
        };
        for i in 0..4 {
            motors.states[i].target_rpm = rpms[i];
        }
        // Also update resource
        motor_rpms.target_rpms = rpms;
    }
}

/// System: update motor RPM with first-order/second-order lag + rate limit.
/// Runs in FixedUpdate.
pub fn motor_dynamics_system(
    time: Res<Time>,
    cfg: Res<DroneConfig>,
    batt: Res<BatteryState>,
    mut motor_rpms: ResMut<MotorRPMs>,
) {
    let dt = time.delta_secs().max(1e-5);
    let voltage_factor = (batt.voltage / 14.8).clamp(0.5, 1.2);

    for i in 0..4 {
        let mc = &cfg.motors[i];
        let tau = mc.time_constant.max(1e-4) / 1.0 / mc.voltage_efficiency.max(0.1);
        let eff_target = motor_rpms.target_rpms[i] * 1.0 * mc.voltage_efficiency;
        let max_delta = mc.rate_limit_rpm_per_s * dt;
        let desired = (eff_target - motor_rpms.rpms[i]).clamp(-max_delta, max_delta);
        if mc.second_order {
            let alpha = dt / (tau + dt);
            let vel = desired * alpha;
            motor_rpms.rpms[i] += vel * mc.damping;
        } else {
            let alpha = dt / (tau + dt);
            motor_rpms.rpms[i] += alpha * (eff_target - motor_rpms.rpms[i]);
        }
        motor_rpms.rpms[i] = motor_rpms.rpms[i].clamp(mc.min_rpm, mc.max_rpm);
    }
}

/// Compute thrust vector (body frame) and torque from motors.
pub fn motor_forces_and_torques(
    cfg: &DroneConfig,
    env: &EnvironmentConfig,
    motor_rpms: &MotorRPMs,
) -> (Vec3, Vec3) {
    let air_scale = (env.air_density / 1.225).max(0.1);
    let mut total_thrust = Vec3::ZERO;
    let mut total_torque = Vec3::ZERO;

    for i in 0..4 {
        let mc = &cfg.motors[i];
        let rpm = motor_rpms.rpms[i].clamp(mc.min_rpm, mc.max_rpm);
        let omega = rpm * (2.0 * std::f32::consts::PI / 60.0);
        let thrust_mag = mc.thrust_coeff * omega * omega * air_scale * mc.voltage_efficiency;
        let reaction_mag = mc.torque_coeff * omega * omega * air_scale;

        let thrust_vec = Vec3::new(0.0, thrust_mag, 0.0);
        total_thrust += thrust_vec;

        total_torque += mc.position.cross(thrust_vec);
        total_torque.y += mc.rotation_dir * reaction_mag;
    }

    (total_thrust, total_torque)
}

/// Mix desired thrust (N) + body torque (N*m) into per-motor RPM.
pub fn mix_thrust_torque(cfg: &DroneConfig, thrust: f32, torque: Vec3) -> [f32; 4] {
    let arm = cfg.arm_length.max(1e-4);
    let kt = cfg.motors[0].thrust_coeff.max(1e-9);
    let kq = cfg.motors[0].torque_coeff.max(1e-9);
    let k = kq / kt;

    let f0 = (thrust - torque.x / arm + torque.z / arm + torque.y / k.max(1e-6)) / 4.0;
    let f1 = (thrust - torque.x / arm - torque.z / arm - torque.y / k.max(1e-6)) / 4.0;
    let f2 = (thrust + torque.x / arm - torque.z / arm + torque.y / k.max(1e-6)) / 4.0;
    let f3 = (thrust + torque.x / arm + torque.z / arm - torque.y / k.max(1e-6)) / 4.0;

    let forces = [f0, f1, f2, f3];
    let mut rpms = [0.0; 4];
    for i in 0..4 {
        let f = forces[i].max(0.0);
        let omega = if kt > 0.0 { (f / kt).sqrt() } else { 0.0 };
        let rpm = omega * 60.0 / (2.0 * std::f32::consts::PI);
        rpms[i] = rpm.clamp(cfg.motors[i].min_rpm, cfg.motors[i].max_rpm);
    }
    rpms
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hover_thrust_yields_upward_force() {
        let cfg = DroneConfig::default();
        let env = EnvironmentConfig::default();
        // Thrust equal to weight should produce an upward force matching weight.
        let mass = cfg.mass;
        let weight = mass * 9.81;
        let rpms = mix_thrust_torque(&cfg, weight, Vec3::ZERO);
        let (thrust, _torque) = motor_forces_and_torques(
            &cfg,
            &env,
            &MotorRPMs {
                rpms,
                target_rpms: [0.0; 4],
            },
        );
        // Total thrust is in body frame +Y.
        assert!(
            thrust.y > weight * 0.9,
            "expected upward thrust near weight, got {:?}",
            thrust
        );
        // All four motors should spin for a hover command.
        assert!(
            rpms.iter().all(|&r| r > 0.0),
            "expected all motors spinning, got {:?}",
            rpms
        );
    }

    #[test]
    fn torque_creates_uneven_motor_rpms() {
        let cfg = DroneConfig::default();
        let base = mix_thrust_torque(&cfg, cfg.mass * 9.81, Vec3::ZERO);
        let rolled = mix_thrust_torque(&cfg, cfg.mass * 9.81, Vec3::new(5.0, 0.0, 0.0));
        // A roll torque should differentiate the left/right motor RPMs.
        assert!(
            (base[0] - rolled[0]).abs() > 1.0 || (base[1] - rolled[1]).abs() > 1.0,
            "expected torque to change motor RPMs, base={:?} rolled={:?}",
            base,
            rolled
        );
    }
}
