use crate::config::{DomainRandomizationConfig, DroneConfig, EnvironmentConfig, SensorConfig};
use crate::motors::Motors;
use avian3d::prelude::*;
use bevy::prelude::*;
use rand::Rng;

#[derive(Resource, Default)]
pub struct ResetRequest(pub bool);

#[derive(Resource, Default)]
pub struct ResetGeneration(pub u64);

pub fn apply_domain_randomization(
    mut drone_cfg: ResMut<DroneConfig>,
    mut sensor_cfg: ResMut<SensorConfig>,
    mut env_cfg: ResMut<EnvironmentConfig>,
    dr: Res<DomainRandomizationConfig>,
) {
    if !dr.enabled {
        return;
    }
    let mut rng = rand::thread_rng();
    drone_cfg.mass = rng.gen_range(dr.mass_range.0..=dr.mass_range.1);
    let is = rng.gen_range(dr.inertia_scale_range.0..=dr.inertia_scale_range.1);
    drone_cfg.inertia *= is;
    let co = rng.gen_range(-dr.com_offset_range..=dr.com_offset_range);
    drone_cfg.center_of_mass_offset = Vec3::new(
        co,
        rng.gen_range(-dr.com_offset_range..=dr.com_offset_range),
        co,
    );
    let ms = rng.gen_range(dr.motor_strength_range.0..=dr.motor_strength_range.1);
    for m in &mut drone_cfg.motors {
        m.thrust_coeff *= ms;
        m.torque_coeff *= ms;
        m.time_constant =
            rng.gen_range(dr.motor_time_constant_range.0..=dr.motor_time_constant_range.1);
    }
    let ns = rng.gen_range(dr.sensor_noise_scale_range.0..=dr.sensor_noise_scale_range.1);
    sensor_cfg.imu_accel_noise *= ns;
    sensor_cfg.imu_gyro_noise *= ns;
    sensor_cfg.imu_accel_bias = Vec3::new(
        rng.gen_range(-dr.sensor_bias_range..=dr.sensor_bias_range),
        rng.gen_range(-dr.sensor_bias_range..=dr.sensor_bias_range),
        rng.gen_range(-dr.sensor_bias_range..=dr.sensor_bias_range),
    );
    sensor_cfg.imu_gyro_bias = Vec3::new(
        rng.gen_range(-dr.sensor_bias_range * 0.1..=dr.sensor_bias_range * 0.1),
        rng.gen_range(-dr.sensor_bias_range * 0.1..=dr.sensor_bias_range * 0.1),
        rng.gen_range(-dr.sensor_bias_range * 0.1..=dr.sensor_bias_range * 0.1),
    );
    env_cfg.wind_gust_intensity = rng.gen_range(dr.wind_range.0..=dr.wind_range.1);
    env_cfg.temperature_c = rng.gen_range(dr.temp_range.0..=dr.temp_range.1);
    env_cfg.update_air_density();
}

pub fn reset_aircraft_system(
    mut reset: ResMut<ResetRequest>,
    mut generation: ResMut<ResetGeneration>,
    mut rpms: ResMut<crate::motor_rpms::MotorRPMs>,
    mut telemetry: ResMut<crate::sensors::Telemetry>,
    mut mavlink: ResMut<crate::mavlink::MavlinkInterface>,
    mut prev_velocity: ResMut<crate::sensors::PrevVelocity>,
    mut query: Query<
        (
            &mut Transform,
            &mut Position,
            &mut Rotation,
            &mut LinearVelocity,
            &mut AngularVelocity,
            Option<&mut Motors>,
            Option<&mut crate::motors::CommandInput>,
            Option<&mut CollidingEntities>,
            Option<&mut crate::aircraft::Propulsion>,
            Option<&mut crate::aircraft::AircraftControls>,
        ),
        With<crate::aircraft::Aircraft>,
    >,
) {
    if !reset.0 {
        return;
    }
    for (
        mut tf,
        mut position,
        mut rotation,
        mut lv,
        mut av,
        motors,
        input,
        contacts,
        propulsion,
        controls,
    ) in &mut query
    {
        *tf = Transform::from_xyz(0.0, 2.0, 0.0);
        position.0 = tf.translation;
        rotation.0 = Quat::IDENTITY;
        lv.0 = Vec3::ZERO;
        av.0 = Vec3::ZERO;
        if let Some(mut motors) = motors {
            *motors = Motors::default();
        }
        if let Some(mut input) = input {
            input.command = crate::motors::DroneCommand::None;
        }
        if let Some(mut contacts) = contacts {
            contacts.clear();
        }
        if let Some(mut propulsion) = propulsion {
            for propulsor in &mut propulsion.0 {
                propulsor.throttle = 0.0;
            }
        }
        if let Some(mut controls) = controls {
            controls.propulsion.fill(0.0);
            controls.surfaces.fill(0.0);
        }
    }
    *rpms = crate::motor_rpms::MotorRPMs::default();
    mavlink.armed = false;
    mavlink.external_control = false;
    mavlink.guided_target = None;
    mavlink.last_velocity_setpoint_secs = None;
    *telemetry = crate::sensors::Telemetry {
        position: Vec3::new(0.0, 2.0, 0.0),
        rotation: Quat::IDENTITY,
        ..default()
    };
    prev_velocity.0 = Vec3::ZERO;
    generation.0 = generation.0.wrapping_add(1);
    reset.0 = false;
}
