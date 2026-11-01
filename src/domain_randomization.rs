use crate::config::{
    DomainRandomizationConfig, DroneConfig, EnvironmentConfig, NetworkConfig, SensorConfig,
};
use crate::motors::{Drone, Motors};
use avian3d::prelude::*;
use bevy::prelude::*;
use rand::Rng;

#[derive(Resource, Default)]
pub struct ResetRequest(pub bool);

pub fn apply_domain_randomization(
    mut drone_cfg: ResMut<DroneConfig>,
    mut sensor_cfg: ResMut<SensorConfig>,
    mut env_cfg: ResMut<EnvironmentConfig>,
    mut net_cfg: ResMut<NetworkConfig>,
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
    net_cfg.latency_ms = rng.gen_range(dr.latency_range.0..=dr.latency_range.1);
    net_cfg.packet_loss = rng.gen_range(dr.packet_loss_range.0..=dr.packet_loss_range.1);
}

pub fn reset_drone_system(
    mut reset: ResMut<ResetRequest>,
    mut query: Query<
        (
            &mut Transform,
            &mut LinearVelocity,
            &mut AngularVelocity,
            &mut Motors,
        ),
        With<Drone>,
    >,
) {
    if !reset.0 {
        return;
    }
    for (mut tf, mut lv, mut av, mut motors) in &mut query {
        tf.translation = Vec3::new(0.0, 2.0, 0.0);
        tf.rotation = Quat::IDENTITY;
        lv.0 = Vec3::ZERO;
        av.0 = Vec3::ZERO;
        for m in &mut motors.states {
            m.rpm = 0.0;
            m.target_rpm = 0.0;
        }
    }
    reset.0 = false;
}
