use crate::aircraft::Aircraft;
use crate::config::{BatteryState, EnvironmentConfig, SensorConfig};
use crate::motor_rpms::MotorRPMs;
use avian3d::prelude::*;
use bevy::prelude::*;
use rand::Rng;
use rand_distr::{Distribution, Normal};

#[derive(Resource, Default, Clone, Debug)]
pub struct Telemetry {
    pub position: Vec3,
    pub rotation: Quat,
    pub velocity: Vec3,
    pub angular_velocity: Vec3,
    pub accel_body: Vec3,
    pub gyro_body: Vec3,
    pub mag_body: Vec3,
    pub baro_altitude: f32,
    pub gps_position: Vec3,
    pub gps_valid: bool,
    pub battery_voltage: f32,
    pub battery_current: f32,
    pub motor_rpms: [f32; 4],
}

#[derive(Resource, Default)]
pub struct PrevVelocity(pub Vec3);

pub fn update_telemetry_system(
    time: Res<Time>,
    env: Res<EnvironmentConfig>,
    sensor_cfg: Res<SensorConfig>,
    batt: Res<BatteryState>,
    motor_rpms: Res<MotorRPMs>,
    mut prev: ResMut<PrevVelocity>,
    mut telemetry: ResMut<Telemetry>,
    query: Query<(&Transform, &LinearVelocity, &AngularVelocity), With<Aircraft>>,
) {
    let dt = time.delta_secs().max(1e-4);
    for (tf, linvel, angvel) in &query {
        let rot = tf.rotation;
        let accel_world = (linvel.0 - prev.0) / dt;
        prev.0 = linvel.0;

        let gravity = Vec3::new(0.0, -env.gravity, 0.0);
        let specific_force_world = accel_world - gravity;

        let mut rng = rand::thread_rng();
        let accel_noise = Normal::new(0.0, sensor_cfg.imu_accel_noise).unwrap();
        let gyro_noise = Normal::new(0.0, sensor_cfg.imu_gyro_noise).unwrap();
        let baro_noise = Normal::new(0.0, sensor_cfg.baro_noise).unwrap();
        let mag_noise = Normal::new(0.0, sensor_cfg.mag_noise).unwrap();
        let gps_noise = Normal::new(0.0, sensor_cfg.gps_pos_noise).unwrap();

        // Motor vibration on IMU - use resource RPMs for vibration.
        let vibe = motor_rpms.rpms.iter().sum::<f32>() / 4.0 * sensor_cfg.imu_vibe * 1e-3;

        let accel_body = rot.inverse() * specific_force_world
            + sensor_cfg.imu_accel_bias
            + Vec3::new(
                accel_noise.sample(&mut rng),
                accel_noise.sample(&mut rng),
                accel_noise.sample(&mut rng),
            )
            + Vec3::new(vibe, 0.0, vibe);
        let gyro_body = rot.inverse() * angvel.0
            + sensor_cfg.imu_gyro_bias
            + Vec3::new(
                gyro_noise.sample(&mut rng),
                gyro_noise.sample(&mut rng),
                gyro_noise.sample(&mut rng),
            );
        let baro = tf.translation.y + baro_noise.sample(&mut rng);
        let mag_body = rot.inverse() * Vec3::new(0.0, 1.0, 0.0) * 50.0 // dummy field
            + Vec3::new(mag_noise.sample(&mut rng), mag_noise.sample(&mut rng), mag_noise.sample(&mut rng))
            + sensor_cfg.mag_bias;

        // GPS with dropout.
        let gps_valid = rng.r#gen::<f32>() > sensor_cfg.gps_dropout_prob;
        let gps_pos = if gps_valid {
            tf.translation
                + Vec3::new(
                    gps_noise.sample(&mut rng),
                    gps_noise.sample(&mut rng) * 0.5,
                    gps_noise.sample(&mut rng),
                )
        } else {
            telemetry.gps_position
        };

        telemetry.position = tf.translation;
        telemetry.rotation = rot;
        telemetry.velocity = linvel.0;
        telemetry.angular_velocity = angvel.0;
        telemetry.accel_body = accel_body;
        telemetry.gyro_body = gyro_body;
        telemetry.mag_body = mag_body;
        telemetry.baro_altitude = baro;
        telemetry.gps_position = gps_pos;
        telemetry.gps_valid = gps_valid;
        telemetry.battery_voltage = batt.voltage;
        telemetry.battery_current = batt.current_a;
        telemetry.motor_rpms = motor_rpms.rpms;
        let elapsed = time.elapsed_secs();
        if elapsed.floor() != (elapsed - dt).floor() {
            bevy::log::info!(
                "drone pos {:?} vel {:?} rot {:?} rpms {:?}",
                telemetry.position,
                telemetry.velocity,
                telemetry.rotation.to_euler(EulerRot::XYZ),
                telemetry.motor_rpms
            );
        }
    }
}
