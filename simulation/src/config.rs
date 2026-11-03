use bevy::prelude::*;

/// Top-level simulation configuration.
#[derive(Resource, Clone, Debug)]
pub struct SimConfig {
    pub headless: bool,
    pub fixed_dt: f32,
    pub gravity: f32,
    pub enable_gizmos: bool,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            headless: false,
            fixed_dt: 1.0 / 240.0,
            gravity: 9.81,
            enable_gizmos: true,
        }
    }
}

/// Resource to enable/disable gizmos globally.
#[derive(Resource, Default)]
pub struct GizmosEnabled(pub bool);

#[derive(Resource, Clone, Copy, Debug)]
pub struct FlightControlLimits {
    pub max_speed_m_s: f32,
    pub max_acceleration_m_s2: f32,
    pub max_tilt_deg: f32,
}

impl Default for FlightControlLimits {
    fn default() -> Self {
        Self {
            max_speed_m_s: 8.0,
            max_acceleration_m_s2: 8.0,
            max_tilt_deg: 26.0,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct MotorConfig {
    pub position: Vec3,
    pub rotation_dir: f32, // +1 CW, -1 CCW (reaction sign)
    pub thrust_coeff: f32, // kt
    pub torque_coeff: f32, // kq
    pub min_rpm: f32,
    pub max_rpm: f32,
    pub time_constant: f32, // first-order (s)
    pub second_order: bool,
    pub damping: f32,
    pub rate_limit_rpm_per_s: f32,
    pub voltage_efficiency: f32,
}

impl Default for MotorConfig {
    fn default() -> Self {
        Self {
            position: Vec3::ZERO,
            rotation_dir: 1.0,
            thrust_coeff: 1.0e-5,
            torque_coeff: 1.5e-7,
            min_rpm: 0.0,
            max_rpm: 9000.0,
            time_constant: 0.02,
            second_order: false,
            damping: 0.8,
            rate_limit_rpm_per_s: 50000.0,
            voltage_efficiency: 1.0,
        }
    }
}

#[derive(Resource, Clone, Debug)]
pub struct DroneConfig {
    pub mass: f32,
    pub center_of_mass_offset: Vec3,
    pub inertia: Vec3,          // Ixx,Iyy,Izz
    pub inertia_products: Vec3, // Ixy,Ixz,Iyz
    pub arm_length: f32,
    pub motors: [MotorConfig; 4],
    pub linear_drag: Vec3, // Cx,Cy,Cz
    pub angular_drag: Vec3,
    pub induced_drag: f32,
    pub ground_effect_height: f32,
    pub ground_effect_coeff: f32,
    pub gyroscopic_effect: bool,
    pub arm_flex: f32,
}

impl Default for DroneConfig {
    fn default() -> Self {
        let m = MotorConfig::default();
        let mut motors = [m; 4];
        let arm = 0.22f32;
        motors[0].position = Vec3::new(arm, 0.0, arm);
        motors[1].position = Vec3::new(-arm, 0.0, arm);
        motors[2].position = Vec3::new(-arm, 0.0, -arm);
        motors[3].position = Vec3::new(arm, 0.0, -arm);
        motors[0].rotation_dir = 1.0;
        motors[1].rotation_dir = -1.0;
        motors[2].rotation_dir = 1.0;
        motors[3].rotation_dir = -1.0;
        Self {
            mass: 1.5,
            center_of_mass_offset: Vec3::ZERO,
            inertia: Vec3::new(0.029, 0.055, 0.029),
            inertia_products: Vec3::ZERO,
            arm_length: arm,
            motors,
            linear_drag: Vec3::new(0.1, 0.1, 0.1),
            angular_drag: Vec3::new(0.05, 0.05, 0.05),
            induced_drag: 0.01,
            ground_effect_height: 0.5,
            ground_effect_coeff: 0.3,
            gyroscopic_effect: true,
            arm_flex: 0.0,
        }
    }
}

#[derive(Resource, Clone, Debug)]
pub struct EnvironmentConfig {
    pub gravity: f32,
    pub air_density: f32,
    pub temperature_c: f32,
    pub pressure_pa: f32,
    pub humidity: f32,
    pub wind_constant: Vec3,      // World air velocity (m/s).
    pub wind_gust_intensity: f32, // Peak gust speed (m/s).
    pub wind_gust_duration: f32,  // Gust period (s).
    pub wind_gust_direction: Vec3,
    pub turbulence_intensity: f32, // Turbulence speed amplitude (m/s).
    pub turbulence_length_scale: f32, // Spatial correlation length (m).
    pub altitude_m: f32,
}

impl Default for EnvironmentConfig {
    fn default() -> Self {
        Self {
            gravity: 9.81,
            air_density: 1.225,
            temperature_c: 20.0,
            pressure_pa: 101325.0,
            humidity: 0.5,
            wind_constant: Vec3::new(1.0, 0.0, 0.5),
            wind_gust_intensity: 1.5,
            wind_gust_duration: 6.0,
            wind_gust_direction: Vec3::X,
            turbulence_intensity: 0.35,
            turbulence_length_scale: 5.0,
            altitude_m: 0.0,
        }
    }
}

impl EnvironmentConfig {
    /// Update air density from temperature/pressure/altitude.
    pub fn update_air_density(&mut self) {
        // ISA-ish: rho = p/(R*T). Use temperature C.
        let t_k = self.temperature_c + 273.15;
        if t_k > 0.0 {
            self.air_density = self.pressure_pa / (287.05 * t_k);
        }
    }
}

#[derive(Resource, Clone, Debug)]
pub struct BatteryConfig {
    pub capacity_mah: f32,
    pub nominal_voltage: f32,
    pub internal_resistance: f32,
    pub temp_coeff: f32,
    pub discharge_curve: Vec<(f32, f32)>, // (soc, voltage)
}

impl Default for BatteryConfig {
    fn default() -> Self {
        Self {
            capacity_mah: 5000.0,
            nominal_voltage: 14.8,
            internal_resistance: 0.05,
            temp_coeff: 0.005,
            discharge_curve: vec![(1.0, 16.8), (0.5, 15.0), (0.1, 13.2), (0.0, 12.0)],
        }
    }
}

#[derive(Resource, Clone, Debug)]
pub struct BatteryState {
    pub soc: f32, // 0..1
    pub current_a: f32,
    pub voltage: f32,
    pub temperature_c: f32,
}

impl Default for BatteryState {
    fn default() -> Self {
        Self {
            soc: 1.0,
            current_a: 0.0,
            voltage: 14.8,
            temperature_c: 20.0,
        }
    }
}

#[derive(Resource, Clone, Debug)]
pub struct SensorConfig {
    // IMU
    pub imu_rate_hz: f32,
    pub imu_accel_noise: f32,
    pub imu_gyro_noise: f32,
    pub imu_accel_bias: Vec3,
    pub imu_gyro_bias: Vec3,
    pub imu_vibe: f32,
    // Magnetometer
    pub mag_noise: f32,
    pub mag_bias: Vec3,
    // Barometer
    pub baro_rate_hz: f32,
    pub baro_noise: f32,
    pub baro_drift: f32,
    // GPS
    pub gps_rate_hz: f32,
    pub gps_pos_noise: f32,
    pub gps_vel_noise: f32,
    pub gps_dropout_prob: f32,
    pub gps_multipath: f32,
    // Current/voltage
    pub current_noise: f32,
    pub voltage_noise: f32,
    // Camera
    pub camera_width: u32,
    pub camera_height: u32,
    pub camera_fov_deg: f32,
    pub camera_fps: f32,
    pub camera_distortion: f32,
    pub camera_motion_blur: f32,
    pub camera_rolling_shutter: f32,
    pub camera_exposure: f32,
    pub camera_iso_noise: f32,
    // Sensor delay/failure
    pub sensor_delay_ms: f32,
    pub sensor_dropout_prob: f32,
}

impl Default for SensorConfig {
    fn default() -> Self {
        Self {
            imu_rate_hz: 200.0,
            imu_accel_noise: 0.05,
            imu_gyro_noise: 0.005,
            imu_accel_bias: Vec3::ZERO,
            imu_gyro_bias: Vec3::ZERO,
            imu_vibe: 0.02,
            mag_noise: 0.5,
            mag_bias: Vec3::ZERO,
            baro_rate_hz: 50.0,
            baro_noise: 0.1,
            baro_drift: 0.01,
            gps_rate_hz: 10.0,
            gps_pos_noise: 1.0,
            gps_vel_noise: 0.1,
            gps_dropout_prob: 0.0,
            gps_multipath: 0.0,
            current_noise: 0.05,
            voltage_noise: 0.05,
            camera_width: 640,
            camera_height: 360,
            camera_fov_deg: 90.0,
            camera_fps: 30.0,
            camera_distortion: 0.0,
            camera_motion_blur: 0.0,
            camera_rolling_shutter: 0.0,
            camera_exposure: 1.0,
            camera_iso_noise: 0.0,
            sensor_delay_ms: 0.0,
            sensor_dropout_prob: 0.0,
        }
    }
}

#[derive(Resource, Clone, Debug)]
pub struct CameraConfig {
    pub offset: Vec3,
    pub rotation: Quat,
    pub mode: i32, // 0 fpv,1 chase,2 free
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            offset: Vec3::new(0.0, 0.1, 0.0),
            rotation: Quat::IDENTITY,
            mode: 0,
        }
    }
}

#[derive(Resource, Clone, Debug)]
pub struct DomainRandomizationConfig {
    pub enabled: bool,
    pub mass_range: (f32, f32),
    pub inertia_scale_range: (f32, f32),
    pub com_offset_range: f32,
    pub motor_strength_range: (f32, f32),
    pub motor_time_constant_range: (f32, f32),
    pub sensor_noise_scale_range: (f32, f32),
    pub sensor_bias_range: f32,
    pub wind_range: (f32, f32),
    pub temp_range: (f32, f32),
    pub lighting_range: (f32, f32),
}

impl Default for DomainRandomizationConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            mass_range: (1.2, 1.8),
            inertia_scale_range: (0.9, 1.1),
            com_offset_range: 0.02,
            motor_strength_range: (0.9, 1.1),
            motor_time_constant_range: (0.015, 0.03),
            sensor_noise_scale_range: (0.8, 1.2),
            sensor_bias_range: 0.1,
            wind_range: (0.0, 5.0),
            temp_range: (0.0, 40.0),
            lighting_range: (0.5, 1.5),
        }
    }
}
