use crate::config::{BatteryConfig, BatteryState, DroneConfig, EnvironmentConfig};
use crate::motor_rpms::MotorRPMs;
use bevy::prelude::*;
const ELECTRICAL_EFFICIENCY: f32 = 0.85;
const IDLE_POWER_PER_MOTOR_W: f32 = 2.0;
const THERMAL_TIME_CONSTANT_S: f32 = 120.0;
const THERMAL_RESISTANCE_C_PER_W: f32 = 1.0;

pub fn battery_system(
    time: Res<Time>,
    cfg: Res<BatteryConfig>,
    drone_cfg: Res<DroneConfig>,
    env: Res<EnvironmentConfig>,
    mut query: Query<(&MotorRPMs, &mut BatteryState), With<crate::motors::Drone>>,
) {
    let dt = time.delta_secs();
    for (motor_rpms, mut batt) in &mut query {
        let power_w = electrical_power(&drone_cfg, &env, &motor_rpms.rpms);
        advance_battery(&cfg, &mut batt, &env, power_w, dt);
    }
}

fn electrical_power(drone: &DroneConfig, env: &EnvironmentConfig, rpms: &[f32; 4]) -> f32 {
    let density_scale = (env.air_density / 1.225).max(0.0);
    drone
        .motors
        .iter()
        .zip(rpms)
        .map(|(motor, rpm)| {
            let omega = rpm.abs() * (std::f32::consts::TAU / 60.0);
            let torque = motor.torque_coeff.max(0.0) * density_scale * omega.powi(2);
            torque * omega / ELECTRICAL_EFFICIENCY + IDLE_POWER_PER_MOTOR_W
        })
        .sum()
}

// Solve P = I * (OCV - I*R) on the low-current, high-voltage branch.
// Above the maximum deliverable power, saturate rather than inventing energy
// or allowing a negative terminal voltage. The RPM model handles the sag.
fn loaded_output(ocv: f32, resistance: f32, power_w: f32) -> (f32, f32) {
    let ocv = ocv.max(0.0);
    let resistance = resistance.max(0.0);
    let power_w = power_w.max(0.0);
    if ocv == 0.0 {
        return (0.0, 0.0);
    }
    if resistance == 0.0 {
        return (power_w / ocv, ocv);
    }
    let power_w = power_w.min(ocv * ocv / (4.0 * resistance));
    let discriminant = (ocv * ocv - 4.0 * resistance * power_w).max(0.0);
    // Rationalized root avoids cancellation at low loads.
    let current = 2.0 * power_w / (ocv + discriminant.sqrt());
    (current, (ocv - current * resistance).max(0.0))
}

fn advance_battery(
    cfg: &BatteryConfig,
    batt: &mut BatteryState,
    env: &EnvironmentConfig,
    power_w: f32,
    dt: f32,
) {
    if dt <= 0.0 {
        return;
    }
    batt.soc = batt.soc.clamp(0.0, 1.0);
    let resistance =
        (cfg.internal_resistance * (1.0 + cfg.temp_coeff * (20.0 - batt.temperature_c))).max(0.0);
    let capacity_ah = cfg.capacity_mah / 1000.0;
    if batt.soc > 0.0 && capacity_ah > 0.0 {
        let (current, _) = loaded_output(interpolate_ocv(cfg, batt.soc), resistance, power_w);
        batt.soc = (batt.soc - current * dt / (3600.0 * capacity_ah)).clamp(0.0, 1.0);
    } else {
        batt.soc = 0.0;
    }

    // Report terminal values for the updated SOC, not the previous frame's voltage.
    (batt.current_a, batt.voltage) = if batt.soc > 0.0 {
        loaded_output(interpolate_ocv(cfg, batt.soc), resistance, power_w)
    } else {
        (0.0, 0.0)
    };
    let heat_w = batt.current_a.powi(2) * resistance;
    let target_temperature = env.temperature_c + heat_w * THERMAL_RESISTANCE_C_PER_W;
    let thermal_alpha = -(-dt / THERMAL_TIME_CONSTANT_S).exp_m1();
    batt.temperature_c += (target_temperature - batt.temperature_c) * thermal_alpha;
}

fn interpolate_ocv(cfg: &BatteryConfig, soc: f32) -> f32 {
    let Some(&(first_soc, first_voltage)) = cfg.discharge_curve.first() else {
        return cfg.nominal_voltage;
    };
    if soc >= first_soc {
        return first_voltage;
    }
    for segment in cfg.discharge_curve.windows(2) {
        let (high_soc, high_voltage) = segment[0];
        let (low_soc, low_voltage) = segment[1];
        if soc >= low_soc && high_soc > low_soc {
            let fraction = ((soc - low_soc) / (high_soc - low_soc)).clamp(0.0, 1.0);
            return low_voltage + fraction * (high_voltage - low_voltage);
        }
    }
    cfg.discharge_curve.last().unwrap().1
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(actual: f32, expected: f32) {
        assert!((actual - expected).abs() < 1e-4, "{actual} != {expected}");
    }

    #[test]
    fn ocv_interpolates_descending_curve_and_clamps_endpoints() {
        let cfg = BatteryConfig::default();
        for (soc, voltage) in [
            (1.2, 16.8),
            (1.0, 16.8),
            (0.75, 15.9),
            (0.5, 15.0),
            (0.3, 14.1),
            (0.1, 13.2),
            (0.05, 12.6),
            (0.0, 12.0),
            (-0.2, 12.0),
        ] {
            assert_close(interpolate_ocv(&cfg, soc), voltage);
        }
    }

    #[test]
    fn ocv_handles_empty_single_point_and_duplicate_soc_curves() {
        let mut cfg = BatteryConfig::default();
        cfg.discharge_curve.clear();
        assert_close(interpolate_ocv(&cfg, 0.5), cfg.nominal_voltage);
        cfg.discharge_curve = vec![(0.5, 14.0)];
        assert_close(interpolate_ocv(&cfg, 0.0), 14.0);
        assert_close(interpolate_ocv(&cfg, 1.0), 14.0);
        cfg.discharge_curve = vec![(1.0, 16.8), (1.0, 16.8), (0.0, 12.0)];
        assert_close(interpolate_ocv(&cfg, 0.5), 14.4);
    }

    #[test]
    fn hover_consumes_rotor_power_plus_idle() {
        let drone = DroneConfig::default();
        let env = EnvironmentConfig::default();
        let cfg = BatteryConfig::default();
        let omega = (drone.mass * env.gravity / (4.0 * drone.motors[0].thrust_coeff)).sqrt();
        let rpm = omega * 60.0 / std::f32::consts::TAU;
        let idle = electrical_power(&drone, &env, &[0.0; 4]);
        let hover = electrical_power(&drone, &env, &[rpm; 4]);
        assert_close(idle, 8.0);
        let expected =
            4.0 * drone.motors[0].torque_coeff * omega.powi(3) / ELECTRICAL_EFFICIENCY + idle;
        assert!((hover - expected).abs() / expected < 1e-5);
        let mut batt = BatteryState::default();
        advance_battery(&cfg, &mut batt, &env, hover, 0.01);
        assert!(
            (5.0..30.0).contains(&batt.current_a),
            "hover current: {} A",
            batt.current_a
        );
        assert!(hover > idle);
    }

    #[test]
    fn rotor_power_scales_with_density_and_rpm_cubed() {
        let drone = DroneConfig::default();
        let mut env = EnvironmentConfig::default();
        let idle = electrical_power(&drone, &env, &[0.0; 4]);
        let power = electrical_power(&drone, &env, &[2000.0; 4]) - idle;
        assert_close(
            electrical_power(&drone, &env, &[4000.0; 4]) - idle,
            power * 8.0,
        );
        env.air_density *= 0.5;
        assert_close(
            electrical_power(&drone, &env, &[2000.0; 4]) - idle,
            power * 0.5,
        );
    }

    #[test]
    fn sag_is_self_consistent_and_overload_is_power_limited() {
        let (current, voltage) = loaded_output(16.0, 0.05, 400.0);
        assert_close(current * voltage, 400.0);
        assert_close(voltage, 16.0 - current * 0.05);
        assert!(voltage < 16.0 && voltage > 8.0);
        let (current, voltage) = loaded_output(16.0, 0.05, 100_000.0);
        assert_close(current, 160.0);
        assert_close(voltage, 8.0);
        assert_close(current * voltage, 1280.0);
        assert_eq!(loaded_output(0.0, 0.05, 400.0), (0.0, 0.0));
        assert_eq!(loaded_output(16.0, 0.0, 400.0), (25.0, 16.0));
    }

    #[test]
    fn voltage_and_soc_fall_with_elapsed_time() {
        let cfg = BatteryConfig {
            temp_coeff: 0.0,
            ..default()
        };
        let env = EnvironmentConfig::default();
        let mut batt = BatteryState::default();
        advance_battery(&cfg, &mut batt, &env, 400.0, 0.01);
        let initial_voltage = batt.voltage;
        let initial_soc = batt.soc;
        for _ in 0..600 {
            advance_battery(&cfg, &mut batt, &env, 400.0, 0.1);
        }
        assert!(batt.soc < initial_soc - 0.08);
        assert!(batt.voltage < initial_voltage);
        assert!(batt.voltage > 0.0);
        assert_close(batt.current_a * batt.voltage, 400.0);
    }

    #[test]
    fn discharge_uses_dt_and_paused_time_does_not_change_state() {
        let cfg = BatteryConfig {
            internal_resistance: 0.0,
            discharge_curve: vec![(1.0, 16.0), (0.0, 16.0)],
            ..default()
        };
        let env = EnvironmentConfig::default();
        let mut single = BatteryState::default();
        let mut split = single.clone();
        advance_battery(&cfg, &mut single, &env, 400.0, 10.0);
        for _ in 0..10 {
            advance_battery(&cfg, &mut split, &env, 400.0, 1.0);
        }
        assert_close(single.soc, 1.0 - 25.0 * 10.0 / 18000.0);
        assert_close(single.soc, split.soc);
        let before = single.clone();
        advance_battery(&cfg, &mut single, &env, 400.0, 0.0);
        assert_eq!(single.soc, before.soc);
        assert_eq!(single.voltage, before.voltage);
        assert_eq!(single.current_a, before.current_a);
        assert_eq!(single.temperature_c, before.temperature_c);
    }

    #[test]
    fn empty_battery_stops_supplying_power_without_negative_voltage() {
        let cfg = BatteryConfig::default();
        let env = EnvironmentConfig::default();
        let mut batt = BatteryState {
            soc: 0.00001,
            ..default()
        };
        advance_battery(&cfg, &mut batt, &env, 100_000.0, 60.0);
        assert_eq!(batt.soc, 0.0);
        assert_eq!(batt.current_a, 0.0);
        assert_eq!(batt.voltage, 0.0);
        advance_battery(&cfg, &mut batt, &env, 100_000.0, 60.0);
        assert_eq!(batt.current_a, 0.0);
        assert_eq!(batt.voltage, 0.0);
        assert!(batt.temperature_c.is_finite());
    }

    #[test]
    fn temperature_approaches_equilibrium_smoothly_and_cools_when_idle() {
        let cfg = BatteryConfig {
            temp_coeff: 0.0,
            ..default()
        };
        let env = EnvironmentConfig::default();
        let mut batt = BatteryState::default();
        advance_battery(&cfg, &mut batt, &env, 400.0, 1.0);
        let target = env.temperature_c + batt.current_a.powi(2) * cfg.internal_resistance;
        assert!(batt.temperature_c > env.temperature_c);
        assert!(batt.temperature_c < target);
        let warm = batt.temperature_c;
        advance_battery(&cfg, &mut batt, &env, 0.0, 1.0);
        assert!(batt.temperature_c < warm);
        assert!(batt.temperature_c > env.temperature_c);
    }
}
