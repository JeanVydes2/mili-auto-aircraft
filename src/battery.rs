use crate::config::{BatteryConfig, BatteryState, DroneConfig};
use crate::motor_rpms::MotorRPMs;
use avian3d::prelude::LinearVelocity;
use bevy::prelude::*;

pub fn battery_system(
    time: Res<Time>,
    cfg: Res<BatteryConfig>,
    drone_cfg: Res<DroneConfig>,
    mut batt: ResMut<BatteryState>,
    env: Res<crate::config::EnvironmentConfig>,
    motor_rpms: Res<MotorRPMs>,
) {
    let dt = time.delta_secs().max(1e-5);
    // Compute current draw from motors + climb/acceleration power.
    let mut total_current = 0.0;
    for i in 0..4 {
        let mc = &drone_cfg.motors[i];
        let rpm = motor_rpms.rpms[i];
        let omega = rpm * (2.0 * std::f32::consts::PI / 60.0);
        let thrust = mc.thrust_coeff * omega * omega;
        // Mechanical power scales with thrust * vertical speed + hover.
        // We don't have linvel here, so we'll approximate.
        let hover_power = thrust * 0.15;
        let mech_power = hover_power * mc.position.length().max(0.1) * 0.1;
        let elec_power = mech_power / 0.75 + 2.0; // + idle
        total_current += elec_power / batt.voltage.max(1.0);
    }
    // Temperature effect on internal resistance.
    let r_int = cfg.internal_resistance * (1.0 + cfg.temp_coeff * (20.0 - env.temperature_c));
    let v_drop = total_current * r_int;
    let ocv = interpolate_ocv(&cfg, batt.soc);
    batt.voltage = (ocv - v_drop).max(0.0);
    batt.current_a = total_current;

    // Discharge.
    let capacity_ah = cfg.capacity_mah / 1000.0;
    if capacity_ah > 0.0 {
        batt.soc -= (total_current * time.delta_secs().max(1e-5) / 3600.0) / capacity_ah;
        batt.soc = batt.soc.clamp(0.0, 1.0);
    }
    batt.temperature_c = env.temperature_c + total_current * 0.1;
}

fn interpolate_ocv(cfg: &BatteryConfig, soc: f32) -> f32 {
    if cfg.discharge_curve.is_empty() {
        return cfg.nominal_voltage;
    }
    // soc descending? curve given soc high to low. Find segment.
    let mut prev = cfg.discharge_curve[0];
    for &pt in &cfg.discharge_curve {
        if soc >= pt.0 {
            // interpolate between prev and pt? Simpler nearest.
            return pt.1;
        }
        prev = pt;
    }
    prev.1
}
