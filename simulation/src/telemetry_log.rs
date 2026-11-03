use crate::config::DroneConfig;
use crate::environment::CurrentWind;
use crate::motor_rpms::MotorRPMs;
use crate::motors::Drone;
use bevy::prelude::Gizmos;
use bevy::prelude::*;

pub fn draw_gizmos_system(
    mut gizmos: Gizmos,
    enabled: Res<crate::config::GizmosEnabled>,
    motor_rpms: Res<MotorRPMs>,
    query: Query<&Transform, With<Drone>>,
    cfg: Res<DroneConfig>,
    wind: Res<CurrentWind>,
) {
    if !enabled.0 {
        return;
    }
    for tf in &query {
        let rot = tf.rotation;
        for i in 0..4 {
            let motor_pos = tf.translation + rot * cfg.motors[i].position;
            let rpm = motor_rpms.rpms[i];
            let thrust =
                cfg.motors[i].thrust_coeff * (rpm * (2.0 * std::f32::consts::PI / 60.0)).powi(2);
            // motor marker (larger)
            gizmos.sphere(motor_pos, 0.12, Color::srgb(1.0, 1.0, 0.0));
            // thrust arrow
            gizmos.line(
                motor_pos,
                motor_pos + rot * Vec3::new(0.0, thrust * 0.0005, 0.0),
                Color::srgb(0.0, 1.0, 0.0),
            );
        }
        // Front indicator (blue arrow).
        gizmos.line(
            tf.translation,
            tf.translation + rot * Vec3::new(0.0, 0.0, 1.5),
            Color::srgb(0.0, 0.0, 1.0),
        );
    }

    // Wind visualization: grid of arrows colored by magnitude (red high, green low).
    let mag = wind.0.length();
    let color = if mag > 2.0 {
        Color::srgb(1.0, 0.0, 0.0)
    } else if mag > 1.0 {
        Color::srgb(1.0, 0.5, 0.0)
    } else {
        Color::srgb(0.0, 1.0, 0.0)
    };
    for x in -5..=5 {
        for z in -5..=5 {
            let pos = Vec3::new(x as f32 * 2.0, 1.0, z as f32 * 2.0);
            let end = pos + wind.0 * 0.5;
            gizmos.line(pos, end, color);
        }
    }
}
