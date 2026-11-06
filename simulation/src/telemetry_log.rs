use crate::config::{DroneConfig, GuidanceVisualsEnabled};
use crate::environment::CurrentWind;
use crate::mavlink::{GuidedTarget, MavlinkInterface};
use crate::motor_rpms::MotorRPMs;
use crate::motors::Drone;
use bevy::prelude::Gizmos;
use bevy::prelude::*;

pub fn draw_gizmos_system(
    mut gizmos: Gizmos,
    enabled: Res<crate::config::GizmosEnabled>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut guidance: ResMut<GuidanceVisualsEnabled>,
    query: Query<(&Transform, &MotorRPMs), With<Drone>>,
    cfg: Res<DroneConfig>,
    wind: Res<CurrentWind>,
    aircraft: Query<(&Transform, &MavlinkInterface), With<crate::aircraft::Aircraft>>,
) {
    if keyboard.just_pressed(KeyCode::KeyH) {
        guidance.0 = !guidance.0;
    }
    if !enabled.0 {
        if !guidance.0 {
            return;
        }
    }
    for (tf, motor_rpms) in &query {
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

    if !guidance.0 {
        return;
    }
    let grid_color = Color::srgba(0.12, 0.45, 0.55, 0.55);
    for axis in -10..=10 {
        let offset = axis as f32 * 2.0;
        gizmos.line(
            Vec3::new(-20.0, 0.01, offset),
            Vec3::new(20.0, 0.01, offset),
            grid_color,
        );
        gizmos.line(
            Vec3::new(offset, 0.012, -20.0),
            Vec3::new(offset, 0.012, 20.0),
            grid_color,
        );
    }
    gizmos.line(Vec3::ZERO, Vec3::new(22.0, 0.0, 0.0), Color::srgb(1.0, 0.2, 0.2));
    gizmos.line(Vec3::ZERO, Vec3::new(0.0, 22.0, 0.0), Color::srgb(0.2, 1.0, 0.3));
    gizmos.line(Vec3::ZERO, Vec3::new(0.0, 0.0, 22.0), Color::srgb(0.2, 0.55, 1.0));

    if let Some((drone_tf, interface)) = aircraft.iter().next()
        && let Some(GuidedTarget::Position { position, .. }) = interface.guided_target
    {
        let distance = drone_tf.translation.distance(position);
        let label = format!("DIST REMAINING  {:.1} m", distance);
        gizmos.line(position, drone_tf.translation, Color::srgb(1.0, 0.67, 0.18));
        gizmos.sphere(position, 0.28, Color::srgb(1.0, 0.67, 0.18));
        gizmos.text(
            Isometry3d::from_translation(position + Vec3::Y * 0.65),
            &label,
            18.0,
            Vec2::new(-75.0, 0.0),
            Color::srgb(1.0, 0.67, 0.18),
        );
    }
}
