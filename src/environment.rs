use crate::config::{EnvironmentConfig, SimConfig};
use crate::motors::Drone;
use avian3d::prelude::WriteRigidBodyForces;
use avian3d::prelude::*;
use bevy::prelude::*;

#[derive(Resource, Default, Clone, Copy)]
pub struct CurrentWind(pub Vec3);

#[derive(Resource, Default, Clone, Copy)]
pub struct Explosion {
    pub timer: f32,
    pub pos: Vec3,
    pub active: bool,
}

pub fn setup_environment(
    mut commands: Commands,
    mut meshes: Option<ResMut<Assets<Mesh>>>,
    mut materials: Option<ResMut<Assets<StandardMaterial>>>,
    sim: Res<SimConfig>,
    env: Res<EnvironmentConfig>,
) {
    // Ground collider (always).
    commands.spawn((
        RigidBody::Static,
        Collider::cuboid(50.0, 0.1, 50.0),
        Transform::from_xyz(0.0, -0.05, 0.0),
    ));

    if !sim.headless {
        if let (Some(m), Some(mat)) = (meshes.as_mut(), materials.as_mut()) {
            commands.spawn((
                Mesh3d(m.add(Plane3d::default().mesh().size(100.0, 100.0))),
                MeshMaterial3d(mat.add(Color::srgb(0.3, 0.3, 0.3))),
                Transform::from_xyz(0.0, -0.05, 0.0),
            ));

            for i in 0..5 {
                let x = (i as f32 - 2.0) * 4.0;
                commands.spawn((
                    RigidBody::Static,
                    Collider::cuboid(1.0, 1.0, 1.0),
                    Transform::from_xyz(x, 0.5, 2.0),
                    Mesh3d(m.add(Cuboid::new(1.0, 1.0, 1.0))),
                    MeshMaterial3d(mat.add(Color::srgb(0.6, 0.2, 0.2))),
                ));
            }
        }

        commands.spawn((
            DirectionalLight {
                illuminance: 10000.0,
                shadow_maps_enabled: false,
                ..default()
            },
            Transform::from_xyz(5.0, 10.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
        ));
    }

    let _ = env;
}

/// Collision -> explosion + reset.
pub fn collision_explosion_system(
    time: Res<Time>,
    mut reset: ResMut<crate::domain_randomization::ResetRequest>,
    mut explosion: ResMut<Explosion>,
    drone_query: Query<&Transform, With<Drone>>,
    obstacles: Query<&Transform, (With<Collider>, With<RigidBody>)>,
) {
    for drone_tf in &drone_query {
        for obs_tf in &obstacles {
            if obs_tf.translation.y < 0.2 {
                continue;
            } // skip ground
            if drone_tf.translation.distance(obs_tf.translation) < 1.2 {
                explosion.active = true;
                explosion.timer = 1.0;
                explosion.pos = drone_tf.translation;
                reset.0 = true;
            }
        }
    }
    if explosion.active {
        explosion.timer -= time.delta_secs();
        if explosion.timer <= 0.0 {
            explosion.active = false;
        }
    }
}

/// Draw explosion.
pub fn draw_explosion_system(mut gizmos: Gizmos, explosion: Res<Explosion>) {
    if !explosion.active {
        return;
    }
    let r = (1.0 - explosion.timer) * 3.0 + 0.1;
    gizmos.sphere(explosion.pos, r, Color::srgb(1.0, 0.3, 0.0));
}

/// Wind + turbulence system.
pub fn wind_gust_system(
    env: Res<EnvironmentConfig>,
    time: Res<Time>,
    mut wind_res: ResMut<CurrentWind>,
    mut query: Query<Forces, With<Drone>>,
) {
    let t = time.elapsed_secs();
    // 6 distinct wind directions.
    let dirs = [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(1.0, 0.0, 1.0).normalize(),
        Vec3::new(-1.0, 0.0, 1.0).normalize(),
    ];
    let mut wind = env.wind_constant;
    for (i, d) in dirs.iter().enumerate() {
        let phase = t * (0.25 + i as f32 * 0.12) + i as f32 * 1.3;
        let amp = env.wind_gust_intensity * (0.4 + 0.6 * (phase.sin() * 0.5 + 0.5));
        wind += *d * amp / 6.0;
    }
    // Turbulence.
    let turb = Vec3::new(
        (t * 1.3).sin() * env.turbulence_intensity,
        (t * 0.9).cos() * env.turbulence_intensity * 0.3,
        (t * 1.7).sin() * env.turbulence_intensity,
    );
    wind += turb;
    let rho = env.air_density;
    for mut forces in &mut query {
        let f = wind * rho * 0.5;
        forces.apply_force(f);
        let torque = forces.position().cross(f) * 0.1;
        forces.apply_torque(torque);
    }
    wind_res.0 = wind;
}
