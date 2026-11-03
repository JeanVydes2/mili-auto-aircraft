use crate::config::{EnvironmentConfig, SimConfig};
use avian3d::prelude::*;
use bevy::prelude::*;

/// Air velocity in world coordinates (m/s), sampled at the drone.
#[derive(Resource, Default, Clone, Copy)]
pub struct CurrentWind(pub Vec3);

#[derive(Component)]
pub struct Obstacle;

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
) {
    commands.spawn((
        RigidBody::Static,
        Collider::cuboid(100.0, 0.1, 100.0),
        Transform::from_xyz(0.0, -0.05, 0.0),
    ));
    // Headless and rendered flight use the same collision geometry.
    for i in 0..5 {
        let mut obstacle = commands.spawn((
            Obstacle,
            RigidBody::Static,
            Collider::cuboid(1.0, 1.0, 1.0),
            Transform::from_xyz((i as f32 - 2.0) * 4.0, 0.5, 2.0),
        ));
        if !sim.headless {
            if let (Some(m), Some(mat)) = (meshes.as_mut(), materials.as_mut()) {
                obstacle.insert((
                    Mesh3d(m.add(Cuboid::new(1.0, 1.0, 1.0))),
                    MeshMaterial3d(mat.add(Color::srgb(0.6, 0.2, 0.2))),
                ));
            }
        }
    }
    if !sim.headless {
        if let (Some(m), Some(mat)) = (meshes.as_mut(), materials.as_mut()) {
            commands.spawn((
                Mesh3d(m.add(Plane3d::default().mesh().size(100.0, 100.0))),
                MeshMaterial3d(mat.add(Color::srgb(0.3, 0.3, 0.3))),
                Transform::IDENTITY,
            ));
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
}

/// Actual obstacle contacts trigger a crash; the solver handles ground landings.
pub fn collision_explosion_system(
    time: Res<Time>,
    mut reset: ResMut<crate::domain_randomization::ResetRequest>,
    mut explosion: ResMut<Explosion>,
    drone_query: Query<(&Transform, &CollidingEntities), With<crate::aircraft::Aircraft>>,
    obstacles: Query<(), (With<Obstacle>, Without<crate::aircraft::Aircraft>)>,
) {
    explosion.timer = (explosion.timer - time.delta_secs()).max(0.0);
    explosion.active = explosion.timer > 0.0;
    for (tf, contacts) in &drone_query {
        if contacts.iter().any(|entity| obstacles.contains(*entity)) {
            explosion.active = true;
            explosion.timer = 1.0;
            explosion.pos = tf.translation;
            reset.0 = true;
        }
    }
}

pub fn draw_explosion_system(mut gizmos: Gizmos, explosion: Res<Explosion>) {
    if explosion.active {
        let r = (1.0 - explosion.timer) * 3.0 + 0.1;
        gizmos.sphere(explosion.pos, r, Color::srgb(1.0, 0.3, 0.0));
    }
}

/// Smooth, reproducible gusts and spatial turbulence. All strengths are m/s;
/// duration sets the gust timescale and length_scale its spatial correlation.
pub fn wind_velocity(env: &EnvironmentConfig, time: f32, position: Vec3) -> Vec3 {
    let phase = std::f32::consts::TAU * time / env.wind_gust_duration.max(0.1);
    let gust = (0.5 + 0.5 * phase.sin()).powi(2) * env.wind_gust_intensity.max(0.0);
    let p = position / env.turbulence_length_scale.max(0.1);
    let t = time * 0.7;
    let turbulence = Vec3::new(
        ((t + p.z).sin() + 0.5 * (1.73 * t + p.y + 0.8).sin()) / 1.5,
        ((0.83 * t + p.x + 1.2).sin() + 0.5 * (1.91 * t + p.z).sin()) / 1.5 * 0.3,
        ((1.13 * t + p.y + 2.4).sin() + 0.5 * (2.17 * t + p.x).sin()) / 1.5,
    ) * env.turbulence_intensity.max(0.0);
    env.wind_constant + env.wind_gust_direction.normalize_or_zero() * gust + turbulence
}

pub fn wind_gust_system(
    env: Res<EnvironmentConfig>,
    time: Res<Time>,
    mut wind: ResMut<CurrentWind>,
    drone: Query<&Transform, With<crate::aircraft::Aircraft>>,
) {
    let position = drone.iter().next().map_or(Vec3::ZERO, |tf| tf.translation);
    wind.0 = wind_velocity(&env, time.elapsed_secs(), position);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain_randomization::ResetRequest;

    #[test]
    fn only_real_obstacle_contacts_request_reset() {
        let mut app = App::new();
        app.init_resource::<Time>()
            .init_resource::<ResetRequest>()
            .init_resource::<Explosion>()
            .add_systems(Update, collision_explosion_system);
        let obstacle = app.world_mut().spawn(Obstacle).id();
        let ground = app.world_mut().spawn_empty().id();
        let drone = app
            .world_mut()
            .spawn((
                crate::aircraft::Aircraft,
                Transform::from_xyz(0.0, 2.0, 0.0),
                CollidingEntities::default(),
            ))
            .id();
        // Neither proximity nor ground contact is an obstacle collision.
        app.world_mut()
            .get_mut::<CollidingEntities>(drone)
            .unwrap()
            .insert(ground);
        app.update();
        assert!(!app.world().resource::<ResetRequest>().0);
        app.world_mut()
            .get_mut::<CollidingEntities>(drone)
            .unwrap()
            .insert(obstacle);
        app.update();
        assert!(app.world().resource::<ResetRequest>().0);
        assert!(app.world().resource::<Explosion>().active);
    }

    #[test]
    fn wind_is_smooth_bounded_and_respects_configuration() {
        let mut env = EnvironmentConfig {
            wind_constant: Vec3::new(2.0, 0.0, -1.0),
            wind_gust_intensity: 3.0,
            wind_gust_direction: Vec3::Z,
            turbulence_intensity: 0.5,
            ..default()
        };
        let position = Vec3::new(1.0, 4.0, 3.0);
        for step in 0..2400 {
            let t = step as f32 / 240.0;
            let wind = wind_velocity(&env, t, position);
            let next = wind_velocity(&env, t + 1.0 / 240.0, position);
            assert!(wind.is_finite());
            assert!((next - wind).length() < 0.1);
            assert!((wind - env.wind_constant).length() < 4.0);
        }
        env.wind_gust_intensity = 0.0;
        env.turbulence_intensity = 0.0;
        assert_eq!(wind_velocity(&env, 1.0, position), env.wind_constant);
    }
}
