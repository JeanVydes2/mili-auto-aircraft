use crate::config::{EnvironmentConfig, SimConfig};
use avian3d::prelude::*;
use bevy::prelude::*;
use rand::{Rng, SeedableRng, rngs::StdRng};

/// Air velocity in world coordinates (m/s), sampled at the drone.
#[derive(Resource, Default, Clone, Copy)]
pub struct CurrentWind(pub Vec3);

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum WindPreset {
    Calm,
    #[default]
    Light,
    Moderate,
    Strong,
    Tornado,
}

impl WindPreset {
    pub fn label(self) -> &'static str {
        match self {
            Self::Calm => "CALM",
            Self::Light => "LIGHT",
            Self::Moderate => "MODERATE",
            Self::Strong => "STRONG",
            Self::Tornado => "TORNADO",
        }
    }
    pub fn base_speed(self) -> f32 {
        match self { Self::Calm => 0.0, Self::Light => 2.0, Self::Moderate => 6.0, Self::Strong => 12.0, Self::Tornado => 8.0 }
    }
    pub fn gust_speed(self) -> f32 {
        match self { Self::Calm => 0.0, Self::Light => 1.5, Self::Moderate => 4.0, Self::Strong => 8.0, Self::Tornado => 14.0 }
    }
    pub fn turbulence(self) -> f32 {
        match self { Self::Calm => 0.0, Self::Light => 0.35, Self::Moderate => 1.0, Self::Strong => 2.0, Self::Tornado => 4.0 }
    }
}

#[derive(Resource, Clone, Copy, Debug, Default)]
pub struct WindSettings(pub WindPreset);

#[derive(Component)]
pub struct Obstacle;

/// Minimum impact speed (m/s) for a collision to explode the drone.
/// Below it, contact only brakes/lands (soft landing).
pub const CRASH_SPEED_M_S: f32 = 2.5;

#[derive(Resource, Default, Clone, Copy)]
pub struct Explosion {
    pub timer: f32,
    pub pos: Vec3,
    pub active: bool,
    burst: u64,
}

#[derive(Resource)]
pub struct ExplosionAssets {
    sphere: Handle<Mesh>,
    flash: Handle<StandardMaterial>,
    flame: Handle<StandardMaterial>,
    spark: Handle<StandardMaterial>,
    smoke: Handle<StandardMaterial>,
}

#[derive(Clone, Copy)]
pub(super) enum ExplosionParticleKind {
    Flash,
    Fire,
    Spark,
    Smoke,
}

#[derive(Component)]
pub(super) struct ExplosionParticle {
    velocity: Vec3,
    age: f32,
    lifetime: f32,
    start_scale: f32,
    end_scale: f32,
    kind: ExplosionParticleKind,
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
    // The ruined city: damaged buildings (half standing, half collapsed
    // to rubble) + craters. Each building has a real collider and the
    // Obstacle marker: hitting any of them explodes the drone.
    spawn_ruined_city(
        &mut commands,
        sim.headless,
        meshes.as_deref_mut(),
        materials.as_deref_mut(),
    );
    // The 5 original cubes are kept as test obstacles in the z=2 row
    // (compatibility with the existing crash test).
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
            commands.spawn((
                Mesh3d(m.add(Cuboid::new(8.0, 0.04, 60.0))),
                MeshMaterial3d(mat.add(Color::srgb(0.055, 0.065, 0.075))),
                Transform::from_xyz(-28.0, 0.02, 0.0),
            ));
            for z in (-24..=24).step_by(8) {
                commands.spawn((
                    Mesh3d(m.add(Cuboid::new(0.25, 0.045, 3.0))),
                    MeshMaterial3d(mat.add(Color::srgb(0.72, 0.68, 0.45))),
                    Transform::from_xyz(-28.0, 0.045, z as f32),
                ));
            }
            commands.insert_resource(ExplosionAssets {
                sphere: m.add(Sphere::new(1.0).mesh().uv(12, 8)),
                flash: mat.add(StandardMaterial {
                    base_color: Color::srgb(1.0, 0.72, 0.28),
                    emissive: LinearRgba::new(9.0, 4.0, 0.8, 1.0),
                    ..default()
                }),
                flame: mat.add(StandardMaterial {
                    base_color: Color::srgb(1.0, 0.24, 0.025),
                    emissive: LinearRgba::new(4.0, 0.65, 0.04, 1.0),
                    ..default()
                }),
                spark: mat.add(StandardMaterial {
                    base_color: Color::srgb(1.0, 0.76, 0.25),
                    emissive: LinearRgba::new(7.0, 3.2, 0.35, 1.0),
                    ..default()
                }),
                smoke: mat.add(StandardMaterial {
                    base_color: Color::srgba(0.13, 0.14, 0.16, 0.72),
                    alpha_mode: AlphaMode::Blend,
                    perceptual_roughness: 1.0,
                    ..default()
                }),
            });
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

/// An impact against an obstacle (building, rubble or cube) explodes
/// the drone only if the speed exceeds CRASH_SPEED_M_S. Below it, the
/// contact is a soft landing/bump: the solver brakes it without explosion.
/// The ground (no Obstacle) never explodes: landing is safe.
pub fn collision_explosion_system(
    time: Res<Time>,
    sim: Res<SimConfig>,
    mut reset: ResMut<crate::domain_randomization::ResetRequest>,
    mut explosion: ResMut<Explosion>,
    assets: Option<Res<ExplosionAssets>>,
    mut commands: Commands,
    drone_query: Query<
        (&Transform, &CollidingEntities, Option<&LinearVelocity>),
        With<crate::aircraft::Aircraft>,
    >,
    obstacles: Query<(), (With<Obstacle>, Without<crate::aircraft::Aircraft>)>,
) {
    explosion.timer = (explosion.timer - time.delta_secs()).max(0.0);
    explosion.active = explosion.timer > 0.0;
    for (tf, contacts, velocity) in &drone_query {
        let speed = velocity.map_or(0.0, |v| v.0.length());
        if speed >= CRASH_SPEED_M_S && contacts.iter().any(|entity| obstacles.contains(*entity)) {
            explosion.active = true;
            explosion.timer = 2.4;
            explosion.pos = tf.translation;
            explosion.burst = explosion.burst.wrapping_add(1);
            if !sim.headless
                && let Some(assets) = assets.as_deref()
            {
                spawn_explosion(
                    &mut commands,
                    assets,
                    tf.translation,
                    velocity.map_or(Vec3::ZERO, |velocity| velocity.0),
                    explosion.burst,
                );
            }
            reset.0 = true;
        }
    }
}

fn spawn_explosion(
    commands: &mut Commands,
    assets: &ExplosionAssets,
    position: Vec3,
    impact_velocity: Vec3,
    burst: u64,
) {
    let mut rng = StdRng::seed_from_u64(0x4558_504c_4f53_494f ^ burst);
    spawn_particle(
        commands,
        assets,
        position,
        impact_velocity * 0.15,
        0.24,
        0.18,
        2.8,
        ExplosionParticleKind::Flash,
    );

    for _ in 0..14 {
        let direction = random_direction(&mut rng);
        let speed = rng.gen_range(1.2..4.0);
        spawn_particle(
            commands,
            assets,
            position + direction * rng.gen_range(0.0..0.18),
            impact_velocity * 0.2 + direction * speed + Vec3::Y * 1.5,
            rng.gen_range(0.16..0.34),
            rng.gen_range(0.42..0.72),
            rng.gen_range(0.42..0.78),
            ExplosionParticleKind::Fire,
        );
    }

    for _ in 0..28 {
        let direction = random_direction(&mut rng);
        let speed = rng.gen_range(3.0..9.0);
        spawn_particle(
            commands,
            assets,
            position,
            impact_velocity * 0.35 + direction * speed + Vec3::Y * rng.gen_range(0.5..3.0),
            rng.gen_range(0.025..0.055),
            rng.gen_range(0.35..0.9),
            0.005,
            ExplosionParticleKind::Spark,
        );
    }

    for _ in 0..18 {
        let direction = random_direction(&mut rng);
        spawn_particle(
            commands,
            assets,
            position + direction * rng.gen_range(0.0..0.25),
            impact_velocity * 0.12
                + Vec3::new(
                    direction.x * 0.65,
                    rng.gen_range(0.8..2.0),
                    direction.z * 0.65,
                ),
            rng.gen_range(0.12..0.24),
            rng.gen_range(1.35..2.2),
            rng.gen_range(0.8..1.5),
            ExplosionParticleKind::Smoke,
        );
    }
}

fn random_direction(rng: &mut StdRng) -> Vec3 {
    loop {
        let direction = Vec3::new(
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-0.75..1.0),
            rng.gen_range(-1.0..1.0),
        );
        if direction.length_squared() > 0.05 {
            return direction.normalize();
        }
    }
}

fn spawn_particle(
    commands: &mut Commands,
    assets: &ExplosionAssets,
    position: Vec3,
    velocity: Vec3,
    start_scale: f32,
    lifetime: f32,
    end_scale: f32,
    kind: ExplosionParticleKind,
) {
    let material = match kind {
        ExplosionParticleKind::Flash => assets.flash.clone(),
        ExplosionParticleKind::Fire => assets.flame.clone(),
        ExplosionParticleKind::Spark => assets.spark.clone(),
        ExplosionParticleKind::Smoke => assets.smoke.clone(),
    };
    commands.spawn((
        Mesh3d(assets.sphere.clone()),
        MeshMaterial3d(material),
        Transform::from_translation(position).with_scale(Vec3::splat(start_scale)),
        Visibility::default(),
        ExplosionParticle {
            velocity,
            age: 0.0,
            lifetime,
            start_scale,
            end_scale,
            kind,
        },
    ));
}

pub fn animate_explosion_particles(
    time: Res<Time>,
    mut commands: Commands,
    mut particles: Query<(Entity, &mut Transform, &mut ExplosionParticle)>,
) {
    let dt = time.delta_secs().clamp(0.0, 0.1);
    for (entity, mut transform, mut particle) in &mut particles {
        particle.age += dt;
        if particle.age >= particle.lifetime {
            commands.entity(entity).despawn();
            continue;
        }
        let progress = particle.age / particle.lifetime;
        match particle.kind {
            ExplosionParticleKind::Flash => {
                let scale = if progress < 0.25 {
                    particle.start_scale + progress / 0.25 * 1.65
                } else {
                    1.8 * (1.0 - (progress - 0.25) / 0.75).powf(0.7)
                };
                transform.scale = Vec3::splat(scale.max(0.01));
                transform.translation += particle.velocity * dt;
            }
            ExplosionParticleKind::Fire => {
                particle.velocity.y -= 3.5 * dt;
                particle.velocity *= (-1.6 * dt).exp();
                transform.translation += particle.velocity * dt;
                let swell = (progress * 5.0).min(1.0);
                let fade = (1.0 - progress).powf(0.65);
                transform.scale =
                    Vec3::splat((particle.start_scale + swell * particle.end_scale) * fade);
            }
            ExplosionParticleKind::Spark => {
                particle.velocity.y -= 9.81 * dt;
                particle.velocity *= (-0.15 * dt).exp();
                transform.translation += particle.velocity * dt;
                transform.rotate_local_x(9.0 * dt);
                transform.rotate_local_z(7.0 * dt);
                transform.scale =
                    Vec3::splat(particle.start_scale * (1.0 - progress).powf(0.5).max(0.05));
            }
            ExplosionParticleKind::Smoke => {
                particle.velocity.y += 0.45 * dt;
                particle.velocity *= (-0.8 * dt).exp();
                transform.translation += particle.velocity * dt;
                let grow = 0.25 + progress * 0.75;
                let fade = if progress < 0.65 {
                    1.0
                } else {
                    (1.0 - (progress - 0.65) / 0.35).max(0.0)
                };
                transform.scale =
                    Vec3::splat((particle.start_scale + particle.end_scale * grow) * fade);
            }
        }
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
    settings: Res<WindSettings>,
    drone: Query<&Transform, With<crate::aircraft::Aircraft>>,
) {
    let position = drone.iter().next().map_or(Vec3::ZERO, |tf| tf.translation);
    wind.0 = wind_velocity(&env, time.elapsed_secs(), position);
    if settings.0 == WindPreset::Tornado {
        let radial = Vec3::new(position.x, 0.0, position.z);
        let radius = radial.length().max(2.0);
        let tangent = Vec3::new(-radial.z, 0.0, radial.x).normalize_or_zero();
        wind.0 += tangent * (18.0 / radius.min(18.0)) + radial.normalize_or_zero() * -4.0;
    }
}

/// Builds the ruined city: damaged buildings (some standing, some
/// collapsed), rubble and craters. Everything with a real collider and
/// the Obstacle component, so impacts explode the drone.
/// Positions avoid the center (0,0) and the z=2 row of test cubes.
fn spawn_ruined_city(
    commands: &mut Commands,
    headless: bool,
    mut meshes: Option<&mut Assets<Mesh>>,
    mut materials: Option<&mut Assets<StandardMaterial>>,
) {
    // (x, z, width, height, depth, ruined)
    let buildings: &[(f32, f32, f32, f32, f32, bool)] = &[
        (-14.0, -10.0, 6.0, 14.0, 6.0, false), // standing (damaged)
        (-6.0, -14.0, 5.0, 8.0, 5.0, false),
        (8.0, -12.0, 7.0, 6.0, 6.0, true), // collapsed
        (14.0, -6.0, 5.0, 10.0, 5.0, false),
        (-12.0, 8.0, 6.0, 4.0, 5.0, true), // collapsed
        (10.0, 10.0, 5.0, 12.0, 6.0, false),
        (-4.0, 12.0, 4.0, 5.0, 4.0, true),
        (4.0, -4.0, 3.0, 3.0, 3.0, true), // low rubble
        (-9.0, -3.0, 3.5, 2.5, 3.0, true),
    ];
    for (x, z, w, h, d, ruined) in buildings {
        let height = if *ruined { h * 0.35 } else { *h };
        let mut e = commands.spawn((
            Obstacle,
            RigidBody::Static,
            Collider::cuboid(w / 2.0, height / 2.0, d / 2.0),
            Transform::from_xyz(*x, height / 2.0, *z),
        ));
        if !headless {
            if let (Some(m), Some(mat)) = (meshes.as_deref_mut(), materials.as_deref_mut()) {
                let color = if *ruined {
                    Color::srgb(0.45, 0.38, 0.33) // burnt concrete
                } else {
                    Color::srgb(0.62, 0.58, 0.52) // damaged facade
                };
                let facade = mat.add(StandardMaterial {
                    base_color: color,
                    perceptual_roughness: 0.95,
                    ..default()
                });
                let trim = mat.add(StandardMaterial {
                    base_color: if *ruined {
                        Color::srgb(0.08, 0.07, 0.06)
                    } else {
                        Color::srgb(0.12, 0.22, 0.25)
                    },
                    emissive: if *ruined {
                        LinearRgba::BLACK
                    } else {
                        LinearRgba::new(0.02, 0.08, 0.1, 1.0)
                    },
                    ..default()
                });
                e.insert((
                    Mesh3d(m.add(Cuboid::new(*w, height, *d))),
                    MeshMaterial3d(facade),
                ));
                // Layered facade details make the blocks read as buildings
                // from the chase camera without adding fragile colliders.
                let strip_width = (*w * 0.08).max(0.12);
                let strip_height = (height * 0.72).max(0.3);
                e.with_children(|building| {
                    building.spawn((
                        Mesh3d(m.add(Cuboid::new(strip_width, strip_height, 0.035))),
                        MeshMaterial3d(trim.clone()),
                        Transform::from_xyz(-*w * 0.22, height * 0.03, *d * 0.5 + 0.02),
                    ));
                    building.spawn((
                        Mesh3d(m.add(Cuboid::new(strip_width, strip_height, 0.035))),
                        MeshMaterial3d(trim.clone()),
                        Transform::from_xyz(*w * 0.22, height * 0.03, *d * 0.5 + 0.02),
                    ));
                    building.spawn((
                        Mesh3d(m.add(Cuboid::new(*w * 1.08, 0.12, *d * 1.08))),
                        MeshMaterial3d(trim.clone()),
                        Transform::from_xyz(0.0, height * 0.5 + 0.06, 0.0),
                    ));
                    if *ruined {
                        building.spawn((
                            Mesh3d(m.add(Cuboid::new(*w * 0.55, 0.18, *d * 0.35))),
                            MeshMaterial3d(trim.clone()),
                            Transform::from_xyz(*w * 0.16, height * 0.5 + 0.17, -*d * 0.1)
                                .with_rotation(Quat::from_rotation_z(0.22)),
                        ));
                    }
                });
            }
        }
    }
    // Small rubble (low, but with colliders: hitting at >2.5 m/s explodes)
    let rubble: &[(f32, f32, f32)] = &[
        (-2.0, -8.0, 0.8),
        (3.0, 6.0, 0.6),
        (6.0, -2.0, 1.0),
        (-8.0, 4.0, 0.7),
        (12.0, 2.0, 0.5),
        (0.0, -16.0, 0.9),
    ];
    for (x, z, s) in rubble {
        let mut e = commands.spawn((
            Obstacle,
            RigidBody::Static,
            Collider::cuboid(s / 2.0, s / 2.0, s / 2.0),
            Transform::from_xyz(*x, s / 2.0, *z),
        ));
        if !headless {
            if let (Some(m), Some(mat)) = (meshes.as_deref_mut(), materials.as_deref_mut()) {
                e.insert((
                    Mesh3d(m.add(Cuboid::new(*s, *s, *s))),
                    MeshMaterial3d(mat.add(StandardMaterial {
                        base_color: Color::srgb(0.4, 0.36, 0.32),
                        perceptual_roughness: 1.0,
                        ..default()
                    })),
                ));
            }
        }
    }
    // Craters: now also with colliders — they are holes in the ground, but
    // their rim (an elevated ring) is a real obstacle: hitting the rim
    // at >2.5 m/s explodes. The center is low so you can land in it.
    let craters: &[(f32, f32, f32)] = &[(-10.0, -6.0, 2.2), (7.0, 4.0, 1.8), (2.0, 14.0, 2.6)];
    for (x, z, r) in craters {
        // Elevated rim (torus approximated with a hollow visual cylinder +
        // ring-shaped collider: we use a low solid cylinder as collider
        // for simplicity; the drone collides against the rim).
        commands.spawn((
            Obstacle,
            RigidBody::Static,
            Collider::cylinder(r * 0.85, 0.25),
            Transform::from_xyz(*x, 0.06, *z),
        ));
        if !headless {
            if let (Some(m), Some(mat)) = (meshes.as_deref_mut(), materials.as_deref_mut()) {
                let dark = mat.add(StandardMaterial {
                    base_color: Color::srgb(0.12, 0.10, 0.09),
                    perceptual_roughness: 1.0,
                    ..default()
                });
                commands.spawn((
                    Mesh3d(m.add(Cylinder::new(*r, 0.12).mesh().resolution(24))),
                    MeshMaterial3d(dark),
                    Transform::from_xyz(*x, 0.06, *z),
                ));
            }
        }
    }
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
            .insert_resource(SimConfig {
                headless: true,
                ..default()
            })
            .add_systems(Update, collision_explosion_system);
        let obstacle = app.world_mut().spawn(Obstacle).id();
        let ground = app.world_mut().spawn_empty().id();
        let drone = app
            .world_mut()
            .spawn((
                crate::aircraft::Aircraft,
                Transform::from_xyz(0.0, 2.0, 0.0),
                CollidingEntities::default(),
                // High speed: the impact against the obstacle must explode.
                avian3d::prelude::LinearVelocity(Vec3::new(10.0, 0.0, 0.0)),
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
    fn slow_impact_does_not_explode_but_fast_does() {
        // The drone needs LinearVelocity to measure impact speed.
        let mut app = App::new();
        app.init_resource::<Time>()
            .init_resource::<ResetRequest>()
            .init_resource::<Explosion>()
            .insert_resource(SimConfig {
                headless: true,
                ..default()
            })
            .add_systems(Update, collision_explosion_system);
        let obstacle = app.world_mut().spawn(Obstacle).id();
        let drone = app
            .world_mut()
            .spawn((
                crate::aircraft::Aircraft,
                Transform::from_xyz(0.0, 2.0, 0.0),
                CollidingEntities::default(),
                avian3d::prelude::LinearVelocity(Vec3::new(1.0, 0.0, 0.0)),
            ))
            .id();
        app.world_mut()
            .get_mut::<CollidingEntities>(drone)
            .unwrap()
            .insert(obstacle);
        // Slow impact (1 m/s < 2.5): no explosion.
        app.update();
        assert!(!app.world().resource::<Explosion>().active);
        assert!(!app.world().resource::<ResetRequest>().0);
        // Fast impact (10 m/s): explodes.
        app.world_mut()
            .get_mut::<avian3d::prelude::LinearVelocity>(drone)
            .unwrap()
            .0 = Vec3::new(10.0, 0.0, 0.0);
        app.update();
        assert!(app.world().resource::<Explosion>().active);
        assert!(app.world().resource::<ResetRequest>().0);
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
