//! Procedural, render-only scenery. Register `setup_visuals` in Startup after
//! setup_scene/setup_environment, and `animate_visuals` in Update.
//! No colliders, physics transforms, external assets, or renderer plugins required.

use std::f32::consts::{FRAC_PI_2, TAU};

use bevy::light::GlobalAmbientLight;
use bevy::prelude::*;

use crate::aircraft::{Aircraft, AircraftModel, AircraftType, Propulsion};
use crate::config::{DroneConfig, SimConfig};
use crate::environment::CurrentWind;
use crate::motor_rpms::MotorRPMs;
use crate::motors::Drone;

const PROP_RADIUS: f32 = 0.10;

#[derive(Component)]
pub struct VisualScene;

#[derive(Component)]
pub struct DroneVisuals;

#[derive(Component)]
pub struct RotorVisual {
    motor: usize,
    direction: f32,
}

#[derive(Component)]
pub struct WindsockVisual;

struct Palette {
    cube: Handle<Mesh>,
    cylinder: Handle<Mesh>,
    sphere: Handle<Mesh>,
    pad_ring: Handle<Mesh>,
    carbon: Handle<StandardMaterial>,
    metal: Handle<StandardMaterial>,
    rubber: Handle<StandardMaterial>,
    battery: Handle<StandardMaterial>,
    copper: Handle<StandardMaterial>,
    glass: Handle<StandardMaterial>,
    white: Handle<StandardMaterial>,
    yellow: Handle<StandardMaterial>,
    orange: Handle<StandardMaterial>,
    asphalt: Handle<StandardMaterial>,
    grass: [Handle<StandardMaterial>; 2],
    concrete: Handle<StandardMaterial>,
    foliage: Handle<StandardMaterial>,
    red_led: Handle<StandardMaterial>,
    green_led: Handle<StandardMaterial>,
    blue_led: Handle<StandardMaterial>,
}

fn material(
    materials: &mut Assets<StandardMaterial>,
    rgb: [f32; 3],
    metallic: f32,
    roughness: f32,
) -> Handle<StandardMaterial> {
    materials.add(StandardMaterial {
        base_color: Color::srgb(rgb[0], rgb[1], rgb[2]),
        metallic,
        perceptual_roughness: roughness,
        ..default()
    })
}

fn led(materials: &mut Assets<StandardMaterial>, rgb: [f32; 3]) -> Handle<StandardMaterial> {
    materials.add(StandardMaterial {
        base_color: Color::srgb(rgb[0], rgb[1], rgb[2]),
        emissive: LinearRgba::new(rgb[0] * 4.0, rgb[1] * 4.0, rgb[2] * 4.0, 1.0),
        perceptual_roughness: 0.35,
        ..default()
    })
}

impl Palette {
    fn new(meshes: &mut Assets<Mesh>, materials: &mut Assets<StandardMaterial>) -> Self {
        Self {
            cube: meshes.add(Cuboid::new(1.0, 1.0, 1.0)),
            cylinder: meshes.add(Cylinder::new(1.0, 1.0).mesh().resolution(16)),
            sphere: meshes.add(Sphere::new(1.0).mesh().uv(12, 8)),
            pad_ring: meshes.add(Annulus::new(1.12, 1.20).mesh().resolution(64)),
            carbon: material(materials, [0.035, 0.045, 0.055], 0.45, 0.46),
            metal: material(materials, [0.40, 0.46, 0.50], 0.82, 0.3),
            rubber: material(materials, [0.018, 0.022, 0.028], 0.0, 0.94),
            battery: material(materials, [0.12, 0.22, 0.28], 0.25, 0.55),
            copper: material(materials, [0.65, 0.23, 0.06], 0.72, 0.35),
            glass: material(materials, [0.025, 0.12, 0.18], 0.7, 0.12),
            white: material(materials, [0.83, 0.86, 0.79], 0.0, 0.78),
            yellow: material(materials, [0.95, 0.63, 0.07], 0.05, 0.65),
            orange: material(materials, [0.93, 0.22, 0.045], 0.0, 0.82),
            asphalt: material(materials, [0.075, 0.095, 0.11], 0.0, 0.96),
            grass: [
                material(materials, [0.22, 0.29, 0.19], 0.0, 1.0),
                material(materials, [0.26, 0.32, 0.22], 0.0, 1.0),
            ],
            concrete: material(materials, [0.35, 0.39, 0.40], 0.05, 0.9),
            foliage: material(materials, [0.12, 0.23, 0.16], 0.0, 1.0),
            red_led: led(materials, [1.0, 0.035, 0.015]),
            green_led: led(materials, [0.02, 1.0, 0.18]),
            blue_led: led(materials, [0.06, 0.45, 1.0]),
        }
    }
}

/// A small palette-backed spawner: all dimensions are metres in the parent's frame.
struct Painter<'a, 'w, 's> {
    commands: &'a mut Commands<'w, 's>,
    palette: &'a Palette,
    parent: Entity,
}

impl Painter<'_, '_, '_> {
    fn mesh(
        &mut self,
        mesh: &Handle<Mesh>,
        mat: &Handle<StandardMaterial>,
        tf: Transform,
    ) -> Entity {
        self.commands
            .spawn((
                Mesh3d(mesh.clone()),
                MeshMaterial3d(mat.clone()),
                tf,
                Visibility::default(),
                ChildOf(self.parent),
            ))
            .id()
    }

    fn cube(&mut self, center: Vec3, size: Vec3, mat: &Handle<StandardMaterial>) -> Entity {
        self.mesh(
            &self.palette.cube,
            mat,
            Transform::from_translation(center).with_scale(size),
        )
    }

    fn cylinder(
        &mut self,
        center: Vec3,
        radius: f32,
        height: f32,
        mat: &Handle<StandardMaterial>,
    ) -> Entity {
        self.mesh(
            &self.palette.cylinder,
            mat,
            Transform::from_translation(center).with_scale(Vec3::new(radius, height, radius)),
        )
    }

    fn beam(&mut self, a: Vec3, b: Vec3, width: f32, mat: &Handle<StandardMaterial>) {
        self.mesh(&self.palette.cube, mat, beam_transform(a, b, width));
    }
}

fn beam_transform(a: Vec3, b: Vec3, width: f32) -> Transform {
    let delta = b - a;
    Transform::from_translation((a + b) * 0.5)
        .with_rotation(Quat::from_rotation_arc(
            Vec3::Y,
            delta.try_normalize().unwrap_or(Vec3::Y),
        ))
        .with_scale(Vec3::new(width, delta.length(), width))
}

/// Safe to register in a headless app: absent mesh/material assets or headless mode
/// produce no entities. Repeated calls do not duplicate the scene or drone model.
pub fn setup_visuals(
    mut commands: Commands,
    meshes: Option<ResMut<Assets<Mesh>>>,
    materials: Option<ResMut<Assets<StandardMaterial>>>,
    cfg: Res<DroneConfig>,
    sim: Option<Res<SimConfig>>,
    drones: Query<Entity, (With<Drone>, Without<DroneVisuals>)>,
    other_aircraft: Query<
        (Entity, &AircraftModel, &Propulsion),
        (With<Aircraft>, Without<Drone>, Without<DroneVisuals>),
    >,
    scene: Query<(), With<VisualScene>>,
) {
    if sim.as_ref().is_some_and(|sim| sim.headless) {
        return;
    }
    let (Some(mut meshes), Some(mut materials)) = (meshes, materials) else {
        return;
    };
    if !scene.is_empty() && drones.is_empty() && other_aircraft.is_empty() {
        return;
    }
    let palette = Palette::new(&mut meshes, &mut materials);
    for drone in &drones {
        commands.entity(drone).insert(DroneVisuals);
        build_drone(
            &mut Painter {
                commands: &mut commands,
                palette: &palette,
                parent: drone,
            },
            &cfg,
        );
    }
    for (entity, model, propulsion) in &other_aircraft {
        commands.entity(entity).insert(DroneVisuals);
        build_generic_aircraft(
            &mut Painter {
                commands: &mut commands,
                palette: &palette,
                parent: entity,
            },
            model.0,
            propulsion,
        );
    }
    if scene.is_empty() {
        // Bevy 0.20 uses a resource distinct from the camera AmbientLight component.
        // Preserve an application-provided ambient setting if one already exists.
        commands.queue(|world: &mut World| {
            if !world.contains_resource::<GlobalAmbientLight>() {
                world.insert_resource(GlobalAmbientLight {
                    color: Color::srgb(0.76, 0.85, 1.0),
                    brightness: 100.0,
                    ..default()
                });
            }
        });
        let root = commands
            .spawn((
                Name::new("Procedural test range"),
                VisualScene,
                Transform::IDENTITY,
                Visibility::default(),
            ))
            .id();
        build_range(&mut Painter {
            commands: &mut commands,
            palette: &palette,
            parent: root,
        });
    }
}

fn build_generic_aircraft(p: &mut Painter, model: AircraftType, propulsion: &Propulsion) {
    let m = p.palette;
    p.cube(
        Vec3::new(0.0, 0.015, 0.0),
        Vec3::new(0.12, 0.08, 0.74),
        &m.carbon,
    );
    match model {
        AircraftType::Hex | AircraftType::Quad => {
            for propulsor in &propulsion.0 {
                p.beam(Vec3::ZERO, propulsor.position, 0.045, &m.carbon);
                p.cylinder(propulsor.position, 0.052, 0.045, &m.metal);
                p.cube(
                    propulsor.position + Vec3::Y * 0.027,
                    Vec3::new(0.22, 0.008, 0.025),
                    &m.carbon,
                );
            }
        }
        AircraftType::FixedWing | AircraftType::Vtol => {
            p.cube(
                Vec3::new(0.0, 0.0, 0.02),
                Vec3::new(
                    if model == AircraftType::Vtol {
                        1.5
                    } else {
                        1.8
                    },
                    0.035,
                    0.28,
                ),
                &m.white,
            );
            p.cube(
                Vec3::new(0.0, 0.045, -0.5),
                Vec3::new(0.62, 0.025, 0.16),
                &m.white,
            );
            p.cube(
                Vec3::new(0.0, 0.17, -0.44),
                Vec3::new(0.035, 0.28, 0.14),
                &m.orange,
            );
            for propulsor in &propulsion.0 {
                if propulsor.axis.dot(Vec3::Y).abs() > 0.8 {
                    p.beam(Vec3::ZERO, propulsor.position, 0.025, &m.carbon);
                    p.cylinder(propulsor.position, 0.042, 0.035, &m.metal);
                    p.cube(
                        propulsor.position + Vec3::Y * 0.023,
                        Vec3::new(0.18, 0.006, 0.018),
                        &m.carbon,
                    );
                } else {
                    p.cylinder(propulsor.position, 0.075, 0.025, &m.carbon);
                }
            }
        }
    }
}

fn build_drone(p: &mut Painter, cfg: &DroneConfig) {
    let m = p.palette;
    // The narrow central stack exposes the X-frame; the whole craft stays within
    // the default 0.7 x 0.12 x 0.7 collider, including feet and rotor tips.
    for y in [-0.018, 0.014] {
        p.cube(
            Vec3::new(0.0, y, 0.0),
            Vec3::new(0.19, 0.009, 0.24),
            &m.carbon,
        );
    }
    for x in [-0.073, 0.073] {
        for z in [-0.085, 0.085] {
            p.cylinder(Vec3::new(x, 0.0, z), 0.006, 0.035, &m.metal);
        }
    }
    p.cube(
        Vec3::new(0.0, 0.036, -0.012),
        Vec3::new(0.105, 0.036, 0.135),
        &m.battery,
    );
    for z in [-0.05, 0.028] {
        p.cube(
            Vec3::new(0.0, 0.055, z),
            Vec3::new(0.115, 0.006, 0.018),
            &m.rubber,
        );
    }
    p.beam(
        Vec3::new(0.046, 0.041, -0.075),
        Vec3::new(0.08, 0.025, -0.1),
        0.006,
        &m.orange,
    );
    p.beam(
        Vec3::new(0.08, 0.025, -0.1),
        Vec3::new(0.058, 0.005, -0.07),
        0.006,
        &m.rubber,
    );
    // Camera barrel faces body +Z, independently of Bevy camera's -Z convention.
    p.cube(
        Vec3::new(0.0, 0.0, 0.126),
        Vec3::new(0.063, 0.043, 0.034),
        &m.metal,
    );
    for (z, radius, height, mat) in [
        (0.149, 0.022, 0.025, &m.rubber),
        (0.163, 0.017, 0.003, &m.glass),
    ] {
        let lens = p.cylinder(Vec3::new(0.0, 0.0, z), radius, height, mat);
        p.commands.entity(lens).insert(
            Transform::from_xyz(0.0, 0.0, z)
                .with_rotation(Quat::from_rotation_x(FRAC_PI_2))
                .with_scale(Vec3::new(radius, height, radius)),
        );
    }
    for (i, motor) in cfg.motors.iter().enumerate() {
        let pos = motor.position;
        p.beam(
            Vec3::new(0.0, -0.006, 0.0),
            pos + Vec3::new(0.0, -0.006, 0.0),
            0.025,
            &m.carbon,
        );
        p.cylinder(pos + Vec3::Y * 0.012, 0.025, 0.032, &m.metal);
        p.cylinder(pos + Vec3::Y * 0.014, 0.026, 0.014, &m.copper);
        p.cylinder(pos + Vec3::Y * 0.031, 0.025, 0.008, &m.carbon);
        let rotor = p
            .commands
            .spawn((
                Name::new(format!("Rotor {i}")),
                RotorVisual {
                    motor: i,
                    direction: motor.rotation_dir.signum(),
                },
                Transform::from_translation(pos + Vec3::Y * 0.044),
                Visibility::default(),
                ChildOf(p.parent),
            ))
            .id();
        let parent = p.parent;
        p.parent = rotor;
        // A full 0.20 m two-blade propeller, with a second narrow chord at its hub.
        p.cube(
            Vec3::ZERO,
            Vec3::new(PROP_RADIUS * 2.0, 0.003, 0.017),
            &m.carbon,
        );
        p.cylinder(Vec3::Y * 0.003, 0.009, 0.009, &m.metal);
        p.parent = parent;
        let nav = if pos.x < 0.0 {
            &m.red_led
        } else {
            &m.green_led
        };
        p.cube(
            pos + Vec3::new(0.0, -0.014, 0.027 * pos.z.signum()),
            Vec3::new(0.022, 0.009, 0.009),
            nav,
        );
        let foot = pos * 0.68;
        p.beam(
            foot + Vec3::Y * -0.018,
            foot + Vec3::Y * -0.048,
            0.012,
            &m.metal,
        );
        p.cube(
            foot + Vec3::Y * -0.054,
            Vec3::new(0.026, 0.012, 0.045),
            &m.rubber,
        );
    }
    p.cube(
        Vec3::new(0.0, 0.008, -0.125),
        Vec3::new(0.027, 0.01, 0.008),
        &m.blue_led,
    );
}

fn build_range(p: &mut Painter) {
    let m = p.palette;
    // Thin overlays sit above the existing y=0 ground, not above its collision plane.
    for x in 0..6 {
        for z in 0..6 {
            p.cube(
                Vec3::new((x as f32 - 2.5) * 4.0, 0.002, (z as f32 - 2.5) * 4.0),
                Vec3::new(4.0, 0.003, 4.0),
                &m.grass[(x + z) % 2],
            );
        }
    }
    for i in -3..=3 {
        let d = i as f32 * 4.0;
        p.cube(
            Vec3::new(d, 0.005, 0.0),
            Vec3::new(0.018, 0.002, 24.0),
            &m.concrete,
        );
        p.cube(
            Vec3::new(0.0, 0.005, d),
            Vec3::new(24.0, 0.002, 0.018),
            &m.concrete,
        );
    }
    p.cylinder(Vec3::new(0.0, 0.008, 0.0), 1.42, 0.008, &m.asphalt);
    p.mesh(
        &m.pad_ring,
        &m.yellow,
        Transform::from_xyz(0.0, 0.014, 0.0).with_rotation(Quat::from_rotation_x(-FRAC_PI_2)),
    );
    for x in [-0.32, 0.32] {
        p.cube(
            Vec3::new(x, 0.015, 0.0),
            Vec3::new(0.11, 0.002, 0.85),
            &m.white,
        );
    }
    p.cube(
        Vec3::new(0.0, 0.015, 0.0),
        Vec3::new(0.64, 0.002, 0.11),
        &m.white,
    );
    for i in 0..4 {
        let angle = i as f32 * FRAC_PI_2;
        let pos = Vec3::new(angle.cos() * 1.32, 0.017, angle.sin() * 1.32);
        p.cube(pos, Vec3::new(0.09, 0.005, 0.09), &m.yellow);
    }
    // A taxi/runway strip offset from the obstacle row and landing pad.
    p.cube(
        Vec3::new(0.0, 0.009, -7.0),
        Vec3::new(20.0, 0.012, 2.6),
        &m.asphalt,
    );
    for z in [-8.18, -5.82] {
        p.cube(
            Vec3::new(0.0, 0.017, z),
            Vec3::new(19.5, 0.002, 0.05),
            &m.white,
        );
    }
    for i in -4..=4 {
        p.cube(
            Vec3::new(i as f32 * 2.0, 0.017, -7.0),
            Vec3::new(0.85, 0.002, 0.07),
            &m.white,
        );
    }
    for x in [-9.0, 9.0] {
        for z in [-7.75, -7.25, -6.75, -6.25] {
            p.cube(
                Vec3::new(x, 0.017, z),
                Vec3::new(0.55, 0.002, 0.18),
                &m.white,
            );
        }
    }
    // Surface decoration only: the five existing 1 m cubes remain authoritative.
    for x in [-8.0, -4.0, 0.0, 4.0, 8.0] {
        for side in [-1.0, 1.0] {
            let z = 2.0 + side * 0.505;
            p.beam(
                Vec3::new(x - 0.43, 0.08, z),
                Vec3::new(x + 0.43, 0.92, z),
                0.032,
                &m.metal,
            );
            p.beam(
                Vec3::new(x + 0.43, 0.08, z),
                Vec3::new(x - 0.43, 0.92, z),
                0.032,
                &m.metal,
            );
            p.cube(
                Vec3::new(x, 0.76, z + side * 0.006),
                Vec3::new(0.92, 0.10, 0.012),
                &m.yellow,
            );
        }
    }
    // Numeric distance boards are mesh seven-segment glyphs, not font assets.
    for (distance, digits) in [
        (5.0, [None, Some(5)]),
        (10.0, [Some(1), Some(0)]),
        (15.0, [Some(1), Some(5)]),
    ] {
        let center = Vec3::new(0.0, 0.55, distance);
        p.cylinder(
            center * Vec3::new(1.0, 0.0, 1.0) + Vec3::Y * 0.22,
            0.022,
            0.44,
            &m.metal,
        );
        p.cube(center, Vec3::new(0.75, 0.43, 0.045), &m.asphalt);
        for (i, digit) in digits.into_iter().enumerate() {
            if let Some(digit) = digit {
                digit_mesh(
                    p,
                    center + Vec3::new((i as f32 - 0.5) * 0.28, 0.0, -0.026),
                    digit,
                );
            }
        }
    }
    // Low-rise silhouettes and stylized trees outside the 24 m test field.
    for i in 0..6 {
        let x = -22.0 + i as f32 * 8.0;
        let height = 2.5 + (i % 3) as f32 * 1.4;
        p.cube(
            Vec3::new(x, height * 0.5, 29.0),
            Vec3::new(5.0, height, 4.0),
            &m.concrete,
        );
        p.cube(
            Vec3::new(x, height + 0.08, 29.0),
            Vec3::new(5.15, 0.16, 4.15),
            &m.metal,
        );
        for z in [-21.0, 20.0] {
            p.cylinder(Vec3::new(x, 0.8, z), 0.14, 1.6, &m.rubber);
            p.mesh(
                &m.sphere,
                &m.foliage,
                Transform::from_xyz(x, 2.2, z).with_scale(Vec3::new(1.1, 1.6, 1.1)),
            );
        }
    }
    for x in [-11.0, 11.0] {
        for z in [-10.0, 0.0, 10.0] {
            p.cylinder(Vec3::new(x, 0.18, z), 0.065, 0.36, &m.metal);
            p.cylinder(Vec3::new(x, 0.38, z), 0.075, 0.08, &m.blue_led);
        }
    }
    build_station(p);
}

fn digit_mesh(p: &mut Painter, center: Vec3, digit: usize) {
    // Segment order: top, upper right, lower right, bottom, lower left, upper left, middle.
    const MASKS: [u8; 10] = [0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f];
    let segments = [
        (0.0, 0.14, true),
        (0.085, 0.07, false),
        (0.085, -0.07, false),
        (0.0, -0.14, true),
        (-0.085, -0.07, false),
        (-0.085, 0.07, false),
        (0.0, 0.0, true),
    ];
    for (i, (x, y, horizontal)) in segments.into_iter().enumerate() {
        if MASKS[digit] & (1 << i) != 0 {
            let size = if horizontal {
                Vec3::new(0.16, 0.025, 0.004)
            } else {
                Vec3::new(0.025, 0.12, 0.004)
            };
            p.cube(center + Vec3::new(x, y, 0.0), size, &p.palette.yellow);
        }
    }
}

fn build_station(p: &mut Painter) {
    let m = p.palette;
    let base = Vec3::new(-3.0, 0.0, -3.0);
    for x in [-0.38, 0.38] {
        for z in [-0.22, 0.22] {
            p.beam(
                base + Vec3::new(x, 0.0, z),
                base + Vec3::new(x, 0.66, z),
                0.035,
                &m.metal,
            );
        }
    }
    p.cube(
        base + Vec3::Y * 0.68,
        Vec3::new(1.0, 0.055, 0.65),
        &m.concrete,
    );
    p.cube(
        base + Vec3::new(-0.12, 0.75, 0.02),
        Vec3::new(0.48, 0.08, 0.31),
        &m.rubber,
    );
    p.cube(
        base + Vec3::new(-0.12, 0.94, 0.15),
        Vec3::new(0.46, 0.32, 0.035),
        &m.metal,
    );
    p.cube(
        base + Vec3::new(-0.12, 0.94, 0.128),
        Vec3::new(0.40, 0.26, 0.008),
        &m.glass,
    );
    p.cube(
        base + Vec3::new(0.32, 0.79, 0.02),
        Vec3::new(0.15, 0.16, 0.2),
        &m.battery,
    );
    p.beam(
        base + Vec3::new(0.32, 0.86, 0.02),
        base + Vec3::new(0.32, 1.3, 0.02),
        0.012,
        &m.metal,
    );
    p.cube(
        base + Vec3::new(0.32, 0.83, -0.085),
        Vec3::new(0.04, 0.02, 0.008),
        &m.green_led,
    );
    let mast = base + Vec3::new(-0.9, 0.0, 0.3);
    p.cylinder(mast + Vec3::Y * 1.05, 0.027, 2.1, &m.metal);
    p.cylinder(mast + Vec3::Y * 0.045, 0.24, 0.08, &m.concrete);
    let sock = p
        .commands
        .spawn((
            Name::new("Wind velocity indicator"),
            WindsockVisual,
            Transform::from_translation(mast + Vec3::Y * 2.0),
            Visibility::default(),
            ChildOf(p.parent),
        ))
        .id();
    let parent = p.parent;
    p.parent = sock;
    // Local +Y is downstream. Tapered bands retain one shared cylinder mesh.
    for i in 0..5 {
        p.cylinder(
            Vec3::Y * (0.08 + i as f32 * 0.145),
            0.13 - i as f32 * 0.019,
            0.15,
            if i % 2 == 0 { &m.orange } else { &m.white },
        );
    }
    p.parent = parent;
}

fn rotor_step(rpm: f32, direction: f32, dt: f32) -> f32 {
    if !rpm.is_finite() || !dt.is_finite() || dt <= 0.0 {
        return 0.0;
    }
    // Reduce the phase before quaternion construction, not the actual angular speed.
    ((rpm.max(0.0) / 60.0) * TAU * dt * direction).rem_euclid(TAU)
}

fn sock_rotation(wind: Vec3, elapsed: f32) -> Quat {
    let horizontal = Vec3::new(wind.x, 0.0, wind.z);
    let speed = horizontal.length();
    if !speed.is_finite() || speed < 0.02 {
        return Quat::from_rotation_arc(Vec3::Y, Vec3::NEG_Y);
    }
    let extension = (speed / 5.0).clamp(0.0, 1.0);
    let flutter = (elapsed * 8.0).sin() * 0.04 * extension;
    let downstream = horizontal / speed;
    let direction =
        (downstream * extension + Vec3::NEG_Y * (1.0 - extension + flutter)).normalize();
    Quat::from_rotation_arc(Vec3::Y, direction)
}

/// Reads actual motor RPM (not target RPM). Only the rotor and windsock local
/// rotations change; the Drone's physics-controlled Transform is never queried.
/// MotorRPMs lives on the parent drone, so rotors are matched through the
/// hierarchy (ChildOf -> Ancestors).
pub fn animate_visuals(
    time: Res<Time>,
    wind: Option<Res<CurrentWind>>,
    mut rotors: Query<(&RotorVisual, &ChildOf, &mut Transform), Without<WindsockVisual>>,
    rpms: Query<&MotorRPMs>,
    mut socks: Query<&mut Transform, (With<WindsockVisual>, Without<RotorVisual>)>,
) {
    for (rotor, parent, mut tf) in &mut rotors {
        let rpm = rpms
            .get(parent.parent())
            .map(|r| r.rpms[rotor.motor])
            .unwrap_or(0.0);
        tf.rotate_y(rotor_step(rpm, rotor.direction, time.delta_secs()));
    }
    let rotation = sock_rotation(wind.map_or(Vec3::ZERO, |wind| wind.0), time.elapsed_secs());
    for mut tf in &mut socks {
        tf.rotation = rotation;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn app_with_assets() -> App {
        let mut app = App::new();
        app.init_resource::<Assets<Mesh>>()
            .init_resource::<Assets<StandardMaterial>>()
            .init_resource::<DroneConfig>()
            .init_resource::<Time>()
            .add_systems(Startup, setup_visuals)
            .add_systems(Update, animate_visuals);
        app
    }

    #[test]
    fn setup_without_renderer_preserves_physics_pose_and_reuses_assets() {
        let mut app = app_with_assets();
        let pose = Transform::from_xyz(4.0, 2.0, -3.0).with_rotation(Quat::from_rotation_z(0.2));
        let drone = app
            .world_mut()
            .spawn((Drone, pose, Visibility::default()))
            .id();
        app.update();
        assert_eq!(*app.world().get::<Transform>(drone).unwrap(), pose);
        assert!(app.world().get::<DroneVisuals>(drone).is_some());
        assert!(app.world().entities().len() <= 300);
        assert_eq!(app.world().resource::<Assets<Mesh>>().len(), 4);
        assert_eq!(app.world().resource::<Assets<StandardMaterial>>().len(), 17);
        let world = app.world_mut();
        let mut query = world.query::<(&RotorVisual, &Transform, &ChildOf)>();
        let cfg = DroneConfig::default();
        assert_eq!(query.iter(world).count(), 4);
        for (rotor, tf, parent) in query.iter(world) {
            assert_eq!(parent.parent(), drone);
            assert_eq!(
                tf.translation,
                cfg.motors[rotor.motor].position + Vec3::Y * 0.044
            );
        }
        // Re-registering setup must not duplicate entities or assets.
        let count = world.entities().len();
        app.add_systems(Update, setup_visuals);
        app.update();
        assert_eq!(app.world().entities().len(), count);
        assert_eq!(app.world().resource::<Assets<Mesh>>().len(), 4);
    }

    #[test]
    fn every_visible_child_has_a_visible_parent_without_renderer() {
        let mut app = app_with_assets();
        // The physics-owned root supplies visibility; visuals only own its children.
        app.world_mut()
            .spawn((Drone, Transform::IDENTITY, Visibility::default()));
        app.update();

        let world = app.world_mut();
        let mut children = world.query_filtered::<(Entity, &ChildOf), With<InheritedVisibility>>();
        for (child, parent) in children.iter(world) {
            assert!(
                world.get::<InheritedVisibility>(parent.parent()).is_some(),
                "visible child {child:?} has parent {:?} without InheritedVisibility",
                parent.parent(),
            );
        }
        let mut meshes = world.query_filtered::<Entity, With<Mesh3d>>();
        assert!(meshes.iter(world).count() > 0);
        for mesh in meshes.iter(world) {
            assert!(world.get::<InheritedVisibility>(mesh).is_some());
        }
        let visual_count = world
            .query_filtered::<Entity, Or<(
                With<Mesh3d>,
                With<VisualScene>,
                With<RotorVisual>,
                With<WindsockVisual>,
            )>>()
            .iter(world)
            .count();
        assert!(
            visual_count <= 300,
            "visual entity budget exceeded: {visual_count}"
        );
        println!(
            "Visual entities: {visual_count} (budget: 300; excludes Drone root and app internals)"
        );
    }

    #[test]
    fn headless_and_missing_render_assets_are_noops() {
        for headless in [false, true] {
            let mut app = App::new();
            app.init_resource::<DroneConfig>()
                .insert_resource(SimConfig {
                    headless,
                    ..default()
                })
                .add_systems(Startup, setup_visuals);
            if headless {
                app.init_resource::<Assets<Mesh>>()
                    .init_resource::<Assets<StandardMaterial>>();
            }
            app.update();
            let world = app.world_mut();
            assert_eq!(world.query::<&Mesh3d>().iter(world).count(), 0);
            assert_eq!(world.query::<&VisualScene>().iter(world).count(), 0);
        }
    }

    #[test]
    fn actual_rpms_rotate_only_rotors_and_wind_points_downstream() {
        let mut app = app_with_assets();
        let drone = app
            .world_mut()
            .spawn((
                Drone,
                Transform::IDENTITY,
                Visibility::default(),
                MotorRPMs {
                    rpms: [15.0, 0.0, 0.0, 0.0],
                    target_rpms: [9000.0; 4],
                },
            ))
            .id();
        app.insert_resource(CurrentWind(Vec3::X * 5.0));
        app.world_mut()
            .resource_mut::<Time>()
            .advance_by(Duration::from_secs_f32(0.25));
        app.update();
        let world = app.world_mut();
        let mut query = world.query::<(&RotorVisual, &Transform)>();
        for (rotor, tf) in query.iter(world) {
            let angle = if rotor.motor == 0 { TAU / 16.0 } else { 0.0 };
            let expected = Quat::from_rotation_y(angle) * Vec3::X;
            assert!((tf.rotation * Vec3::X - expected).length() < 1e-5);
        }
        assert_eq!(*world.get::<Transform>(drone).unwrap(), Transform::IDENTITY);
        assert!((sock_rotation(Vec3::X * 5.0, 0.0) * Vec3::Y - Vec3::X).length() < 1e-5);
        assert!((sock_rotation(Vec3::ZERO, 0.0) * Vec3::Y - Vec3::NEG_Y).length() < 1e-5);
    }

    #[test]
    fn geometry_and_rotor_phase_are_well_behaved() {
        let a = Vec3::new(-1.0, 2.0, 3.0);
        let b = Vec3::new(2.0, -1.0, 0.0);
        let tf = beam_transform(a, b, 0.02);
        assert!((tf.transform_point(Vec3::Y * -0.5) - a).length() < 1e-5);
        assert!((tf.transform_point(Vec3::Y * 0.5) - b).length() < 1e-5);
        assert!(beam_transform(a, a, 0.02).rotation.is_finite());
        assert!((rotor_step(15.0, 1.0, 1.0) - FRAC_PI_2).abs() < 1e-5);
        assert!((rotor_step(15.0, -1.0, 1.0) - 3.0 * FRAC_PI_2).abs() < 1e-5);
        assert_eq!(rotor_step(f32::NAN, 1.0, 1.0), 0.0);
        for motor in DroneConfig::default().motors {
            assert!(motor.position.x.abs() + PROP_RADIUS < 0.35);
            assert!(motor.position.z.abs() + PROP_RADIUS < 0.35);
        }
    }
}
