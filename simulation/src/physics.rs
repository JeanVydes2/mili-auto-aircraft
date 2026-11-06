use crate::config::{DroneConfig, EnvironmentConfig};
use crate::motors::Drone;
use crate::sensors::{PrevVelocity, Telemetry};
use avian3d::prelude::*;
use bevy::prelude::*;

pub fn spawn_drone(commands: &mut Commands, cfg: &DroneConfig) {
    spawn_quad(
        commands,
        cfg,
        Vec3::new(0.0, 2.0, 0.0),
        Vec3::new(0.35, 0.06, 0.35),
    );
}

pub fn spawn_quad(
    commands: &mut Commands,
    cfg: &DroneConfig,
    position: Vec3,
    collider_half_extents: Vec3,
) -> Entity {
    let entity = commands
        .spawn((
            Drone,
            crate::motors::Motors::new(),
            crate::motors::CommandInput::default(),
            RigidBody::Dynamic,
            Collider::cuboid(
                collider_half_extents.x * 2.0,
                collider_half_extents.y * 2.0,
                collider_half_extents.z * 2.0,
            ),
            CollidingEntities::default(),
            Mass(cfg.mass),
            AngularInertia::new(cfg.inertia),
            CenterOfMass(cfg.center_of_mass_offset),
            NoAutoMass,
            NoAutoAngularInertia,
            NoAutoCenterOfMass,
            Transform::from_translation(position),
            GlobalTransform::default(),
            Visibility::default(),
        ))
        .insert((
            crate::motors::FlightControlState::default(),
            crate::motors::MotorDynamicsState::default(),
            crate::motor_rpms::MotorRPMs::default(),
            crate::config::BatteryState::default(),
            Telemetry::default(),
            PrevVelocity::default(),
        ))
        .id();
    let (airframe, propulsion) = crate::aircraft::from_quad_config(cfg);
    commands.entity(entity).insert((
        crate::aircraft::Aircraft,
        crate::aircraft::AircraftModel(crate::aircraft::AircraftType::Quad),
        airframe,
        propulsion,
        crate::aircraft::FlightSurfaces::default(),
        crate::aircraft::AircraftControls {
            propulsion: vec![0.0; cfg.motors.len()],
            ..default()
        },
    ));
    entity
}

pub fn setup_gravity(mut gravity: ResMut<Gravity>, env: Res<EnvironmentConfig>) {
    gravity.0 = Vec3::NEG_Y * env.gravity;
}
