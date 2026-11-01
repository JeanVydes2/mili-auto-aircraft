use crate::config::{DroneConfig, EnvironmentConfig};
use crate::motor_rpms::MotorRPMs;
use crate::motors::{CommandInput, Drone, Motors};
use avian3d::prelude::WriteRigidBodyForces;
use avian3d::prelude::*;
use bevy::prelude::*;

pub fn spawn_drone(
    commands: &mut Commands,
    meshes: &mut Option<ResMut<Assets<Mesh>>>,
    materials: &mut Option<ResMut<Assets<StandardMaterial>>>,
    cfg: &DroneConfig,
    headless: bool,
) {
    let mut entity = commands.spawn((
        Drone,
        Motors::new(),
        CommandInput::default(),
        RigidBody::Dynamic,
        Collider::cuboid(0.3, 0.1, 0.3),
        Mass(cfg.mass),
        AngularInertia::new(cfg.inertia),
        CenterOfMass(cfg.center_of_mass_offset),
        NoAutoMass,
        NoAutoAngularInertia,
        NoAutoCenterOfMass,
        LinearVelocity::default(),
        AngularVelocity::default(),
        Transform::from_xyz(0.0, 2.0, 0.0),
        GlobalTransform::default(),
    ));
    if !headless {
        if let (Some(m), Some(mat)) = (meshes.as_mut(), materials.as_mut()) {
            entity.insert((
                Mesh3d(m.add(Cuboid::new(0.7, 0.12, 0.7))),
                MeshMaterial3d(mat.add(Color::srgb(0.9, 0.15, 0.15))),
            ));
        }
    }
}

pub fn apply_drone_physics_system(
    cfg: Res<DroneConfig>,
    env: Res<EnvironmentConfig>,
    motor_rpms: Res<MotorRPMs>,
    mut query: Query<Forces, With<Drone>>,
) {
    let (thrust, torque) = crate::motors::motor_forces_and_torques(&cfg, &env, &motor_rpms);
    for mut forces in &mut query {
        let pos = forces.position();
        let rot = forces.rotation();
        let linvel = forces.linear_velocity();
        let angvel = forces.angular_velocity();

        // Ground effect.
        let height = pos.y.max(0.01);
        let ge = if height < cfg.ground_effect_height {
            1.0 + cfg.ground_effect_coeff * (1.0 - height / cfg.ground_effect_height)
        } else {
            1.0
        };
        let thrust = thrust * ge;

        let thrust_world = rot * thrust;
        let torque_world = rot * torque;
        forces.apply_force(thrust_world);
        forces.apply_torque(torque_world);

        // Drag in body frame.
        let vel_body = rot.inverse() * linvel;
        let drag_body = Vec3::new(
            -cfg.linear_drag.x * vel_body.x,
            -cfg.linear_drag.y * vel_body.y,
            -cfg.linear_drag.z * vel_body.z,
        );
        let drag_world = rot * drag_body;
        forces.apply_force(drag_world);

        let ang_body = rot.inverse() * angvel;
        let drag_torque_body = Vec3::new(
            -cfg.angular_drag.x * ang_body.x,
            -cfg.angular_drag.y * ang_body.y,
            -cfg.angular_drag.z * ang_body.z,
        );
        forces.apply_torque(rot * drag_torque_body);
    }
}

pub fn setup_gravity(mut commands: Commands, env: Res<EnvironmentConfig>) {
    commands.insert_resource(Gravity(Vec3::new(0.0, -env.gravity, 0.0)));
}
