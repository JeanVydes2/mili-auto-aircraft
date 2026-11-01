mod battery;
mod camera;
mod config;
mod controller;
mod domain_randomization;
mod environment;
mod motor_rpms;
mod motors;
mod network;
mod physics;
mod sensors;
mod telemetry_log;

use avian3d::prelude::*;
use bevy::prelude::*;
use config::*;

use sensors::{PrevVelocity, Telemetry};

fn main() {
    let headless = std::env::var("HEADLESS").map(|v| v == "1").unwrap_or(false);

    let mut app = App::new();

    app.insert_resource(ClearColor(Color::srgb(0.53, 0.75, 0.98)))
        .insert_resource(SimConfig {
            headless,
            ..default()
        })
        .insert_resource(DroneConfig::default())
        .insert_resource(EnvironmentConfig::default())
        .insert_resource(environment::CurrentWind::default())
        .insert_resource(environment::Explosion::default())
        .insert_resource(BatteryConfig::default())
        .insert_resource(BatteryState::default())
        .insert_resource(SensorConfig::default())
        .insert_resource(NetworkConfig::default())
        .insert_resource(DomainRandomizationConfig::default())
        .insert_resource(CameraConfig::default())
        .insert_resource(Telemetry::default())
        .insert_resource(PrevVelocity(Vec3::ZERO))
        .insert_resource(controller::ActiveController(Box::new(
            controller::AttitudeAltitudeController::default(),
        )))
        .insert_resource(network::ControllerOutput::default())
        .insert_resource(network::ControllerTelemetry::default())
        .insert_resource(network::PendingCommands::default())
        .insert_resource(network::PendingTelemetry::default())
        .insert_resource(domain_randomization::ResetRequest(false))
        .insert_resource(camera::CameraMode::default())
        .insert_resource(camera::FreeCamActive::default())
        .insert_resource(config::GizmosEnabled::default())
        .insert_resource(motor_rpms::MotorRPMs::default());

    if headless {
        app.add_plugins(MinimalPlugins);
        app.add_plugins(bevy::asset::AssetPlugin::default());
        app.add_plugins(bevy::input::InputPlugin::default());
        app.insert_resource(
            bevy::ecs::message::Messages::<bevy::asset::AssetEvent<Mesh>>::default(),
        );
        app.insert_resource(bevy::ecs::message::Messages::<
            bevy::asset::AssetEvent<StandardMaterial>,
        >::default());
    } else {
        app.add_plugins(DefaultPlugins);
    }
    app.add_plugins(PhysicsPlugins::default());

    app.add_systems(Startup, environment::setup_environment);
    app.add_systems(Startup, physics::setup_gravity);
    app.add_systems(Startup, domain_randomization::apply_domain_randomization);
    app.add_systems(Startup, setup_scene);

    app.add_systems(FixedUpdate, motors::command_mixer_system);
    app.add_systems(FixedUpdate, motors::motor_dynamics_system);
    app.add_systems(FixedUpdate, physics::apply_drone_physics_system);
    app.add_systems(FixedUpdate, environment::wind_gust_system);
    app.add_systems(FixedUpdate, environment::collision_explosion_system);
    app.add_systems(FixedUpdate, battery::battery_system);
    app.add_systems(FixedUpdate, sensors::update_telemetry_system);
    app.add_systems(FixedUpdate, domain_randomization::reset_drone_system);

    app.add_systems(Update, controller::run_controller_system);
    app.add_systems(Update, network::send_commands_system);
    app.add_systems(Update, network::deliver_commands_system);
    app.add_systems(Update, network::send_telemetry_system);
    app.add_systems(Update, network::deliver_telemetry_system);

    if !headless {
        app.add_systems(Startup, camera::setup_cameras);
        app.add_systems(Startup, camera::init_cameras);
        app.add_systems(Update, camera::attach_fpv_camera);
        app.add_systems(Update, camera::toggle_freecam_system);
        app.add_systems(Update, camera::cursor_grab_system);
        app.add_systems(Update, camera::freecam_mouse_look_system);
        app.add_systems(Update, camera::freecam_movement_system);
        app.add_systems(Update, telemetry_log::draw_gizmos_system);
        app.add_systems(Update, environment::draw_explosion_system);
    }

    app.insert_resource(Time::<bevy::time::Fixed>::from_hz(240.0));

    app.run();
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: Option<ResMut<Assets<Mesh>>>,
    mut materials: Option<ResMut<Assets<StandardMaterial>>>,
    drone_cfg: Res<DroneConfig>,
    sim: Res<SimConfig>,
) {
    physics::spawn_drone(
        &mut commands,
        &mut meshes,
        &mut materials,
        &drone_cfg,
        sim.headless,
    );
}
