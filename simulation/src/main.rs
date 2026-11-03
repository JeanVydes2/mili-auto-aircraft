mod aircraft;
mod battery;
mod camera;
mod config;
mod domain_randomization;
mod environment;
mod hud;
mod mavlink;
mod motor_rpms;
mod motors;
mod physics;
mod sensors;
mod stream;
mod telemetry_log;
mod visuals;

use avian3d::prelude::*;
use bevy::prelude::*;
use config::*;

use sensors::{PrevVelocity, Telemetry};

fn main() {
    let headless = std::env::var("HEADLESS").map(|v| v == "1").unwrap_or(false);

    build_app(headless).run();
}

fn build_app(headless: bool) -> App {
    let mut app = App::new();

    app.insert_resource(ClearColor(Color::srgb(0.53, 0.75, 0.98)))
        .insert_resource(aircraft::AircraftCatalog::load())
        .insert_resource(SimConfig {
            headless,
            ..default()
        })
        .insert_resource(FlightControlLimits::default())
        .insert_resource(DroneConfig::default())
        .insert_resource(EnvironmentConfig::default())
        .insert_resource(environment::CurrentWind::default())
        .insert_resource(environment::Explosion::default())
        .insert_resource(BatteryConfig::default())
        .insert_resource(BatteryState::default())
        .insert_resource(SensorConfig::default())
        .insert_resource(DomainRandomizationConfig::default())
        .insert_resource(CameraConfig::default())
        .insert_resource(Telemetry::default())
        .insert_resource(PrevVelocity(Vec3::ZERO))
        .insert_resource(mavlink::MavlinkConfig::default())
        .insert_resource(mavlink::MavlinkInterface::default())
        .insert_resource(domain_randomization::ResetRequest(false))
        .init_resource::<domain_randomization::ResetGeneration>()
        .insert_resource(camera::CameraMode::default())
        .insert_resource(camera::FreeCamActive::default())
        .insert_resource(config::GizmosEnabled::default())
        .insert_resource(motor_rpms::MotorRPMs::default())
        .init_resource::<stream::VideoLinkConfig>()
        .init_resource::<stream::VideoReceiver>()
        .init_resource::<stream::StreamStats>()
        .init_resource::<stream::VideoOutput>()
        .init_resource::<stream::ReceiverView>();

    if headless {
        app.add_plugins((MinimalPlugins, TransformPlugin));
        app.add_plugins(bevy::asset::AssetPlugin::default());
        app.add_plugins(bevy::input::InputPlugin::default());
        app.init_asset::<Mesh>();
        app.init_asset::<StandardMaterial>();
    } else {
        app.add_plugins(
            DefaultPlugins
                .set(bevy::winit::WinitPlugin {
                    run_on_any_thread: cfg!(test),
                    ..default()
                })
                .set(bevy::window::WindowPlugin {
                    primary_window: Some(Window {
                        title: "VYDE | Ground Control".into(),
                        resolution: (1440, 900).into(),
                        ..default()
                    }),
                    ..default()
                }),
        );
    }
    app.add_plugins(PhysicsPlugins::new(FixedUpdate));
    if !headless {
        app.add_observer(select_profile_button);
    }

    app.add_systems(
        Startup,
        (
            domain_randomization::apply_domain_randomization,
            physics::setup_gravity,
            setup_scene,
            environment::setup_environment,
        )
            .chain(),
    );
    app.add_systems(Startup, mavlink::setup_mavlink);

    // Reset poses before Avian computes mass/transform data. Run the entire
    // control/actuator/force pipeline at a deterministic 240 Hz, not render FPS.
    app.add_systems(
        FixedUpdate,
        domain_randomization::reset_aircraft_system.before(PhysicsSystems::Prepare),
    );
    app.add_systems(
        FixedUpdate,
        aircraft::apply_aircraft_forces_system
            .after(PhysicsSystems::Prepare)
            .after(motors::motor_dynamics_system)
            .after(environment::wind_gust_system)
            .before(PhysicsSystems::StepSimulation),
    );
    app.add_systems(
        FixedUpdate,
        (
            mavlink::apply_guided_control_system,
            motors::command_mixer_system,
            motors::motor_dynamics_system,
            environment::wind_gust_system,
            battery::battery_system,
        )
            .chain()
            .after(PhysicsSystems::Prepare)
            .before(PhysicsSystems::StepSimulation),
    );
    app.add_systems(
        FixedUpdate,
        (
            environment::collision_explosion_system,
            sensors::update_telemetry_system,
        )
            .chain()
            .after(PhysicsSystems::Writeback),
    );
    app.add_systems(
        Update,
        (
            request_reset_system,
            select_aircraft_system,
            mavlink::mavlink_io_system,
        ),
    );
    if !headless {
        app.add_systems(Update, visuals::setup_visuals.after(select_aircraft_system));
    }

    if !headless {
        app.add_systems(
            Startup,
            (
                visuals::setup_visuals,
                camera::setup_cameras,
                camera::attach_fpv_camera,
                stream::setup_stream,
                hud::setup_hud,
            )
                .chain()
                .after(setup_scene)
                .after(environment::setup_environment),
        );
        app.add_observer(stream::receive_gpu_frame);
        app.add_systems(
            Update,
            (
                camera::toggle_freecam_system,
                camera::attach_fpv_camera,
                camera::update_view_cameras,
                camera::cursor_grab_system,
                camera::freecam_mouse_look_system,
                camera::freecam_movement_system,
                camera::chase_camera_system,
                stream::video_controls,
                stream::capture_video,
                stream::present_video,
                hud::update_hud,
                visuals::animate_visuals,
            )
                .chain(),
        );
        app.add_systems(
            Update,
            telemetry_log::draw_gizmos_system.after(hud::update_hud),
        );
        app.add_systems(Update, environment::draw_explosion_system);
    }

    app.insert_resource(Time::<bevy::time::Fixed>::from_hz(240.0));

    app
}

fn request_reset_system(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut reset: ResMut<domain_randomization::ResetRequest>,
) {
    if keyboard.just_pressed(KeyCode::KeyR) {
        reset.0 = true;
    }
}

fn setup_scene(
    mut commands: Commands,
    mut drone_cfg: ResMut<DroneConfig>,
    catalog: Res<aircraft::AircraftCatalog>,
) {
    spawn_profile(&mut commands, &mut drone_cfg, catalog.selected());
}

fn spawn_profile(
    commands: &mut Commands,
    drone_cfg: &mut DroneConfig,
    profile: &aircraft::AircraftDefinition,
) {
    if profile.kind == aircraft::AircraftType::Quad {
        drone_cfg.mass = profile.body.mass;
        drone_cfg.inertia = profile.body.inertia;
        drone_cfg.center_of_mass_offset = profile.body.center_of_mass;
        drone_cfg.linear_drag = profile.body.linear_drag;
        drone_cfg.induced_drag = profile.body.quadratic_drag.max_element();
        drone_cfg.angular_drag = profile.body.angular_drag;
        physics::spawn_quad(
            commands,
            drone_cfg,
            Vec3::new(0.0, 2.0, 0.0),
            profile.collider_half_extents,
        );
    } else {
        profile.spawn(commands, Vec3::new(0.0, 2.0, 0.0));
    }
}

fn select_aircraft_system(
    mut commands: Commands,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut catalog: ResMut<aircraft::AircraftCatalog>,
    mut drone_cfg: ResMut<DroneConfig>,
    active: Query<Entity, With<aircraft::Aircraft>>,
) {
    let keys = [
        KeyCode::Digit1,
        KeyCode::Digit2,
        KeyCode::Digit3,
        KeyCode::Digit4,
    ];
    let Some(index) = keys.iter().position(|key| keyboard.just_pressed(*key)) else {
        return;
    };
    select_profile(&mut commands, &mut catalog, &mut drone_cfg, &active, index);
}

fn select_profile_button(
    activate: On<bevy_ui_widgets::Activate>,
    buttons: Query<&hud::AircraftSelectButton>,
    mut commands: Commands,
    mut catalog: ResMut<aircraft::AircraftCatalog>,
    mut drone_cfg: ResMut<DroneConfig>,
    active: Query<Entity, With<aircraft::Aircraft>>,
) {
    let Ok(button) = buttons.get(activate.entity) else {
        return;
    };
    select_profile(
        &mut commands,
        &mut catalog,
        &mut drone_cfg,
        &active,
        button.0,
    );
}

fn select_profile(
    commands: &mut Commands,
    catalog: &mut aircraft::AircraftCatalog,
    drone_cfg: &mut DroneConfig,
    active: &Query<Entity, With<aircraft::Aircraft>>,
    index: usize,
) {
    if index >= catalog.definitions.len() || index == catalog.selected {
        return;
    }
    catalog.selected = index;
    for entity in active.iter() {
        commands.entity(entity).despawn();
    }
    spawn_profile(commands, drone_cfg, &catalog.definitions[catalog.selected]);
}

#[cfg(test)]
mod graphics_tests {
    use super::*;
    use bevy::render::view::screenshot::{Screenshot, save_to_disk};

    struct SmokeProgress {
        frames: u32,
        first_rx: bool,
        started: std::time::Instant,
    }

    impl Default for SmokeProgress {
        fn default() -> Self {
            Self {
                frames: 0,
                first_rx: false,
                started: std::time::Instant::now(),
            }
        }
    }

    fn smoke_path(name: &str) -> std::path::PathBuf {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../target")
            .join(name)
    }

    fn smoke_step(
        mut commands: Commands,
        stats: Res<stream::StreamStats>,
        mut mode: ResMut<camera::CameraMode>,
        mut view: ResMut<stream::ReceiverView>,
        mut link: ResMut<stream::VideoLinkConfig>,
        mut progress: Local<SmokeProgress>,
        mut exit: MessageWriter<bevy::app::AppExit>,
    ) {
        progress.first_rx |= stats.received_frames >= 3;
        assert!(
            progress.started.elapsed().as_secs() < 50,
            "GPU/video smoke test stalled: {:?}, frames RX {}",
            stats.state,
            stats.received_frames
        );
        if !progress.first_rx {
            return;
        }
        progress.frames += 1;
        match progress.frames {
            120 => {
                *mode = camera::CameraMode::Chase;
            }
            150 => {
                commands
                    .spawn(Screenshot::primary_window())
                    .observe(save_to_disk(smoke_path("ground-control.png")));
            }
            180 => {
                view.0 = true;
                *link = stream::VideoLinkConfig::for_profile(stream::VideoProfile::Poor);
            }
            240 => {
                commands
                    .spawn(Screenshot::primary_window())
                    .observe(save_to_disk(smoke_path("video-receiver.png")));
            }
            270 => {
                *link = stream::VideoLinkConfig::for_profile(stream::VideoProfile::Lost);
            }
            300 => {
                commands
                    .spawn(Screenshot::primary_window())
                    .observe(save_to_disk(smoke_path("video-lost.png")));
            }
            330.. => {
                exit.write(bevy::app::AppExit::Success);
            }
            _ => {}
        }
    }

    #[test]
    #[ignore = "requires a graphics adapter and window system; saves screenshots under target/"]
    fn graphical_smoke_receives_real_gpu_video_and_composites_hud() {
        let mut app = build_app(false);
        app.add_systems(Update, smoke_step.after(hud::update_hud));
        assert_eq!(app.run(), bevy::app::AppExit::Success);
        for file in ["ground-control.png", "video-receiver.png", "video-lost.png"] {
            assert!(std::fs::metadata(smoke_path(file)).unwrap().len() > 1000);
        }
    }
}

#[cfg(test)]
mod flight_tests {
    use super::*;
    use ::mavlink::MavConnection;
    use std::net::UdpSocket;
    use std::time::Duration;

    #[test]
    fn mavlink_parameter_commands_update_simulation_limits_and_reply() {
        let probe = UdpSocket::bind("127.0.0.1:0").unwrap();
        let port = probe.local_addr().unwrap().port();
        drop(probe);

        let mut app = build_app(true);
        app.world_mut()
            .resource_mut::<mavlink::MavlinkConfig>()
            .connection_string = format!("udpin:0.0.0.0:{port}");
        app.finish();
        app.cleanup();
        app.update();

        let client = ::mavlink::connect::<::mavlink::dialects::common::MavMessage>(&format!(
            "udpout:127.0.0.1:{port}"
        ))
        .unwrap();
        client
            .send(
                &::mavlink::MavHeader {
                    system_id: 255,
                    component_id: 190,
                    sequence: 0,
                },
                &::mavlink::dialects::common::MavMessage::PARAM_SET(
                    ::mavlink::dialects::common::PARAM_SET_DATA {
                        param_value: 5.0,
                        target_system: 1,
                        target_component: 1,
                        param_id: ::mavlink::types::CharArray::new(*b"VYDE_MAX_SPEED\0\0"),
                        param_type:
                            ::mavlink::dialects::common::MavParamType::MAV_PARAM_TYPE_REAL32,
                    },
                ),
            )
            .unwrap();

        let deadline = std::time::Instant::now() + Duration::from_secs(2);
        let reply = loop {
            app.update();
            if app.world().resource::<FlightControlLimits>().max_speed_m_s == 5.0 {
                if let Ok((_, ::mavlink::dialects::common::MavMessage::PARAM_VALUE(data))) =
                    client.try_recv()
                {
                    if data.param_id.to_str().ok() == Some("VYDE_MAX_SPEED") {
                        break data.param_value;
                    }
                }
            }
            assert!(
                std::time::Instant::now() < deadline,
                "MAVLink parameter was not applied"
            );
            std::thread::sleep(Duration::from_millis(5));
        };
        assert_eq!(reply, 5.0);
    }

    fn flight_app(env: EnvironmentConfig) -> App {
        flight_app_for_drone(env, DroneConfig::default())
    }

    fn flight_app_for_drone(env: EnvironmentConfig, drone: DroneConfig) -> App {
        let mut app = build_app(true);
        app.insert_resource(env);
        app.insert_resource(drone);
        app.insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
            Duration::from_secs_f64(1.0 / 240.0),
        ));
        app.finish();
        app.cleanup();
        app.update();
        {
            let mut interface = app.world_mut().resource_mut::<mavlink::MavlinkInterface>();
            interface.armed = true;
            interface.external_control = true;
            interface.guided_target = Some(mavlink::GuidedTarget::Position {
                position: Vec3::Y * 2.0,
                yaw: Some(0.0),
            });
        }
        app
    }

    fn calm_air() -> EnvironmentConfig {
        EnvironmentConfig {
            wind_constant: Vec3::ZERO,
            wind_gust_intensity: 0.0,
            turbulence_intensity: 0.0,
            ..default()
        }
    }

    fn step(app: &mut App, ticks: usize) {
        for _ in 0..ticks {
            app.update();
            let telemetry = app.world().resource::<Telemetry>();
            assert!(telemetry.position.is_finite());
            assert!(telemetry.rotation.is_finite());
            assert!(telemetry.velocity.is_finite());
            assert!(
                telemetry.angular_velocity.length() < 10.0,
                "unstable body rate: {telemetry:?}"
            );
        }
    }

    #[test]
    fn actual_physics_recovers_tilt_and_holds_hover() {
        let mut app = flight_app(calm_air());
        {
            let world = app.world_mut();
            let mut query =
                world.query_filtered::<(&mut Transform, &mut Rotation), With<motors::Drone>>();
            for (mut tf, mut rotation) in query.iter_mut(world) {
                tf.rotation = Quat::from_euler(EulerRot::YXZ, 0.5, 0.25, -0.2);
                rotation.0 = tf.rotation;
            }
        }
        step(&mut app, 240 * 12);
        let telemetry = app.world().resource::<Telemetry>();
        assert!(
            (telemetry.position - Vec3::Y * 2.0).length() < 0.2,
            "{telemetry:?}"
        );
        assert!(telemetry.velocity.length() < 0.15, "{telemetry:?}");
        assert!((telemetry.rotation * Vec3::Y).y > 0.99);
        assert_eq!(
            app.world()
                .resource::<domain_randomization::ResetGeneration>()
                .0,
            0
        );
    }

    #[test]
    fn actual_physics_hovers_with_offset_center_of_mass_and_heavier_frame() {
        let cfg = DroneConfig {
            mass: 1.8,
            center_of_mass_offset: Vec3::new(0.02, 0.005, -0.015),
            ..default()
        };
        let mut app = flight_app_for_drone(EnvironmentConfig::default(), cfg);
        step(&mut app, 240 * 15);
        let telemetry = app.world().resource::<Telemetry>();
        assert!(
            (telemetry.position - Vec3::Y * 2.0).length() < 0.4,
            "{telemetry:?}"
        );
        assert!(telemetry.velocity.length() < 0.3, "{telemetry:?}");
        assert_eq!(
            app.world()
                .resource::<domain_randomization::ResetGeneration>()
                .0,
            0
        );
    }

    #[test]
    fn actual_physics_rejects_gusts_under_mavlink_guidance() {
        let mut app = flight_app(EnvironmentConfig::default());
        step(&mut app, 240 * 15);
        let telemetry = app.world().resource::<Telemetry>();
        assert!(
            (telemetry.position - Vec3::Y * 2.0).length() < 0.4,
            "{telemetry:?}"
        );
        assert!(telemetry.velocity.length() < 0.3, "{telemetry:?}");
        assert_eq!(
            app.world()
                .resource::<domain_randomization::ResetGeneration>()
                .0,
            0
        );
    }

    #[test]
    fn mavlink_position_setpoint_moves_then_holds_aircraft() {
        let mut app = flight_app(calm_air());
        app.world_mut()
            .resource_mut::<mavlink::MavlinkInterface>()
            .guided_target = Some(mavlink::GuidedTarget::Position {
            position: Vec3::new(0.0, 2.0, 5.0),
            yaw: Some(0.0),
        });
        step(&mut app, 240 * 2);
        let stopped_position = app.world().resource::<Telemetry>().position;
        assert!(stopped_position.z > 2.0, "{stopped_position:?}");
        app.world_mut()
            .resource_mut::<mavlink::MavlinkInterface>()
            .guided_target = Some(mavlink::GuidedTarget::Position {
            position: stopped_position,
            yaw: Some(0.0),
        });
        step(&mut app, 240 * 10);
        let telemetry = app.world().resource::<Telemetry>();
        assert!(
            (telemetry.position - stopped_position).length() < 0.4,
            "{telemetry:?}"
        );
        assert!(telemetry.velocity.length() < 0.15, "{telemetry:?}");
    }

    #[test]
    fn fixed_control_is_independent_of_render_frame_rate() {
        let mut positions = Vec::new();
        for fps in [30, 60, 120] {
            let mut app = flight_app(EnvironmentConfig::default());
            app.insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
                Duration::from_secs_f64(1.0 / fps as f64),
            ));
            step(&mut app, fps * 10);
            positions.push(app.world().resource::<Telemetry>().position);
        }
        for position in &positions[1..] {
            assert!((*position - positions[0]).length() < 0.02, "{positions:?}");
        }
    }

    #[test]
    fn real_obstacle_contact_crashes_once_then_resets_pose() {
        let mut app = flight_app(calm_air());
        {
            let world = app.world_mut();
            let mut query =
                world.query_filtered::<(&mut Transform, &mut Position), With<motors::Drone>>();
            for (mut tf, mut position) in query.iter_mut(world) {
                tf.translation = Vec3::new(0.0, 0.5, 2.0);
                position.0 = tf.translation;
            }
        }
        for _ in 0..4 {
            app.update();
        }
        assert_eq!(
            app.world()
                .resource::<domain_randomization::ResetGeneration>()
                .0,
            1
        );
        assert!(app.world().resource::<environment::Explosion>().active);
        assert!((app.world().resource::<Telemetry>().position - Vec3::Y * 2.0).length() < 0.05);
    }

    #[test]
    fn unpowered_body_lands_on_ground_instead_of_falling_through() {
        let mut app = flight_app(calm_air());
        app.world_mut()
            .resource_mut::<mavlink::MavlinkInterface>()
            .armed = false;
        step(&mut app, 240 * 3);
        let telemetry = app.world().resource::<Telemetry>();
        assert!(
            telemetry.position.y > 0.04 && telemetry.position.y < 0.08,
            "{telemetry:?}"
        );
        assert!(telemetry.velocity.y.abs() < 0.05, "{telemetry:?}");
        assert_eq!(
            app.world()
                .resource::<domain_randomization::ResetGeneration>()
                .0,
            0
        );
    }

    #[test]
    fn reset_clears_rotors_and_disarms_the_simulated_aircraft() {
        let mut app = flight_app(calm_air());
        step(&mut app, 240);
        app.world_mut()
            .resource_mut::<domain_randomization::ResetRequest>()
            .0 = true;
        step(&mut app, 1);
        assert_eq!(
            app.world()
                .resource::<domain_randomization::ResetGeneration>()
                .0,
            1
        );
        let interface = app.world().resource::<mavlink::MavlinkInterface>();
        assert!(!interface.armed);
        assert!(interface.guided_target.is_none());
        assert!(
            app.world()
                .resource::<motor_rpms::MotorRPMs>()
                .rpms
                .iter()
                .all(|rpm| *rpm < 500.0)
        );
        step(&mut app, 240 * 5);
        assert!(app.world().resource::<Telemetry>().position.y < 0.1);
    }
}
