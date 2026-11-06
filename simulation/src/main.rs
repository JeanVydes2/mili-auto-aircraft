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
mod visual_settings;

use avian3d::prelude::*;
use bevy::prelude::*;
use config::*;

use mavlink::SharedMavlink as _SharedMavlink;

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
        .insert_resource(SensorConfig::default())
        .insert_resource(DomainRandomizationConfig::default())
        .insert_resource(CameraConfig::default())
        .insert_resource(mavlink::MavlinkConfig::default())
        .insert_resource(domain_randomization::ResetRequest(false))
        .init_resource::<domain_randomization::ResetGeneration>()
        .insert_resource(camera::CameraMode::default())
        .insert_resource(camera::FreeCamActive::default())
        .insert_resource(config::GizmosEnabled::default())
        .insert_resource(config::GuidanceVisualsEnabled::default())
        .init_resource::<stream::VideoLinkConfig>()
        .init_resource::<stream::VideoReceivers>()
        .init_resource::<stream::StreamStatsList>()
        .init_resource::<stream::VideoOutput>()
        .init_resource::<stream::ReceiverView>()
        .init_resource::<hud::HudCharts>()
        .init_resource::<visual_settings::VisualSettingsState>()
        .init_resource::<environment::WindSettings>()
        .init_resource::<camera::CameraFrameBuffers>();

    if headless {
        app.add_plugins((MinimalPlugins, TransformPlugin));
        app.add_plugins(bevy::asset::AssetPlugin::default());
        app.add_plugins(bevy::input::InputPlugin::default());
        app.add_plugins(bevy::log::LogPlugin::default());
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
            mavlink::setup_mavlink,
        )
            .chain(),
    );

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
                visual_settings::setup_visual_settings,
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
                visual_settings::visual_settings_input,
                visual_settings::apply_visual_settings,
                hud::update_hud,
                visuals::animate_visuals,
            )
                .chain(),
        );
        app.add_systems(
            Update,
            telemetry_log::draw_gizmos_system.after(hud::update_hud),
        );
        app.add_systems(Update, environment::animate_explosion_particles);
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

pub fn setup_scene(
    mut commands: Commands,
    mut drone_cfg: ResMut<DroneConfig>,
    catalog: Res<aircraft::AircraftCatalog>,
) {
    // Multi-vehicle: MULTI_AIRCRAFT="quad,hex" or MULTI_COUNT=N spawns N drones.
    let multi = std::env::var("MULTI_AIRCRAFT")
        .ok()
        .map(|s| {
            s.split(',')
                .map(|name| match name.trim().to_ascii_lowercase().as_str() {
                    "hex" => aircraft::AircraftType::Hex,
                    "fixed_wing" | "fixedwing" => aircraft::AircraftType::FixedWing,
                    "vtol" => aircraft::AircraftType::Vtol,
                    _ => aircraft::AircraftType::Quad,
                })
                .collect::<Vec<_>>()
        })
        .unwrap_or_else(|| {
            let count = std::env::var("MULTI_COUNT")
                .ok()
                .and_then(|s| s.parse::<usize>().ok())
                .unwrap_or(1);
            (0..count).map(|_| catalog.selected_kind()).collect()
        });
    let count = multi.len().max(1);
    for (i, kind) in multi.into_iter().enumerate() {
        let profile = catalog.profile(kind);
        let offset = spawn_position(kind, i);
        let entity = spawn_profile_at(
            &mut commands,
            &mut drone_cfg,
            &profile,
            offset,
        );
        commands.entity(entity).insert(aircraft::VehicleIndex(i));
    }
    let _ = count;
}

fn spawn_position(kind: aircraft::AircraftType, index: usize) -> Vec3 {
    if kind == aircraft::AircraftType::FixedWing {
        return Vec3::new(-28.0, 0.28, -22.0 + index as f32 * 4.0);
    }
    Vec3::new(
        (index as f32 % 3.0) * 8.0 - 8.0,
        2.0,
        (index as f32 / 3.0) * 8.0,
    )
}

fn spawn_profile(
    commands: &mut Commands,
    drone_cfg: &mut DroneConfig,
    profile: &aircraft::AircraftDefinition,
) -> Entity {
    spawn_profile_at(commands, drone_cfg, profile, Vec3::new(0.0, 2.0, 0.0))
}

fn spawn_profile_at(
    commands: &mut Commands,
    drone_cfg: &mut DroneConfig,
    profile: &aircraft::AircraftDefinition,
    position: Vec3,
) -> Entity {
    let entity = if profile.kind == aircraft::AircraftType::Quad {
        drone_cfg.mass = profile.body.mass;
        drone_cfg.inertia = profile.body.inertia;
        drone_cfg.center_of_mass_offset = profile.body.center_of_mass;
        drone_cfg.linear_drag = profile.body.linear_drag;
        drone_cfg.induced_drag = profile.body.quadratic_drag.max_element();
        drone_cfg.angular_drag = profile.body.angular_drag;
        physics::spawn_quad(commands, drone_cfg, position, profile.collider_half_extents)
    } else {
        profile.spawn(commands, position)
    };
    entity
}

fn select_aircraft_system(
    mut commands: Commands,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut catalog: ResMut<aircraft::AircraftCatalog>,
    mut drone_cfg: ResMut<DroneConfig>,
    active: Query<(Entity, Option<&aircraft::VehicleIndex>), With<aircraft::Aircraft>>,
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
    active: Query<(Entity, Option<&aircraft::VehicleIndex>), With<aircraft::Aircraft>>,
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
    active: &Query<(Entity, Option<&aircraft::VehicleIndex>), With<aircraft::Aircraft>>,
    index: usize,
) {
    if index >= catalog.definitions.len() || index == catalog.selected {
        return;
    }
    catalog.selected = index;
    // Preserve indices across despawn/respawn so MAVLink system_id, FPV
    // buffers and video feeds stay mapped to the same vehicle slot.
    let mut indices: Vec<usize> = active
        .iter()
        .map(|(_, idx)| idx.map(|v| v.0).unwrap_or(0))
        .collect();
    indices.sort_unstable();
    indices.dedup();
    if indices.is_empty() {
        indices.push(0);
    }
    for (entity, _) in active.iter() {
        commands.entity(entity).despawn();
    }
    for (slot, vehicle_index) in indices.into_iter().enumerate() {
        let kind = catalog.definitions[catalog.selected].kind;
        let entity = spawn_profile_at(
            commands,
            drone_cfg,
            &catalog.definitions[catalog.selected],
            spawn_position(kind, vehicle_index),
        );
        commands
            .entity(entity)
            .insert(aircraft::VehicleIndex(vehicle_index));
        let _ = slot;
    }
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
        stats_list: Res<stream::StreamStatsList>,
        mut mode: ResMut<camera::CameraMode>,
        mut view: ResMut<stream::ReceiverView>,
        mut link: ResMut<stream::VideoLinkConfig>,
        mut progress: Local<SmokeProgress>,
        mut exit: MessageWriter<bevy::app::AppExit>,
    ) {
        let fallback = stream::StreamStats::default();
        let stats = stats_list.primary().unwrap_or(&fallback);
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

    fn telemetry_of(app: &mut App) -> sensors::Telemetry {
        let world = app.world_mut();
        world
            .query::<&sensors::Telemetry>()
            .iter(world)
            .next()
            .cloned()
            .unwrap_or_default()
    }
    fn set_interface(app: &mut App, index: usize, f: impl FnOnce(&mut mavlink::MavlinkInterface)) {
        let world = app.world_mut();
        let mut q = world.query::<&mut mavlink::MavlinkInterface>();
        for (i, mut interface) in q.iter_mut(world).enumerate() {
            if i == index {
                f(&mut interface);
                break;
            }
        }
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
        set_interface(&mut app, 0, |interface| {
            interface.armed = true;
            interface.external_control = true;
            interface.guided_target = Some(mavlink::GuidedTarget::Position {
                position: Vec3::Y * 2.0,
                yaw: Some(0.0),
            });
        });
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
            let telemetry = telemetry_of(&mut *app);
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
        let telemetry = telemetry_of(&mut app);
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
        let telemetry = telemetry_of(&mut app);
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
        let telemetry = telemetry_of(&mut app);
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
        set_interface(&mut app, 0, |interface| {
            interface.guided_target = Some(mavlink::GuidedTarget::Position {
                position: Vec3::new(0.0, 2.0, 5.0),
                yaw: Some(0.0),
            });
        });
        step(&mut app, 240 * 2);
        let stopped_position = telemetry_of(&mut app).position;
        assert!(stopped_position.z > 2.0, "{stopped_position:?}");
        set_interface(&mut app, 0, |interface| {
            interface.guided_target = Some(mavlink::GuidedTarget::Position {
                position: stopped_position,
                yaw: Some(0.0),
            });
        });
        step(&mut app, 240 * 10);
        let telemetry = telemetry_of(&mut app);
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
            positions.push(telemetry_of(&mut app).position);
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
            let mut query = world.query_filtered::<(
                &mut Transform,
                &mut Position,
                &mut avian3d::prelude::LinearVelocity,
            ), With<motors::Drone>>();
            for (mut tf, mut position, mut velocity) in query.iter_mut(world) {
                tf.translation = Vec3::new(0.0, 0.5, 2.0);
                position.0 = tf.translation;
                // High impact speed toward the cube: must explode.
                velocity.0 = Vec3::new(0.0, -5.0, 0.0);
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
        assert!((telemetry_of(&mut app).position - Vec3::Y * 2.0).length() < 0.05);
    }

    #[test]
    fn unpowered_body_lands_on_ground_instead_of_falling_through() {
        let mut app = flight_app(calm_air());
        set_interface(&mut app, 0, |interface| {
            interface.armed = false;
        });
        step(&mut app, 240 * 3);
        let telemetry = telemetry_of(&mut app);
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
        {
            let world = app.world_mut();
            let mut q = world.query::<&mavlink::MavlinkInterface>();
            let interface = q.iter(world).next().unwrap();
            assert!(!interface.armed);
            assert!(interface.guided_target.is_none());
            let mut rq = world.query::<&motor_rpms::MotorRPMs>();
            let rpms = rq.iter(world).next().unwrap();
            assert!(rpms.rpms.iter().all(|rpm| *rpm < 500.0));
        }
        step(&mut app, 240 * 5);
        assert!(telemetry_of(&mut app).position.y < 0.1);
    }
}
