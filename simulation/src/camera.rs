use crate::config::{SensorConfig, SimConfig};
use crate::motors::Drone;
use bevy::asset::RenderAssetUsages;
use bevy::camera::RenderTarget;
use bevy::input::mouse::AccumulatedMouseMotion;
use bevy::prelude::*;
use bevy::render::render_resource::{TextureFormat, TextureUsages};
use bevy::ui::IsDefaultUiCamera;
use bevy::window::{CursorGrabMode, CursorOptions, PrimaryWindow};

#[derive(Resource, Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum CameraMode {
    #[default]
    Fpv,
    Chase,
    Free,
}

#[derive(Resource, Default)]
pub struct FreeCamActive(pub bool);

#[derive(Resource, Clone, Debug)]
pub struct CameraFrameBuffer {
    pub image: Handle<Image>,
    pub width: u32,
    pub height: u32,
}

/// Per-vehicle FPV frame buffers. Index == `VehicleIndex`, mirroring the
/// MAVLink system_id and video feed numbering. Vehicle 0 keeps behavior
/// identical to the legacy single `CameraFrameBuffer`.
#[derive(Resource, Default)]
pub struct CameraFrameBuffers {
    pub buffers: Vec<CameraFrameBuffer>,
}

impl CameraFrameBuffers {
    pub fn get(&self, index: usize) -> Option<&CameraFrameBuffer> {
        self.buffers.get(index)
    }
    pub fn len(&self) -> usize {
        self.buffers.len()
    }
    pub fn is_empty(&self) -> bool {
        self.buffers.is_empty()
    }
}

#[derive(Component)]
pub struct FrameCamera {
    /// Vehicle this offscreen camera belongs to (`VehicleIndex`).
    pub vehicle: usize,
}
#[derive(Component)]
pub struct FpvAttached;
#[derive(Component)]
pub struct FreeCamera;
#[derive(Component)]
pub struct ChaseCamera;
#[derive(Component)]
pub struct ViewCamera(CameraMode);

pub fn setup_cameras(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    sensor: Res<SensorConfig>,
    sim: Res<SimConfig>,
) {
    if sim.headless {
        return;
    }
    let width = sensor.camera_width.clamp(64, 1280);
    let height = sensor.camera_height.clamp(64, 720);
    let feed_count = std::env::var("MULTI_AIRCRAFT")
        .ok()
        .map(|s| s.split(',').filter(|n| !n.trim().is_empty()).count())
        .or_else(|| {
            std::env::var("MULTI_COUNT")
                .ok()
                .and_then(|s| s.parse::<usize>().ok())
        })
        .unwrap_or(1)
        .max(1);
    let mut buffers = Vec::with_capacity(feed_count);
    for _ in 0..feed_count {
        let mut source =
            Image::new_target_texture(width, height, TextureFormat::Rgba8UnormSrgb, None);
        source.asset_usage = RenderAssetUsages::RENDER_WORLD;
        source.texture_descriptor.usage |= TextureUsages::COPY_SRC;
        buffers.push(CameraFrameBuffer {
            image: images.add(source),
            width,
            height,
        });
    }
    // Keep legacy single-buffer resource pointing at vehicle 0 for compat.
    if let Some(first) = buffers.first() {
        commands.insert_resource(first.clone());
    }
    commands.insert_resource(CameraFrameBuffers { buffers });
    commands.spawn((
        FreeCamera,
        ViewCamera(CameraMode::Free),
        Camera3d::default(),
        Msaa::Off,
        Camera {
            is_active: false,
            order: 0,
            ..default()
        },
        Transform::from_xyz(0.0, 5.0, -10.0).looking_at(Vec3::Y * 2.0, Vec3::Y),
        Projection::Perspective(PerspectiveProjection {
            fov: 70.0_f32.to_radians(),
            ..default()
        }),
    ));
    commands.spawn((
        ChaseCamera,
        ViewCamera(CameraMode::Chase),
        Camera3d::default(),
        Msaa::Off,
        Camera {
            is_active: false,
            order: 0,
            ..default()
        },
        Transform::from_xyz(0.0, 4.0, -5.0).looking_at(Vec3::Y * 2.0, Vec3::Y),
    ));
    // UI must composite after every local camera, including free/chase.
    commands.spawn((
        Camera2d,
        IsDefaultUiCamera,
        Msaa::Off,
        Camera {
            order: 100,
            clear_color: ClearColorConfig::None,
            ..default()
        },
    ));
}

pub fn attach_fpv_camera(
    mut commands: Commands,
    drones: Query<
        (Entity, Option<&crate::aircraft::VehicleIndex>),
        (With<crate::aircraft::Aircraft>, Without<FpvAttached>),
    >,
    buffers: Res<CameraFrameBuffers>,
    sensor: Res<SensorConfig>,
) {
    for (drone, vehicle_index) in &drones {
        let vehicle = vehicle_index.map(|v| v.0).unwrap_or(0);
        let Some(buffer) = buffers.get(vehicle) else {
            continue;
        };
        commands
            .entity(drone)
            .insert(FpvAttached)
            .with_children(|parent| {
                let projection = Projection::Perspective(PerspectiveProjection {
                    fov: sensor.camera_fov_deg.clamp(30.0, 150.0).to_radians(),
                    ..default()
                });
                // Both cameras share optics/pose, but only the offscreen source is
                // packetized. The received image never points at the live texture.
                parent.spawn((
                    ViewCamera(CameraMode::Fpv),
                    Camera3d::default(),
                    Msaa::Off,
                    Camera {
                        order: 0,
                        ..default()
                    },
                    projection.clone(),
                    fpv_transform(),
                ));
                parent.spawn((
                    FrameCamera { vehicle },
                    Camera3d::default(),
                    Msaa::Off,
                    Camera {
                        is_active: false,
                        order: -10,
                        ..default()
                    },
                    projection,
                    RenderTarget::Image(buffer.image.clone().into()),
                    fpv_transform(),
                ));
            });
    }
}

fn fpv_transform() -> Transform {
    Transform::from_xyz(0.0, 0.08, 0.34).looking_at(Vec3::new(0.0, -0.28, 5.0), Vec3::Y)
}

pub fn toggle_freecam_system(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut active: ResMut<FreeCamActive>,
    mut mode: ResMut<CameraMode>,
    mut previous: Local<CameraMode>,
) {
    let ctrl = keyboard.pressed(KeyCode::ControlLeft) || keyboard.pressed(KeyCode::ControlRight);
    if ctrl && keyboard.just_pressed(KeyCode::KeyF) {
        if *mode == CameraMode::Free {
            *mode = *previous;
        } else {
            *previous = *mode;
            *mode = CameraMode::Free;
        }
    }
    if keyboard.just_pressed(KeyCode::KeyC) && *mode != CameraMode::Free {
        *mode = if *mode == CameraMode::Fpv {
            CameraMode::Chase
        } else {
            CameraMode::Fpv
        };
    }
    active.0 = *mode == CameraMode::Free;
}

pub fn update_view_cameras(mode: Res<CameraMode>, mut cameras: Query<(&ViewCamera, &mut Camera)>) {
    for (view, mut camera) in &mut cameras {
        camera.is_active = view.0 == *mode;
    }
}

pub fn chase_camera_system(
    time: Res<Time>,
    drone: Query<&Transform, With<crate::aircraft::Aircraft>>,
    mut chase: Query<
        &mut Transform,
        (
            With<ChaseCamera>,
            Without<Drone>,
            Without<crate::aircraft::Aircraft>,
        ),
    >,
) {
    let Some(drone) = drone.iter().next() else {
        return;
    };
    let yaw = Quat::from_rotation_y(drone.rotation.to_euler(EulerRot::YXZ).0);
    let target = drone.translation + yaw * Vec3::new(0.0, 2.0, -4.5);
    let look_at = drone.translation + yaw * Vec3::Z * 1.2;
    let alpha = 1.0 - (-time.delta_secs() * 6.0).exp();
    for mut camera in &mut chase {
        camera.translation = camera.translation.lerp(target, alpha);
        camera.look_at(look_at, Vec3::Y);
    }
}

pub fn freecam_movement_system(
    time: Res<Time>,
    keyboard: Res<ButtonInput<KeyCode>>,
    active: Res<FreeCamActive>,
    mut cameras: Query<&mut Transform, With<FreeCamera>>,
) {
    if !active.0 {
        return;
    }
    for mut camera in &mut cameras {
        let forward = camera.forward().as_vec3();
        let right = camera.right().as_vec3();
        let forward = Vec3::new(forward.x, 0.0, forward.z).normalize_or_zero();
        let right = Vec3::new(right.x, 0.0, right.z).normalize_or_zero();
        let mut direction = Vec3::ZERO;
        if keyboard.pressed(KeyCode::KeyW) {
            direction += forward;
        }
        if keyboard.pressed(KeyCode::KeyS) {
            direction -= forward;
        }
        if keyboard.pressed(KeyCode::KeyD) {
            direction += right;
        }
        if keyboard.pressed(KeyCode::KeyA) {
            direction -= right;
        }
        if keyboard.pressed(KeyCode::Space) {
            direction.y += 1.0;
        }
        if keyboard.pressed(KeyCode::ShiftLeft) || keyboard.pressed(KeyCode::ShiftRight) {
            direction.y -= 1.0;
        }
        camera.translation += direction.normalize_or_zero() * 12.0 * time.delta_secs();
        let yaw = f32::from(u8::from(keyboard.pressed(KeyCode::KeyQ)))
            - f32::from(u8::from(keyboard.pressed(KeyCode::KeyE)));
        camera.rotate_y(yaw * time.delta_secs() * 1.5);
    }
}

pub fn cursor_grab_system(
    active: Res<FreeCamActive>,
    mut cursors: Query<&mut CursorOptions, With<PrimaryWindow>>,
) {
    for mut cursor in &mut cursors {
        cursor.visible = !active.0;
        cursor.grab_mode = if active.0 {
            CursorGrabMode::Locked
        } else {
            CursorGrabMode::None
        };
    }
}

pub fn freecam_mouse_look_system(
    active: Res<FreeCamActive>,
    motion: Res<AccumulatedMouseMotion>,
    mut cameras: Query<&mut Transform, With<FreeCamera>>,
) {
    if !active.0 || motion.delta == Vec2::ZERO {
        return;
    }
    for mut camera in &mut cameras {
        let (yaw, pitch, _) = camera.rotation.to_euler(EulerRot::YXZ);
        camera.rotation = Quat::from_euler(
            EulerRot::YXZ,
            yaw - motion.delta.x * 0.002,
            (pitch - motion.delta.y * 0.002).clamp(-1.4, 1.4),
            0.0,
        );
    }
}
