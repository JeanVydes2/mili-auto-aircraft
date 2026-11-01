use crate::config::{SensorConfig, SimConfig};
use bevy::asset::RenderAssetUsages;
use bevy::camera::RenderTarget;
use bevy::image::Image;
use bevy::input::mouse::AccumulatedMouseMotion;
use bevy::prelude::*;
use bevy::render::render_resource::TextureFormat;
use bevy::ui::IsDefaultUiCamera;
use bevy::window::CursorGrabMode;
use bevy::window::{CursorOptions, PrimaryWindow};

#[derive(Resource, Clone, Copy, PartialEq, Eq, Debug)]
pub enum CameraMode {
    Fpv,
    Chase,
    Free,
}

impl Default for CameraMode {
    fn default() -> Self {
        CameraMode::Fpv
    }
}

#[derive(Resource, Default)]
pub struct FreeCamActive(pub bool);

#[derive(Resource, Clone, Debug)]
pub struct CameraFrameBuffer {
    pub image: Handle<Image>,
    pub width: u32,
    pub height: u32,
}

#[derive(Component)]
pub struct DroneCamera;

#[derive(Component)]
pub struct FpvAttached;

#[derive(Component)]
pub struct FrameCamera;

#[derive(Component)]
pub struct FreeCamera;

pub fn setup_cameras(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    sensor_cfg: Res<SensorConfig>,
    sim: Res<SimConfig>,
) {
    if sim.headless {
        return;
    }

    let width = sensor_cfg.camera_width;
    let height = sensor_cfg.camera_height;

    let mut image = Image::new_target_texture(width, height, TextureFormat::Rgba8UnormSrgb, None);
    image.asset_usage = RenderAssetUsages::RENDER_WORLD;
    let image_handle = images.add(image);

    commands.insert_resource(CameraFrameBuffer {
        image: image_handle.clone(),
        width,
        height,
    });

    // Free camera (inactive by default).
    commands.spawn((
        FreeCamera,
        Camera3d::default(),
        Msaa::Off,
        Camera {
            is_active: false,
            order: 10,
            ..default()
        },
        Transform::from_xyz(0.0, 5.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
        Projection::Perspective(PerspectiveProjection {
            fov: 2.094,
            ..default()
        }),
    ));
    // UI camera.
    commands.spawn((
        Camera2d,
        IsDefaultUiCamera,
        Msaa::Off,
        Camera {
            order: 3,
            clear_color: ClearColorConfig::None,
            ..default()
        },
    ));
}

pub fn attach_fpv_camera(
    mut commands: Commands,
    drone_query: Query<Entity, (With<crate::motors::Drone>, Without<FpvAttached>)>,
    buffer: Option<Res<CameraFrameBuffer>>,
) {
    if let Some(buffer) = buffer {
        for drone in &drone_query {
            commands.entity(drone).insert(FpvAttached);
            // FPV view camera (window), child of drone.
            commands.entity(drone).with_children(|parent| {
                parent.spawn((
                    DroneCamera,
                    Camera3d::default(),
                    Msaa::Off,
                    Camera {
                        order: 0,
                        ..default()
                    },
                    Projection::Perspective(PerspectiveProjection {
                        fov: 2.094,
                        ..default()
                    }),
                    Transform::from_xyz(0.0, 0.5, 0.8)
                        .looking_at(Vec3::new(0.0, -2.0, 5.0), Vec3::Y),
                ));
            });
            // Frame extraction camera (render to image).
            commands.entity(drone).with_children(|parent| {
                parent.spawn((
                    FrameCamera,
                    Camera3d::default(),
                    Msaa::Off,
                    Camera {
                        order: 2,
                        ..default()
                    },
                    RenderTarget::Image(buffer.image.clone().into()),
                    Transform::from_xyz(0.0, 0.5, 0.8)
                        .looking_at(Vec3::new(0.0, -2.0, 5.0), Vec3::Y),
                ));
            });
        }
    }
}

pub fn toggle_freecam_system(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut free_active_res: ResMut<FreeCamActive>,
    mut params: ParamSet<(
        Query<&mut Camera, With<DroneCamera>>,
        Query<&mut Camera, With<FreeCamera>>,
    )>,
) {
    let ctrl = keyboard.pressed(KeyCode::ControlLeft) || keyboard.pressed(KeyCode::ControlRight);
    if ctrl && keyboard.just_pressed(KeyCode::KeyF) {
        let free_active = params.p1().iter().any(|c| c.is_active);
        let new_state = !free_active;
        for mut cam in params.p1().iter_mut() {
            cam.is_active = new_state;
        }
        for mut cam in params.p0().iter_mut() {
            cam.is_active = !new_state;
        }
        free_active_res.0 = new_state;
    }
}

pub fn freecam_movement_system(
    time: Res<Time>,
    keyboard: Res<ButtonInput<KeyCode>>,
    free_active: Res<FreeCamActive>,
    mut query: Query<&mut Transform, With<FreeCamera>>,
) {
    if !free_active.0 {
        return;
    }
    for mut transform in &mut query {
        bevy::log::info_once!("free cam active, moving");
        // Only move if there is an active free camera; check by transform? We'll just move.
        let dt = time.delta_secs();
        let speed = 12.0;
        let forward = transform.forward().as_vec3();
        let right = transform.right().as_vec3();
        // Horizontal projection.
        let fwd_h = Vec3::new(forward.x, 0.0, forward.z).normalize_or_zero();
        let right_h = Vec3::new(right.x, 0.0, right.z).normalize_or_zero();

        let mut dir = Vec3::ZERO;
        if keyboard.pressed(KeyCode::KeyW) {
            dir += fwd_h;
        }
        if keyboard.pressed(KeyCode::KeyS) {
            dir -= fwd_h;
        }
        if keyboard.pressed(KeyCode::KeyA) {
            dir -= right_h;
        }
        if keyboard.pressed(KeyCode::KeyD) {
            dir += right_h;
        }
        if keyboard.pressed(KeyCode::Space) {
            dir.y += 1.0;
        }
        if keyboard.pressed(KeyCode::ShiftLeft) || keyboard.pressed(KeyCode::ShiftRight) {
            dir.y -= 1.0;
        }

        if dir.length_squared() > 0.0 {
            transform.translation += dir.normalize() * speed * dt;
        }

        // Mouse look not implemented (no cursor grab). Keep orientation fixed unless keys.
        // Optional: Q/E to yaw.
        if keyboard.pressed(KeyCode::KeyQ) {
            transform.rotate_y(1.5 * dt);
        }
        if keyboard.pressed(KeyCode::KeyE) {
            transform.rotate_y(-1.5 * dt);
        }
    }
}

pub fn cursor_grab_system(
    free_active: Res<FreeCamActive>,
    mut cursors: Query<&mut CursorOptions, With<PrimaryWindow>>,
) {
    for mut cursor in &mut cursors {
        if free_active.0 {
            cursor.visible = false;
            cursor.grab_mode = CursorGrabMode::Locked;
        } else {
            cursor.visible = true;
            cursor.grab_mode = CursorGrabMode::None;
        }
    }
}

pub fn log_cameras(query: Query<&Camera>) {
    let total = query.iter().count();
    let active: Vec<_> = query
        .iter()
        .filter(|c| c.is_active)
        .map(|c| c.order)
        .collect();
    bevy::log::info!("cameras total {} active orders {:?}", total, active);
}

pub fn init_cameras(mut free_cameras: Query<&mut Camera, With<FreeCamera>>) {
    for mut cam in &mut free_cameras {
        cam.is_active = false;
    }
}

pub fn freecam_mouse_look_system(
    free_active: Res<FreeCamActive>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    mut query: Query<&mut Transform, With<FreeCamera>>,
) {
    if !free_active.0 {
        return;
    }
    let delta = mouse_motion.delta;
    if delta == Vec2::ZERO {
        return;
    }
    let sens = 0.002;
    for mut transform in &mut query {
        let (yaw, pitch, _) = transform.rotation.to_euler(EulerRot::YXZ);
        let new_yaw = yaw - delta.x * sens;
        let new_pitch = (pitch - delta.y * sens).clamp(-1.4, 1.4);
        transform.rotation = Quat::from_euler(EulerRot::YXZ, new_yaw, new_pitch, 0.0);
    }
}
