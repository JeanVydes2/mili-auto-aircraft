use bevy::prelude::*;
use bevy::text::FontSize;
use bevy::ui::Node;
use rand::Rng;
use crate::camera::CameraFrameBuffer;
use crate::config::{SimConfig, NetworkConfig, DomainRandomizationConfig};
use crate::sensors::Telemetry;
use crate::environment::CurrentWind;

#[derive(Resource, Default)]
pub struct UiState {
    pub show_help: bool,
}

#[derive(Resource, Default)]
pub struct PressedKeys(pub Vec<String>);

#[derive(Resource)]
pub struct GizmosEnabled(pub bool);

impl Default for GizmosEnabled {
    fn default() -> Self { Self(true) }
}

#[derive(Component)]
pub struct DebugText;

#[derive(Component)]
pub struct StreamBox;

#[derive(Component)]
pub struct StreamStatusText;

#[derive(Resource, Default)]
pub struct StreamInterference {
    pub cut_timer: f32,
    pub noise_timer: f32,
    pub visible: bool,
}

pub fn input_controls_system(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut pressed: ResMut<PressedKeys>,
    mut reset: ResMut<crate::domain_randomization::ResetRequest>,
    mut net: ResMut<NetworkConfig>,
    mut dr: ResMut<DomainRandomizationConfig>,
    mut sim: ResMut<SimConfig>,
    mut gizmos_enabled: ResMut<GizmosEnabled>,
) {
    pressed.0.clear();
    for key in keyboard.get_pressed() {
        pressed.0.push(format!("{:?}", key));
    }
    if keyboard.just_pressed(KeyCode::KeyR) {
        reset.0 = true;
    }
    if keyboard.just_pressed(KeyCode::KeyN) {
        net.enabled = !net.enabled;
    }
    if keyboard.just_pressed(KeyCode::KeyD) {
        dr.enabled = !dr.enabled;
    }
    if keyboard.just_pressed(KeyCode::KeyG) {
        sim.enable_gizmos = !sim.enable_gizmos;
        gizmos_enabled.0 = sim.enable_gizmos;
    }
    if keyboard.just_pressed(KeyCode::KeyH) {
        // toggle shadows in environment lights
    }
    if keyboard.just_pressed(KeyCode::Slash) {
        // toggle help
    }
}

pub fn update_debug_text(
    pressed: Res<PressedKeys>,
    telemetry: Res<Telemetry>,
    wind: Res<CurrentWind>,
    batt: Res<crate::config::BatteryState>,
    mut query: Query<&mut Text, With<DebugText>>,
) {
    for mut text in &mut query {
        let keys = if pressed.0.is_empty() {
            "none".to_string()
        } else {
            pressed.0.join(", ")
        };
        text.0 = format!(
            "keys: {}\npos: {:.2?}\nvel: {:.2?}\nrot: {:.2?}\nrpms: {:.0?}\nwind: {:.2?}\nbatt: {:.1}% V={:.1} A={:.1}",
            keys,
            telemetry.position,
            telemetry.velocity,
            telemetry.rotation.to_euler(EulerRot::XYZ),
            telemetry.motor_rpms,
            wind.0,
            batt.soc * 100.0,
            batt.voltage,
            batt.current_a,
        );
    }
}

pub fn setup_ui(mut commands: Commands) {
    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        },
        Text::new("Controls: R reset | N net | D domain rand | G gizmos | Ctrl+F free cam (WASD, Space up, Shift down, Q/E yaw)"),
        TextFont { font_size: FontSize::Px(14.0), ..default() },
        TextColor(Color::WHITE),
    ));

    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(40.0),
            left: Val::Px(10.0),
            ..default()
        },
        Text::new("debug"),
        TextFont { font_size: FontSize::Px(14.0), ..default() },
        TextColor(Color::srgb(0.0, 1.0, 0.0)),
        DebugText,
    ));
}

pub fn ensure_stream_box(
    mut commands: Commands,
    buffer: Res<CameraFrameBuffer>,
    has_stream: Query<Entity, With<StreamBox>>,
) {
    if has_stream.single().is_ok() {
        return;
    }
    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            right: Val::Px(10.0),
            width: Val::Px(320.0),
            height: Val::Px(240.0),
            border: UiRect::all(Val::Px(2.0)),
            ..default()
        },
        ImageNode {
            image: buffer.image.clone(),
            ..default()
        },
        StreamBox,
    ));
    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(255.0),
            right: Val::Px(10.0),
            ..default()
        },
        Text::new("STREAM (interference: off)"),
        TextFont { font_size: FontSize::Px(12.0), ..default() },
        TextColor(Color::srgb(0.0, 1.0, 0.0)),
        StreamStatusText,
    ));
}

pub fn update_stream_interference(
    time: Res<Time>,
    net: Res<NetworkConfig>,
    mut interference: ResMut<StreamInterference>,
    mut stream_query: Query<(&mut Visibility, &mut BorderColor), With<StreamBox>>,
    mut status_text: Query<&mut Text, With<StreamStatusText>>,
) {
    let mut cut = false;
    let mut noise = false;
    if net.enabled {
        let mut rng = rand::thread_rng();
        if rng.r#gen::<f32>() < net.packet_loss {
            cut = true;
        }
        if rng.r#gen::<f32>() < net.corrupt_prob {
            noise = true;
        }
    }

    interference.cut_timer = if cut { 0.15 } else { (interference.cut_timer - time.delta_secs()).max(0.0) };
    interference.noise_timer = if noise { 0.1 } else { (interference.noise_timer - time.delta_secs()).max(0.0) };
    interference.visible = interference.cut_timer <= 0.0;

    for (mut visibility, mut border) in &mut stream_query {
        *visibility = if interference.visible { Visibility::Inherited } else { Visibility::Hidden };
        let border_color = if interference.noise_timer > 0.0 {
            Color::srgb(1.0, 0.0, 0.0)
        } else if interference.visible {
            Color::srgb(0.0, 1.0, 0.0)
        } else {
            Color::srgb(0.5, 0.5, 0.5)
        };
        border.top = border_color;
        border.right = border_color;
        border.bottom = border_color;
        border.left = border_color;
    }

    for mut text in &mut status_text {
        text.0 = format!(
            "STREAM (latency: {}ms | loss: {}% | {})",
            net.latency_ms,
            (net.packet_loss * 100.0) as i32,
            if interference.visible { "OK" } else { "CUT" }
        );
    }
}
