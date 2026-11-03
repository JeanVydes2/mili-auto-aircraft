//! Ground control: received instruments and local engineering are deliberately separate.
//!
//! Register `setup_font` before HUD/visuals startup, `setup_hud` after camera/stream
//! startup, then `update_hud` in Update. UiScale is global (0.9–1.0).
//! The existing default UI camera must composite AFTER every window camera (including
//! freecam). No camera, stream resource, or external asset is created here.
//! Tab/V/C/Ctrl+F remain owned by stream/camera; this module owns F1/F3/G/N.

use std::time::Duration;

use bevy::asset::AssetId;
use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use bevy::text::FontSize;
use bevy::ui::IsDefaultUiCamera;
use bevy::window::PrimaryWindow;

use crate::camera::{CameraMode, FreeCamActive};
use crate::config::{
    BatteryState, DroneConfig, EnvironmentConfig, FlightControlLimits, GizmosEnabled,
};
use crate::domain_randomization::ResetGeneration;
use crate::environment::CurrentWind;
use crate::motors::Drone;
use crate::sensors::Telemetry;
use crate::stream::{LinkState, ReceiverView, StreamDisplay, StreamStats};

const CYAN: Color = Color::srgb(0.25, 0.91, 0.97);
const AMBER: Color = Color::srgb(1.0, 0.73, 0.29);
const RED: Color = Color::srgb(1.0, 0.32, 0.35);
const INK: Color = Color::srgba(0.025, 0.045, 0.065, 0.91);
const WHITE: Color = Color::srgb(0.89, 0.95, 0.97);
const MUTED: Color = Color::srgb(0.59, 0.71, 0.76);
const EDGE: Color = Color::srgba(0.25, 0.72, 0.80, 0.45);

#[derive(Component)]
enum Element {
    Instruments,
    ViewBadge,
    Received,
    Compass,
    AttitudeLabel,
    Horizon,
    Local,
    MotorLabel(usize),
    MotorBar(usize),
    BatteryBar,
    Link,
    Feed,
    FeedImage,
    FeedTop,
    FeedBottom,
    FeedTitle,
    FeedCaption,
    SignalBar,
    Help,
    Footer,
}

#[derive(Resource)]
pub struct HudState {
    visible: bool,
    help: bool,
    home: Vec3,
    started_at: f64,
    generation: u64,
}

/// Local text clock; layout, input, image handles and attitude still update every frame.
pub struct HudClock {
    timer: Timer,
    first: bool,
}

#[derive(Component)]
pub struct AircraftSelectButton(pub usize);

impl Default for HudClock {
    fn default() -> Self {
        Self {
            timer: Timer::from_seconds(0.1, TimerMode::Repeating),
            first: true,
        }
    }
}

#[derive(SystemParam)]
pub struct HudSources<'w> {
    local: Res<'w, Telemetry>,
    battery: Res<'w, BatteryState>,
    drone: Res<'w, DroneConfig>,
    environment: Res<'w, EnvironmentConfig>,
    flight_limits: Res<'w, FlightControlLimits>,

    wind: Option<Res<'w, CurrentWind>>,
    camera: Res<'w, CameraMode>,
    free_camera: Res<'w, FreeCamActive>,
    display: Res<'w, StreamDisplay>,
    stream: Res<'w, StreamStats>,
    receiver: Res<'w, ReceiverView>,
    reset: Option<Res<'w, ResetGeneration>>,
}

// One query avoids overlapping mutable Node/Text queries and their B0001 conflicts.
#[derive(SystemParam)]
pub struct HudWidgets<'w, 's> {
    nodes: Query<
        'w,
        's,
        (
            &'static Element,
            &'static mut Node,
            Option<&'static mut Text>,
            Option<&'static mut TextColor>,
            Option<&'static mut BackgroundColor>,
            Option<&'static mut BorderColor>,
            Option<&'static mut ImageNode>,
            Option<&'static mut UiTransform>,
        ),
    >,
}

fn text(value: impl Into<String>, size: f32, color: Color) -> impl Bundle {
    (
        Text::new(value),
        TextFont {
            font_size: FontSize::Px(size),
            ..default()
        },
        TextColor(color),
    )
}

fn panel() -> impl Bundle {
    (
        Node {
            flex_direction: FlexDirection::Column,
            padding: UiRect::all(px(12)),
            row_gap: px(5),
            border: UiRect::all(px(1)),
            border_radius: BorderRadius::all(px(7)),
            flex_shrink: 0.0,
            ..default()
        },
        BackgroundColor(INK),
        BorderColor::all(EDGE),
    )
}

fn screen() -> Node {
    Node {
        position_type: PositionType::Absolute,
        width: percent(100),
        height: percent(100),
        ..default()
    }
}

/// Replaces Bevy's global default font, including Text3d; no runtime asset I/O.
/// Source and license: assets/fonts/DejaVuSansMono.ttf and LICENSE-DejaVu.txt.
pub fn setup_font(mut fonts: ResMut<Assets<Font>>) {
    let font = Font::from_bytes(include_bytes!("../assets/fonts/DejaVuSansMono.ttf").to_vec());
    fonts.insert(AssetId::default(), font).unwrap();
}

// Keep body text at least 12.6 logical pixels (14 × 0.9). Do not enlarge the
// established 1440×900 layout, or shrink text indefinitely on tiny windows.
fn responsive_scale(width: f32, height: f32) -> f32 {
    if !width.is_finite() || !height.is_finite() || width <= 0.0 || height <= 0.0 {
        return 1.0;
    }
    (width / 1280.0).min(height / 800.0).clamp(0.9, 1.0)
}

fn update_scale(window: &Query<&Window, With<PrimaryWindow>>, scale: &mut ResMut<UiScale>) {
    if let Ok(window) = window.single() {
        let next = responsive_scale(window.width(), window.height());
        if (scale.0 - next).abs() > 0.0001 {
            scale.0 = next;
        }
    }
}

/// Uses the existing UI camera; intentionally does not spawn a second camera.
pub fn setup_hud(
    mut commands: Commands,
    sources: HudSources,
    time: Res<Time>,
    ui_camera: Query<Entity, With<IsDefaultUiCamera>>,
    drone: Query<&Transform, With<Drone>>,
    window: Query<&Window, With<PrimaryWindow>>,
    mut scale: ResMut<UiScale>,
) {
    update_scale(&window, &mut scale);
    let Ok(camera) = ui_camera.single() else {
        warn!("HUD: exactly one IsDefaultUiCamera is required; UI was not created");
        return;
    };
    commands.insert_resource(HudState {
        visible: true,
        help: false,
        home: drone
            .iter()
            .next()
            .map_or(sources.local.position, |tf| tf.translation),
        started_at: time.elapsed_secs_f64(),
        generation: sources.reset.as_ref().map_or(0, |r| r.0),
    });

    // Video is a separate, lower layer: F3 must not remove the received picture.
    commands
        .spawn((screen(), UiTargetCamera(camera), GlobalZIndex(0)))
        .with_children(|root| {
            root.spawn((
                Element::Feed,
                feed_node(sources.receiver.0),
                BackgroundColor(Color::BLACK),
                BorderColor::all(AMBER),
            ))
            .with_children(|feed| {
                feed.spawn((
                    Element::FeedImage,
                    Node {
                        width: percent(100),
                        height: percent(100),
                        ..default()
                    },
                    ImageNode::new(sources.display.image.clone()),
                ));
                feed.spawn((
                    Element::FeedTop,
                    Node {
                        position_type: PositionType::Absolute,
                        justify_content: JustifyContent::Center,
                        top: px(0),
                        left: px(0),
                        right: px(0),
                        padding: UiRect::axes(px(9), px(5)),
                        ..default()
                    },
                    BackgroundColor(INK),
                    ZIndex(1),
                ))
                .with_children(|banner| {
                    banner.spawn((Element::FeedTitle, text("RX DELAYED · BUFFER", 14.0, AMBER)));
                });
                feed.spawn((
                    Element::FeedBottom,
                    Node {
                        position_type: PositionType::Absolute,
                        justify_content: JustifyContent::Center,
                        bottom: px(0),
                        left: px(0),
                        right: px(0),
                        padding: UiRect::axes(px(9), px(5)),
                        ..default()
                    },
                    BackgroundColor(INK),
                    ZIndex(1),
                ))
                .with_children(|banner| {
                    banner.spawn((
                        Element::FeedCaption,
                        text("WAITING FOR RECEIVED FRAME", 12.0, WHITE),
                    ));
                });
            });
        });

    commands
        .spawn((
            Element::Instruments,
            screen(),
            UiTargetCamera(camera),
            GlobalZIndex(10),
        ))
        .with_children(|root| {
            root.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    top: px(16),
                    left: px(20),
                    right: px(20),
                    min_height: px(56),
                    padding: UiRect::axes(px(16), px(10)),
                    justify_content: JustifyContent::SpaceBetween,
                    align_items: AlignItems::Center,
                    border: UiRect::bottom(px(2)),
                    ..default()
                },
                BackgroundColor(INK),
                BorderColor::all(CYAN),
            ))
            .with_children(|header| {
                header.spawn(text("VYDE / FLIGHT CONTROL", 21.0, CYAN));
                header
                    .spawn(Node {
                        align_items: AlignItems::Center,
                        column_gap: px(7),
                        ..default()
                    })
                    .with_children(|controls| {
                        controls.spawn((Element::ViewBadge, text("AIRFRAME", 12.0, MUTED)));
                        for (index, label) in ["QUAD", "HEX", "FIXED WING", "VTOL"].into_iter().enumerate() {
                            controls
                                .spawn((
                                    bevy_ui_widgets::Button,
                                    AircraftSelectButton(index),
                                    Node {
                                        padding: UiRect::axes(px(10), px(6)),
                                        border: UiRect::all(px(1)),
                                        border_radius: BorderRadius::all(px(4)),
                                        ..default()
                                    },
                                    BackgroundColor(Color::srgb(0.08, 0.16, 0.19)),
                                    BorderColor::all(EDGE),
                                ))
                                .with_children(|button| {
                                    button.spawn(text(label, 12.0, WHITE));
                                });
                        }
                    });
            });

            root.spawn(Node {
                position_type: PositionType::Absolute,
                top: px(90),
                bottom: px(92),
                left: px(20),
                width: px(310),
                flex_direction: FlexDirection::Column,
                row_gap: px(10),
                // Keep the control strip unobstructed if the window is too short.
                overflow: Overflow::clip(),
                ..default()
            })
            .with_children(|column| {
                column.spawn(panel()).with_children(|p| {
                    p.spawn(text("01 / RECEIVED · RX DELAYED", 15.0, AMBER));
                    p.spawn((Element::Received, text("WAITING FOR TELEMETRY…", 15.0, WHITE)));
                    p.spawn(text("HOME = start / reset · NORTH +Z / EAST +X", 11.0, MUTED));
                });
                column.spawn(panel()).with_children(|p| {
                    p.spawn(text("02 / ENGINEERING · LIVE LOCAL", 15.0, CYAN));
                    p.spawn((Element::Local, text("", 14.0, WHITE)));
                    p.spawn((
                        Node { height: px(5), width: percent(100), ..default() },
                        BackgroundColor(Color::srgb(0.10, 0.19, 0.23)),
                    ))
                    .with_children(|track| {
                        track.spawn((
                            Element::BatteryBar,
                            Node { height: percent(100), width: percent(100), ..default() },
                            BackgroundColor(CYAN),
                        ));
                    });
                    p.spawn(text("ACTUATOR OUTPUT · THRUST AUTO", 11.0, MUTED));
                    for i in 0..4 {
                        p.spawn(Node {
                            align_items: AlignItems::Center,
                            column_gap: px(7),
                            ..default()
                        })
                        .with_children(|row| {
                            row.spawn((
                                Element::MotorLabel(i),
                                text(format!("M{}       0 RPM", i + 1), 12.0, MUTED),
                                Node { width: px(123), ..default() },
                            ));
                            row.spawn((
                                Node { height: px(6), flex_grow: 1.0, ..default() },
                                BackgroundColor(Color::srgb(0.10, 0.19, 0.23)),
                            ))
                            .with_children(|track| {
                                track.spawn((
                                    Element::MotorBar(i),
                                    Node { height: percent(100), width: percent(0), ..default() },
                                    BackgroundColor(CYAN),
                                ));
                            });
                        });
                    }
                });
            });

            root.spawn((
                BackgroundColor(INK),
                BorderColor::all(EDGE),
                Node {
                    position_type: PositionType::Absolute,
                    top: px(90),
                    right: px(20),
                    width: px(400),
                    flex_direction: FlexDirection::Column,
                    padding: UiRect::all(px(12)),
                    row_gap: px(5),
                    border: UiRect::all(px(1)),
                    border_radius: BorderRadius::all(px(7)),
                    ..default()
                },
            ))
            .with_children(|p| {
                p.spawn(text("03 / VIDEO LINK · RECEIVER", 15.0, AMBER));
                p.spawn((Element::Link, text("BUFFER / CONNECTING…", 14.0, WHITE)));
                p.spawn((
                    Node { height: px(5), width: percent(100), ..default() },
                    BackgroundColor(Color::srgb(0.10, 0.19, 0.23)),
                ))
                .with_children(|track| {
                    track.spawn((
                        Element::SignalBar,
                        Node { height: percent(100), width: percent(0), ..default() },
                        BackgroundColor(AMBER),
                    ));
                });
            });

            // The moving horizon is clipped independently of the fixed reticle.
            root.spawn(Node {
                position_type: PositionType::Absolute,
                left: percent(50),
                top: percent(50),
                margin: UiRect { left: px(-110), top: px(-60), ..default() },
                width: px(220),
                height: px(120),
                overflow: Overflow::clip(),
                ..default()
            })
            .with_children(|instrument| {
                instrument.spawn((
                    Element::Horizon,
                    Node {
                        position_type: PositionType::Absolute,
                        left: px(10),
                        top: px(59),
                        width: px(200),
                        height: px(2),
                        ..default()
                    },
                    UiTransform::IDENTITY,
                    BackgroundColor(CYAN),
                ))
                .with_children(|line| {
                    for offset in [-30.0, 30.0] {
                        line.spawn((
                            Node {
                                position_type: PositionType::Absolute,
                                left: px(75),
                                top: px(offset),
                                width: px(50),
                                height: px(1),
                                ..default()
                            },
                            BackgroundColor(EDGE),
                        ));
                    }
                });
                instrument.spawn((
                    Node {
                        position_type: PositionType::Absolute,
                        left: px(66),
                        top: px(46),
                        padding: UiRect::axes(px(8), px(1)),
                        ..default()
                    },
                    BackgroundColor(Color::srgba(0.02, 0.04, 0.06, 0.7)),
                    ZIndex(1),
                ))
                .with_children(|reticle| { reticle.spawn(text("— + —", 20.0, AMBER)); });
            });
            root.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    top: percent(50),
                    left: percent(50),
                    margin: UiRect { left: px(-105), top: px(-110), ..default() },
                    width: px(210),
                    padding: UiRect::all(px(6)),
                    flex_direction: FlexDirection::Column,
                    align_items: AlignItems::Center,
                    ..default()
                },
                BackgroundColor(INK),
            ))
            .with_children(|p| {
                p.spawn((Element::Compass, text("---° / RX WAIT", 18.0, WHITE)));
                p.spawn((Element::AttitudeLabel, text("RX ATTITUDE · WAITING", 11.0, AMBER)));
            });

            root.spawn((
                Element::Help,
                Node {
                    display: Display::None,
                    position_type: PositionType::Absolute,
                    left: percent(50),
                    top: percent(50),
                    margin: UiRect { left: px(-230), top: px(-155), ..default() },
                    width: px(460),
                    padding: UiRect::all(px(22)),
                    border: UiRect::all(px(1)),
                    flex_direction: FlexDirection::Column,
                    row_gap: px(10),
                    ..default()
                },
                BackgroundColor(Color::srgba(0.02, 0.04, 0.06, 0.98)),
                BorderColor::all(CYAN),
                ZIndex(50),
            ))
            .with_children(|p| {
                p.spawn(text("HELP / GROUND STATION", 20.0, CYAN));
                p.spawn(text(
                    "FLIGHT CONTROL: run `cargo run -p controller`\nIn the controller: :arm, WASD/Space/X/QE manual,\n:goto LAT LON ALT, :hold, :takeoff ALT, :land, :rtl\n:speed M/S · :accel M/S2 · :tilt DEG\nCtrl+F   free camera · C   chase camera\nTAB   fullscreen receiver\nV   cycle video-link profile sent to the controller\nG   gizmos · R   reset\nF1   help · F3   instruments",
                    16.0, WHITE,
                ));
                p.spawn(text(
                    "LIVE LOCAL = ideal simulation state.\nRX DELAYED = received video and telemetry;\ntheir delays may differ.\nYaw changes heading; pitch tilts forward/back; roll tilts left/right.\nVelocity mode manages thrust and altitude automatically.\nV changes loss, interference, and latency on the encoded stream.\nThe attitude horizon is independent of the free camera.",
                    14.0, AMBER,
                ));
            });
            root.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    bottom: px(16),
                    left: px(20),
                    right: px(20),
                    padding: UiRect::axes(px(14), px(9)),
                    border: UiRect::top(px(1)),
                    ..default()
                },
                BackgroundColor(INK),
                BorderColor::all(EDGE),
            ))
            .with_children(|p| {
                p.spawn((Element::Footer, text("", 13.0, WHITE)));
            });
        });
}

fn feed_node(fullscreen: bool) -> Node {
    Node {
        position_type: PositionType::Absolute,
        right: px(if fullscreen { 0 } else { 20 }),
        bottom: px(if fullscreen { 0 } else { 94 }),
        width: if fullscreen { percent(100) } else { px(400) },
        height: if fullscreen { percent(100) } else { px(225) },
        border: UiRect::all(px(2)),
        overflow: Overflow::clip(),
        ..default()
    }
}

/// Text/bars: 10 Hz. Receiver geometry, source labels and RX attitude: every frame.
/// Reads ReceiverView, never toggles it: stream owns Tab and V.
pub fn update_hud(
    time: Res<Time>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut gizmos: Option<ResMut<GizmosEnabled>>,
    state: Option<ResMut<HudState>>,
    sources: HudSources,
    mut widgets: HudWidgets,
    mut clock: Local<HudClock>,
    window: Query<&Window, With<PrimaryWindow>>,
    mut scale: ResMut<UiScale>,
) {
    update_scale(&window, &mut scale);
    let Some(mut state) = state else {
        return;
    };
    let mut refresh = clock.first;
    clock.first = false;
    clock.timer.tick(time.delta());
    refresh |= clock.timer.just_finished()
        || sources.receiver.is_changed()
        || sources.camera.is_changed()
        || sources.free_camera.is_changed();
    if keyboard.just_pressed(KeyCode::F1) {
        state.help = !state.help;
        state.visible = true;
        refresh = true;
    }
    if keyboard.just_pressed(KeyCode::F3) {
        state.visible = !state.visible;
        refresh = true;
    }
    if keyboard.just_pressed(KeyCode::KeyG) {
        if let Some(ref mut enabled) = gizmos {
            enabled.0 = !enabled.0;
        }
        refresh = true;
    }
    if let Some(generation) = &sources.reset {
        if state.generation != generation.0 {
            state.generation = generation.0;
            state.home = sources.local.position;
            state.started_at = time.elapsed_secs_f64();
            refresh = true;
        }
    }
    let rx = &sources.local;
    let stream = &sources.stream;
    let fullscreen = sources.receiver.0;
    let (status, status_color) = link_status(stream.state);
    let (heading, pitch, roll) = attitude(rx.rotation);
    let seconds = (time.elapsed_secs_f64() - state.started_at).max(0.0);

    for (element, mut node, label, color, background, border, image, transform) in
        &mut widgets.nodes
    {
        match element {
            Element::Instruments => {
                let display = if state.visible {
                    Display::Flex
                } else {
                    Display::None
                };
                if node.display != display {
                    node.display = display;
                }
            }
            Element::Help => {
                let display = if state.help {
                    Display::Flex
                } else {
                    Display::None
                };
                if node.display != display {
                    node.display = display;
                }
            }
            Element::Feed => {
                let layout = feed_node(fullscreen);
                if node.width != layout.width || node.height != layout.height {
                    *node = layout;
                }
                if let Some(mut border) = border {
                    let next = BorderColor::all(status_color);
                    if *border != next {
                        *border = next;
                    }
                }
            }
            Element::FeedImage => {
                if let Some(mut image) = image {
                    if image.image != sources.display.image {
                        image.image = sources.display.image.clone();
                    }
                }
            }
            Element::FeedTop => {
                let top = px(if fullscreen && state.visible { 80 } else { 0 });
                if node.top != top {
                    node.top = top;
                }
            }
            Element::FeedBottom => {
                let bottom = px(if fullscreen && state.visible { 80 } else { 0 });
                if node.bottom != bottom {
                    node.bottom = bottom;
                }
            }
            Element::Horizon => {
                let display = Display::Flex;
                if node.display != display {
                    node.display = display;
                }
                if let Some(mut tf) = transform {
                    let tilt_limit = sources.flight_limits.max_tilt_deg;
                    tf.translation = Val2::px(0.0, pitch.clamp(-tilt_limit, tilt_limit) * 1.5);
                    tf.rotation = Rot2::degrees(-roll.clamp(-tilt_limit, tilt_limit));
                }
            }
            _ if !refresh => {}
            Element::ViewBadge => {
                let mode = if sources.free_camera.0 {
                    "FREE"
                } else {
                    match *sources.camera {
                        CameraMode::Fpv => "FPV",
                        CameraMode::Chase => "CHASE",
                        CameraMode::Free => "FREE",
                    }
                };
                let value = if fullscreen {
                    "RX DELAYED VIEW · RECEIVER".to_owned()
                } else {
                    format!("LIVE LOCAL VIEW · {mode} / RX RECEIVER IN MONITOR")
                };
                set_text(label, value);
                set_color(color, if fullscreen { AMBER } else { CYAN });
            }
            Element::FeedTitle => {
                // Keep this source/status marker even when F3 hides all instruments.
                set_text(
                    label,
                    format!("RX DELAYED · {status} · {:?}", stream.profile),
                );
                set_color(color, status_color);
            }
            Element::Received => {
                set_text(label, flight_text(rx, state.home, seconds));
                set_color(color, WHITE);
            }
            Element::Compass => {
                set_text(
                    label,
                    format!("{:03.0}° / {}", rounded_heading(heading), cardinal(heading)),
                );
            }
            Element::AttitudeLabel => {
                set_text(
                    label,
                    format!(
                        "SIM ATTITUDE · PITCH / ROLL LIMIT ±{:.0}°",
                        sources.flight_limits.max_tilt_deg
                    ),
                );
            }
            Element::Local => {
                let t = &sources.local;
                let b = &sources.battery;
                let e = &sources.environment;
                let wind = sources.wind.as_ref().map_or(e.wind_constant, |w| w.0);
                let wind_source = if sources.wind.is_some() {
                    "LIVE"
                } else {
                    "BASE"
                };
                set_text(
                    label,
                    format!(
                        "ALT {:5.1} m · H-SPEED {:4.1} m/s\nV/S {:+5.1} m/s · BAT {:3.0}%\n{:5.2} V · {:5.1} A · {:6.1} W\nTEMP {:4.1} °C · WIND {wind_source} {:4.1} m/s\nXYZ {:+.1}/{:+.1}/{:+.1}\nLIMITS  VMAX {:.1} m/s · AMAX {:.1} m/s²\nTILT ±{:.0}° · g {:.2} m/s² · ρ {:.3} kg/m³\nSIM TIME {}",
                        t.position.y,
                        horizontal_speed(t.velocity),
                        t.velocity.y,
                        unit_percent(b.soc),
                        b.voltage,
                        b.current_a,
                        b.voltage * b.current_a,
                        b.temperature_c,
                        wind.length(),
                        wind.x,
                        wind.y,
                        wind.z,
                        sources.flight_limits.max_speed_m_s,
                        sources.flight_limits.max_acceleration_m_s2,
                        sources.flight_limits.max_tilt_deg,
                        e.gravity,
                        e.air_density,
                        elapsed_text(seconds),
                    ),
                );
            }
            Element::MotorLabel(i) => {
                set_text(
                    label,
                    format!("M{} {:6.0} RPM", i + 1, sources.local.motor_rpms[*i]),
                );
            }
            Element::MotorBar(i) => {
                let ratio = safe_ratio(
                    sources.local.motor_rpms[*i],
                    sources.drone.motors[*i].max_rpm,
                );
                node.width = percent(unit_percent(ratio));
                if let Some(mut bg) = background {
                    bg.0 = if ratio >= 0.9 { AMBER } else { CYAN };
                }
            }
            Element::BatteryBar => {
                node.width = percent(unit_percent(sources.battery.soc));
                if let Some(mut bg) = background {
                    bg.0 = if sources.battery.soc <= 0.15 {
                        RED
                    } else if sources.battery.soc <= 0.3 {
                        AMBER
                    } else {
                        CYAN
                    };
                }
            }
            Element::SignalBar => {
                node.width = percent(unit_percent(stream.signal_quality));
                if let Some(mut bg) = background {
                    bg.0 = status_color;
                }
            }
            Element::Link => {
                set_text(
                    label,
                    format!(
                        "{status} · {:?} · {:.0} m\nSIGNAL {:3.0}% · RSSI {:.0} dBm\nRX {:4.1} FPS · {:.2} Mbps\nFRAME AGE {:.0} ms · LATENCY {:.0} ms\nJITTER {:.0} ms · BUFFER {:.0} ms\nQUEUE {} frames · RX {}\nLOSS {} · DROP {} · CORRUPT {}",
                        stream.profile,
                        stream.distance_m,
                        unit_percent(stream.signal_quality),
                        stream.rssi_dbm,
                        stream.rx_fps,
                        stream.bitrate_kbps / 1000.0,
                        stream.frame_age_ms,
                        stream.latency_ms,
                        stream.jitter_ms,
                        stream.buffer_ms,
                        stream.queue_frames,
                        stream.received_frames,
                        stream.lost_frames,
                        stream.dropped_frames,
                        stream.corrupted_frames,
                    ),
                );
                set_color(color, status_color);
            }
            Element::FeedCaption => {
                set_text(
                    label,
                    format!(
                        "{:.1} FPS / AGE {:.0} ms / {}×{} · TAB {}",
                        stream.rx_fps,
                        stream.frame_age_ms,
                        sources.display.width,
                        sources.display.height,
                        if fullscreen { "MONITOR" } else { "EXPAND" },
                    ),
                );
            }
            Element::Footer => {
                let gizmos = gizmos.as_ref().is_some_and(|g| g.0);
                set_text(
                    label,
                    format!(
                        "MAVLink CONTROL: run `controller` · R reset · Ctrl+F free camera · C chase · TAB monitor · V video-link profile · G gizmos {} · F1 help · F3 HUD",
                        if gizmos { "ON" } else { "OFF" },
                    ),
                );
            }
        }
    }
}

fn set_text(label: Option<Mut<Text>>, value: String) {
    if let Some(mut label) = label {
        if label.0 != value {
            label.0 = value;
        }
    }
}

fn set_color(color: Option<Mut<TextColor>>, value: Color) {
    if let Some(mut color) = color {
        if color.0 != value {
            color.0 = value;
        }
    }
}

fn link_status(state: LinkState) -> (&'static str, Color) {
    match state {
        LinkState::Connecting => ("BUFFER / CONNECTING", AMBER),
        LinkState::Buffering => ("BUFFER", AMBER),
        LinkState::Live => ("LIVE", CYAN),
        LinkState::Degraded => ("LIVE / DEGRADED", AMBER),
        LinkState::Frozen => ("LOST / FROZEN", RED),
        LinkState::Lost => ("LOST", RED),
    }
}

// The world and MAVLink frame use north +Z, east +X, and altitude +Y.
fn attitude(rotation: Quat) -> (f32, f32, f32) {
    if !rotation.is_finite() || rotation.length_squared() < 1e-6 {
        return (0.0, 0.0, 0.0);
    }
    let (yaw, pitch, roll) = rotation.normalize().to_euler(EulerRot::YXZ);
    (
        yaw.to_degrees().rem_euclid(360.0),
        -pitch.to_degrees(),
        roll.to_degrees(),
    )
}

fn horizontal_speed(velocity: Vec3) -> f32 {
    Vec2::new(velocity.x, velocity.z).length()
}

fn flight_text(t: &Telemetry, home: Vec3, seconds: f64) -> String {
    let (heading, pitch, roll) = attitude(t.rotation);
    format!(
        "ALT {:7.1} m\nH-SPEED {:5.1} m/s   V/S {:+5.1} m/s\nHEADING {:03.0}° {}\nPITCH {:+5.1}° / ROLL {:+5.1}°\nX {:+.1}  Y {:+.1}  Z {:+.1}\nHOME {:7.1} m · GPS {}\nFLIGHT / SIM {}",
        t.position.y,
        horizontal_speed(t.velocity),
        t.velocity.y,
        rounded_heading(heading),
        cardinal(heading),
        pitch,
        roll,
        t.position.x,
        t.position.y,
        t.position.z,
        t.position.distance(home),
        if t.gps_valid { "OK" } else { "NO" },
        elapsed_text(seconds),
    )
}

// Pure formatting/math helpers also run without a renderer or stream resources.
fn unit_percent(value: f32) -> f32 {
    if value.is_finite() {
        value.clamp(0.0, 1.0) * 100.0
    } else {
        0.0
    }
}

fn safe_ratio(value: f32, maximum: f32) -> f32 {
    if value.is_finite() && maximum.is_finite() && maximum > 0.0 {
        (value / maximum).clamp(0.0, 1.0)
    } else {
        0.0
    }
}

fn rounded_heading(degrees: f32) -> f32 {
    if degrees.is_finite() {
        degrees.round().rem_euclid(360.0)
    } else {
        0.0
    }
}

fn cardinal(degrees: f32) -> &'static str {
    const POINTS: [&str; 8] = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
    let heading = if degrees.is_finite() {
        degrees.rem_euclid(360.0)
    } else {
        0.0
    };
    POINTS[((heading + 22.5) / 45.0).floor() as usize % 8]
}

fn elapsed_text(seconds: f64) -> String {
    let seconds = if seconds.is_finite() {
        seconds.max(0.0)
    } else {
        0.0
    };
    let total = Duration::from_secs_f64(seconds.min(u32::MAX as f64)).as_secs();
    format!(
        "{:02}:{:02}:{:02}",
        total / 3600,
        total / 60 % 60,
        total % 60
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn embedded_font_replaces_the_global_default() {
        let mut app = App::new();
        app.insert_resource(Assets::<Font>::default());
        app.add_systems(Startup, setup_font);
        app.update();
        let fonts = app.world().resource::<Assets<Font>>();
        let font = fonts
            .get(AssetId::default())
            .expect("default font installed");
        assert_eq!(
            font.data.as_ref(),
            include_bytes!("../assets/fonts/DejaVuSansMono.ttf").as_slice(),
        );
    }

    #[test]
    fn scaling_fits_small_windows_without_making_body_text_tiny() {
        assert_eq!(responsive_scale(1440.0, 900.0), 1.0);
        assert_eq!(responsive_scale(1024.0, 768.0), 0.9);
        assert_eq!(responsive_scale(1280.0, 800.0), 1.0);
        assert_eq!(responsive_scale(3840.0, 2160.0), 1.0);
        assert_eq!(responsive_scale(640.0, 480.0), 0.9);
        assert_eq!(responsive_scale(0.0, 0.0), 1.0);
        assert_eq!(responsive_scale(f32::NAN, 768.0), 1.0);
    }

    #[test]
    fn scale_changes_only_when_a_resize_changes_the_target() {
        #[derive(Resource, Default)]
        struct ScaleChanged(bool);

        fn adapt(window: Query<&Window, With<PrimaryWindow>>, mut scale: ResMut<UiScale>) {
            update_scale(&window, &mut scale);
        }
        fn observe(scale: Res<UiScale>, mut changed: ResMut<ScaleChanged>) {
            changed.0 = scale.is_changed();
        }

        let mut app = App::new();
        app.insert_resource(UiScale(1.0));
        app.init_resource::<ScaleChanged>();
        let window = app
            .world_mut()
            .spawn((Window::default(), PrimaryWindow))
            .id();
        app.add_systems(Update, (adapt, observe).chain());
        app.update();
        app.update();
        assert!(!app.world().resource::<ScaleChanged>().0);
        app.world_mut()
            .get_mut::<Window>(window)
            .unwrap()
            .resolution
            .set(1440.0, 900.0);
        app.update();
        assert!(app.world().resource::<ScaleChanged>().0);
        assert_eq!(app.world().resource::<UiScale>().0, 1.0);
        app.update();
        assert!(!app.world().resource::<ScaleChanged>().0);
    }

    #[test]
    fn percentages_and_bars_are_bounded() {
        assert_eq!(unit_percent(-0.1), 0.0);
        assert_eq!(unit_percent(0.42), 42.0);
        assert_eq!(unit_percent(2.0), 100.0);
        assert_eq!(unit_percent(f32::NAN), 0.0);
        assert_eq!(safe_ratio(4000.0, 8000.0), 0.5);
        assert_eq!(safe_ratio(100.0, 0.0), 0.0);
        assert_eq!(safe_ratio(f32::INFINITY, 8000.0), 0.0);
    }

    #[test]
    fn compass_wraps_and_uses_english_cardinals() {
        assert_eq!(cardinal(0.0), "N");
        assert_eq!(cardinal(90.0), "E");
        assert_eq!(cardinal(180.0), "S");
        assert_eq!(cardinal(225.0), "SW");
        assert_eq!(cardinal(-90.0), "W");
        assert_eq!(cardinal(359.9), "N");
        assert_eq!(rounded_heading(359.9), 0.0);
        assert_eq!(rounded_heading(-1.0), 359.0);
    }

    #[test]
    fn positive_yaw_points_north_forward_axis_east() {
        let (heading, _, _) = attitude(Quat::from_rotation_y(std::f32::consts::FRAC_PI_2));
        assert_eq!(rounded_heading(heading), 90.0);
        assert_eq!(cardinal(heading), "E");
    }

    #[test]
    fn clock_handles_hours_and_invalid_values() {
        assert_eq!(elapsed_text(0.0), "00:00:00");
        assert_eq!(elapsed_text(3661.9), "01:01:01");
        assert_eq!(elapsed_text(-2.0), "00:00:00");
        assert_eq!(elapsed_text(f64::NAN), "00:00:00");
    }
}
