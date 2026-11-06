use bevy::light::GlobalAmbientLight;
use bevy::prelude::*;
use bevy::text::FontSize;
use bevy::pbr::{DistanceFog, FogFalloff};
use bevy::ui::IsDefaultUiCamera;

use crate::config::EnvironmentConfig;
use crate::environment::{WindPreset, WindSettings};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum FogPreset {
    Off,
    #[default]
    Light,
    Medium,
    Dense,
}

impl FogPreset {
    fn label(self) -> &'static str {
        match self {
            Self::Off => "OFF",
            Self::Light => "LIGHT",
            Self::Medium => "MEDIUM",
            Self::Dense => "DENSE",
        }
    }

    fn fog(self) -> Option<DistanceFog> {
        let (color, start, end) = match self {
            Self::Off => return None,
            Self::Light => (Color::srgb(0.42, 0.52, 0.58), 18.0, 85.0),
            Self::Medium => (Color::srgb(0.36, 0.47, 0.53), 9.0, 48.0),
            Self::Dense => (Color::srgb(0.30, 0.39, 0.44), 3.0, 22.0),
        };
        Some(DistanceFog {
            color,
            falloff: FogFalloff::Linear { start, end },
            ..default()
        })
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TimePreset {
    #[default]
    Day,
    Afternoon,
    Dawn,
    Night,
}

impl TimePreset {
    fn label(self) -> &'static str {
        match self {
            Self::Day => "DAY",
            Self::Afternoon => "AFTERNOON",
            Self::Dawn => "DAWN",
            Self::Night => "NIGHT",
        }
    }

    fn lighting(self) -> (Color, f32, Color, f32) {
        match self {
            Self::Day => (Color::srgb(0.78, 0.87, 1.0), 100.0, Color::srgb(0.58, 0.72, 0.92), 12000.0),
            Self::Afternoon => (Color::srgb(1.0, 0.65, 0.36), 75.0, Color::srgb(0.92, 0.44, 0.18), 8500.0),
            Self::Dawn => (Color::srgb(0.96, 0.48, 0.38), 48.0, Color::srgb(0.88, 0.30, 0.22), 5600.0),
            Self::Night => (Color::srgb(0.08, 0.13, 0.24), 18.0, Color::srgb(0.06, 0.12, 0.28), 900.0),
        }
    }
}

#[derive(Resource, Clone, Copy, Debug)]
pub struct VisualSettingsState {
    pub menu_open: bool,
    pub fog: FogPreset,
    pub time: TimePreset,
    pub wind: WindPreset,
    pub wind_direction: usize,
}

impl Default for VisualSettingsState {
    fn default() -> Self {
        Self {
            menu_open: false,
            fog: FogPreset::Light,
            time: TimePreset::Day,
            wind: WindPreset::Light,
            wind_direction: 0,
        }
    }
}

#[derive(Component)]
pub(crate) struct SettingsPanel;

#[derive(Component)]
pub(crate) struct SettingsText;

const DIRECTIONS: [(&str, Vec3); 8] = [
    ("N", Vec3::Z),
    ("NE", Vec3::new(1.0, 0.0, 1.0)),
    ("E", Vec3::X),
    ("SE", Vec3::new(1.0, 0.0, -1.0)),
    ("S", Vec3::NEG_Z),
    ("SW", Vec3::new(-1.0, 0.0, -1.0)),
    ("W", Vec3::NEG_X),
    ("NW", Vec3::new(-1.0, 0.0, 1.0)),
];

pub fn setup_visual_settings(mut commands: Commands, camera: Query<Entity, With<IsDefaultUiCamera>>) {
    let Ok(camera) = camera.single() else {
        return;
    };
    commands.spawn((
        SettingsPanel,
        UiTargetCamera(camera),
        Node {
            display: Display::None,
            position_type: PositionType::Absolute,
            left: percent(50),
            top: percent(50),
            width: px(560),
            margin: UiRect { left: px(-280), top: px(-220), ..default() },
            padding: UiRect::all(px(22)),
            flex_direction: FlexDirection::Column,
            row_gap: px(10),
            border: UiRect::all(px(2)),
            ..default()
        },
        BackgroundColor(Color::srgba(0.005, 0.012, 0.018, 0.98)),
        BorderColor::all(Color::srgb(0.25, 0.92, 1.0)),
        ZIndex(200),
    ))
    .with_children(|panel| {
        panel.spawn((
            SettingsText,
            Text::new("VISUAL CONFIGURATION // ESC TO CLOSE"),
            TextFont { font_size: FontSize::Px(20.0), ..default() },
            TextColor(Color::srgb(0.25, 0.92, 1.0)),
        ));
        panel.spawn((
            SettingsText,
            Text::new(""),
            TextFont { font_size: FontSize::Px(16.0), ..default() },
            TextColor(Color::srgb(0.86, 0.94, 0.96)),
        ));
        panel.spawn((
            Text::new("F fog   T time   W wind   ←/→ direction   R reset presets"),
            TextFont { font_size: FontSize::Px(13.0), ..default() },
            TextColor(Color::srgb(1.0, 0.67, 0.18)),
        ));
    });
}

pub fn visual_settings_input(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut state: ResMut<VisualSettingsState>,
    mut env: ResMut<EnvironmentConfig>,
    mut wind: ResMut<WindSettings>,
) {
    if keyboard.just_pressed(KeyCode::Escape) {
        state.menu_open = !state.menu_open;
    }
    if !state.menu_open {
        return;
    }
    if keyboard.just_pressed(KeyCode::KeyF) {
        state.fog = match state.fog {
            FogPreset::Off => FogPreset::Light,
            FogPreset::Light => FogPreset::Medium,
            FogPreset::Medium => FogPreset::Dense,
            FogPreset::Dense => FogPreset::Off,
        };
    }
    if keyboard.just_pressed(KeyCode::KeyT) {
        state.time = match state.time {
            TimePreset::Day => TimePreset::Afternoon,
            TimePreset::Afternoon => TimePreset::Dawn,
            TimePreset::Dawn => TimePreset::Night,
            TimePreset::Night => TimePreset::Day,
        };
    }
    if keyboard.just_pressed(KeyCode::KeyW) {
        state.wind = match state.wind {
            WindPreset::Calm => WindPreset::Light,
            WindPreset::Light => WindPreset::Moderate,
            WindPreset::Moderate => WindPreset::Strong,
            WindPreset::Strong => WindPreset::Tornado,
            WindPreset::Tornado => WindPreset::Calm,
        };
    }
    if keyboard.just_pressed(KeyCode::ArrowRight) {
        state.wind_direction = (state.wind_direction + 1) % DIRECTIONS.len();
    }
    if keyboard.just_pressed(KeyCode::ArrowLeft) {
        state.wind_direction = (state.wind_direction + DIRECTIONS.len() - 1) % DIRECTIONS.len();
    }
    if keyboard.just_pressed(KeyCode::KeyR) {
        *state = default();
    }

    let (_, direction) = DIRECTIONS[state.wind_direction];
    wind.0 = state.wind;
    env.wind_constant = direction * state.wind.base_speed();
    env.wind_gust_direction = direction;
    env.wind_gust_intensity = state.wind.gust_speed();
    env.turbulence_intensity = state.wind.turbulence();
}

pub fn apply_visual_settings(
    state: Res<VisualSettingsState>,
    mut commands: Commands,
    mut cameras: Query<(Entity, Option<&mut DistanceFog>), With<Camera3d>>,
    mut ambient: ResMut<GlobalAmbientLight>,
    mut lights: Query<&mut DirectionalLight>,
    mut clear: ResMut<ClearColor>,
    mut panel: Query<&mut Node, With<SettingsPanel>>,
    mut text: Query<&mut Text, With<SettingsText>>,
) {
    let (ambient_color, ambient_brightness, sun_color, sun_lux) = state.time.lighting();
    ambient.color = ambient_color;
    ambient.brightness = ambient_brightness;
    clear.0 = ambient_color;
    for mut light in &mut lights {
        light.color = sun_color;
        light.illuminance = sun_lux;
    }
    for (entity, fog) in &mut cameras {
        match (state.fog.fog(), fog) {
            (Some(next), Some(mut current)) => *current = next,
            (Some(next), None) => {
                commands.entity(entity).insert(next);
            }
            (None, Some(_)) => {
                commands.entity(entity).remove::<DistanceFog>();
            }
            (None, None) => {}
        }
    }
    if let Ok(mut node) = panel.single_mut() {
        node.display = if state.menu_open { Display::Flex } else { Display::None };
    }
    let direction = DIRECTIONS[state.wind_direction].0;
    if let Some(mut value) = text.iter_mut().nth(1) {
        *value = Text::new(format!(
            "FOG       {:<9} [F]\nLIGHTING  {:<9} [T]\nWIND      {:<9} {:>2}  [W]\nVECTOR    {:<2}  ◀ / ▶\n\nSTATUS    LIVE / APPLIED",
            state.fog.label(),
            state.time.label(),
            state.wind.label(),
            format!("{:.1} m/s", state.wind.base_speed()),
            direction,
        ));
    }
}
