use crossterm::cursor;
use crossterm::event::{
    self, Event, KeyCode, KeyEvent, KeyEventKind, KeyModifiers, KeyboardEnhancementFlags,
    PopKeyboardEnhancementFlags, PushKeyboardEnhancementFlags,
};
use crossterm::{execute, terminal};
use mavlink::dialects::common::*;
use mavlink::{MavConnection, MavHeader, connect};
use std::collections::HashMap;
use std::io::{self, Write};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

const SYSTEM_ID: u8 = 255;
const COMPONENT_ID: u8 = 190;
const TARGET_SYSTEM: u8 = 1;
const TARGET_COMPONENT: u8 = 1;
const MAVLINK_DEFAULT: &str = "udpout:127.0.0.1:14550";
const VIDEO_DEFAULT: &str = "udp://0.0.0.0:5600";
const KEY_REPEAT_TIMEOUT: Duration = Duration::from_millis(750);

#[derive(Clone, Copy, Debug, Default)]
enum ControlMode {
    #[default]
    Hold,
    Manual,
    Guided,
}

impl ControlMode {
    fn label(self) -> &'static str {
        match self {
            Self::Hold => "HOLD",
            Self::Manual => "MANUAL",
            Self::Guided => "GUIDED",
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct ManualInput {
    north: f32,
    east: f32,
    down: f32,
    yaw_rate: f32,
}

#[derive(Clone, Copy, Debug)]
struct FlightTuning {
    max_speed_m_s: f32,
    max_acceleration_m_s2: f32,
    max_tilt_deg: f32,
}

impl Default for FlightTuning {
    fn default() -> Self {
        Self {
            max_speed_m_s: 3.0,
            max_acceleration_m_s2: 8.0,
            max_tilt_deg: 26.0,
        }
    }
}

impl ManualInput {
    fn is_active(self) -> bool {
        self.north != 0.0 || self.east != 0.0 || self.down != 0.0 || self.yaw_rate != 0.0
    }
}

#[derive(Default)]
struct TelemetrySnapshot {
    vehicle: Option<String>,
    armed: bool,
    attitude_deg: Option<[f32; 3]>,
    position: Option<[f64; 7]>,
    battery: Option<[f32; 3]>,
    imu: Option<[f32; 6]>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let options = Options::parse(std::env::args().skip(1))?;
    if options.help {
        print_help();
        return Ok(());
    }

    let mut connection = connect::<MavMessage>(&options.mavlink)?;
    let terminal_mode = TerminalMode::enable()?;

    let mut video_error = None;
    let mut video_player = if options.no_video {
        None
    } else {
        match start_video_player(&options.video) {
            Ok(player) => Some(player),
            Err(error) => {
                video_error = Some(format!("Video receiver unavailable (ffplay): {error}"));
                None
            }
        }
    };

    let mut sequence = 0_u8;
    let mut last_heartbeat = Instant::now() - Duration::from_secs(2);
    let mut last_manual_setpoint = Instant::now();
    let mut manual_input = None;
    let mut last_dashboard_render = Instant::now() - Duration::from_secs(1);
    let mut telemetry = TelemetrySnapshot::default();
    let mut command_buffer = None::<String>;
    let mut pressed_keys = HashMap::new();
    let mut keyboard_manual = false;
    let mut flight_tuning = FlightTuning::default();
    let mut control_mode = ControlMode::Hold;
    let mut status = video_error
        .unwrap_or_else(|| format!("MAVLink: {} · Video: {}", options.mavlink, options.video));

    'app: loop {
        if last_heartbeat.elapsed() >= Duration::from_secs(1) {
            send_heartbeat(&mut connection, &mut sequence)?;
            last_heartbeat = Instant::now();
        }

        match connection.try_recv() {
            Ok((_, message)) => {
                update_telemetry(message, &mut telemetry, &mut flight_tuning, &mut status)
            }
            Err(mavlink::error::MessageReadError::Io(error))
                if error.kind() == io::ErrorKind::WouldBlock => {}
            Err(error) => status = format!("MAVLink receive error: {error}"),
        }

        while event::poll(Duration::ZERO)? {
            if let Event::Key(key) = event::read()? {
                if key.code == KeyCode::Char(':')
                    && (key.kind == KeyEventKind::Press || key.kind == KeyEventKind::Repeat)
                    && keyboard_manual
                {
                    send_velocity(&mut connection, &mut sequence, ManualInput::default())?;
                    manual_input = None;
                    keyboard_manual = false;
                    control_mode = ControlMode::Hold;
                }
                if let Some(line) = read_terminal_key(
                    key,
                    &mut command_buffer,
                    &mut pressed_keys,
                    terminal_mode.reports_key_releases,
                ) {
                    match handle_command(
                        &line,
                        &mut connection,
                        &mut sequence,
                        &options,
                        &mut video_player,
                        &mut manual_input,
                        &mut last_manual_setpoint,
                        &mut control_mode,
                        &mut flight_tuning,
                        &mut status,
                    ) {
                        Ok(true) => {
                            send_velocity(&mut connection, &mut sequence, ManualInput::default())?;
                            break 'app;
                        }
                        Ok(false) if line.trim() != "help" => {
                            status = format!("Command accepted locally: {line}");
                        }
                        Ok(false) => {}
                        Err(error) => status = error.to_string(),
                    }
                }
            }
        }

        if command_buffer.is_none() {
            let input =
                manual_from_keys(&pressed_keys, Instant::now(), flight_tuning.max_speed_m_s);
            if input.is_active() {
                manual_input = Some(input);
                keyboard_manual = true;
                control_mode = ControlMode::Manual;
            } else if keyboard_manual {
                send_velocity(&mut connection, &mut sequence, ManualInput::default())?;
                manual_input = None;
                pressed_keys.clear();
                keyboard_manual = false;
                control_mode = ControlMode::Hold;
                status = "Manual input released; holding position.".to_owned();
            }
        }

        if let Some(input) = manual_input {
            if last_manual_setpoint.elapsed() >= Duration::from_millis(100) {
                send_velocity(&mut connection, &mut sequence, input)?;
                last_manual_setpoint = Instant::now();
            }
        }
        if last_dashboard_render.elapsed() >= Duration::from_millis(100) {
            render_dashboard(
                &telemetry,
                control_mode,
                manual_input,
                command_buffer.as_deref(),
                flight_tuning,
                &status,
            )?;
            last_dashboard_render = Instant::now();
        }
        std::thread::sleep(Duration::from_millis(5));
    }

    stop_video_player(&mut video_player);
    io::stdout().flush()?;
    Ok(())
}

struct TerminalMode {
    reports_key_releases: bool,
}

impl TerminalMode {
    fn enable() -> io::Result<Self> {
        terminal::enable_raw_mode()?;
        let term = std::env::var("TERM").unwrap_or_default();
        let term_program = std::env::var("TERM_PROGRAM").unwrap_or_default();
        let known_keyboard_protocol = term == "xterm-kitty"
            || matches!(
                term_program.as_str(),
                "WezTerm" | "kitty" | "Ghostty" | "iTerm.app" | "foot"
            );
        let reports_key_releases =
            known_keyboard_protocol && terminal::supports_keyboard_enhancement().unwrap_or(false);
        let mut stdout = io::stdout();
        if let Err(error) = execute!(stdout, terminal::EnterAlternateScreen, cursor::Hide) {
            let _ = terminal::disable_raw_mode();
            return Err(error);
        }
        if reports_key_releases {
            if let Err(error) = execute!(
                stdout,
                PushKeyboardEnhancementFlags(KeyboardEnhancementFlags::REPORT_EVENT_TYPES)
            ) {
                let _ = execute!(stdout, cursor::Show, terminal::LeaveAlternateScreen);
                let _ = terminal::disable_raw_mode();
                return Err(error);
            }
        }
        Ok(Self {
            reports_key_releases,
        })
    }
}

impl Drop for TerminalMode {
    fn drop(&mut self) {
        if self.reports_key_releases {
            let _ = execute!(io::stdout(), PopKeyboardEnhancementFlags);
        }
        let _ = execute!(io::stdout(), cursor::Show, terminal::LeaveAlternateScreen);
        let _ = terminal::disable_raw_mode();
    }
}

fn read_terminal_key(
    key: KeyEvent,
    command_buffer: &mut Option<String>,
    pressed_keys: &mut HashMap<KeyCode, Instant>,
    reports_key_releases: bool,
) -> Option<String> {
    if let Some(buffer) = command_buffer {
        if key.kind == KeyEventKind::Release {
            return None;
        }
        match control_key_code(key) {
            KeyCode::Enter => return command_buffer.take(),
            KeyCode::Esc => *command_buffer = None,
            KeyCode::Backspace => {
                buffer.pop();
            }
            KeyCode::Char(character) => {
                if key.modifiers.contains(KeyModifiers::SHIFT) {
                    buffer.push(character.to_ascii_uppercase());
                } else {
                    buffer.push(character);
                }
            }
            _ => {}
        }
        return None;
    }

    let code = control_key_code(key);
    if key.kind == KeyEventKind::Release {
        pressed_keys.remove(&code);
        return None;
    }
    if key.kind == KeyEventKind::Press || key.kind == KeyEventKind::Repeat {
        match code {
            KeyCode::Esc => return Some("quit".to_owned()),
            KeyCode::Char(':') => {
                pressed_keys.clear();
                *command_buffer = Some(String::new());
            }
            KeyCode::Char('w' | 'a' | 's' | 'd' | 'q' | 'e' | 'x')
            | KeyCode::Char(' ')
            | KeyCode::Down => {
                let now = Instant::now();
                if !reports_key_releases {
                    for last_event in pressed_keys.values_mut() {
                        *last_event = now;
                    }
                }
                pressed_keys.insert(code, now);
            }
            _ => {}
        }
    }
    None
}

fn control_key_code(key: KeyEvent) -> KeyCode {
    match key.code {
        KeyCode::Char(' ') if key.modifiers.contains(KeyModifiers::SHIFT) => KeyCode::Char('x'),
        KeyCode::Char(';') if key.modifiers.contains(KeyModifiers::SHIFT) => KeyCode::Char(':'),
        KeyCode::Char(character) if character.is_ascii_alphabetic() => {
            KeyCode::Char(character.to_ascii_lowercase())
        }
        code => code,
    }
}

fn manual_from_keys(
    keys: &HashMap<KeyCode, Instant>,
    now: Instant,
    max_speed_m_s: f32,
) -> ManualInput {
    let pressed = |key| {
        keys.get(&key)
            .is_some_and(|last_event| now.duration_since(*last_event) <= KEY_REPEAT_TIMEOUT)
    };
    let east =
        (i32::from(pressed(KeyCode::Char('a'))) - i32::from(pressed(KeyCode::Char('d')))) as f32;
    let north =
        (i32::from(pressed(KeyCode::Char('w'))) - i32::from(pressed(KeyCode::Char('s')))) as f32;
    let horizontal_scale = (east * east + north * north).sqrt().max(1.0);
    ManualInput {
        north: north / horizontal_scale * max_speed_m_s,
        east: east / horizontal_scale * max_speed_m_s,
        down: (i32::from(pressed(KeyCode::Down)) + i32::from(pressed(KeyCode::Char('x')))
            - i32::from(pressed(KeyCode::Char(' ')))) as f32
            * 2.0,
        yaw_rate: (i32::from(pressed(KeyCode::Char('e'))) - i32::from(pressed(KeyCode::Char('q'))))
            as f32
            * 1.5,
    }
}

struct Options {
    mavlink: String,
    video: String,
    no_video: bool,
    help: bool,
}

impl Options {
    fn parse(args: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut options = Self {
            mavlink: MAVLINK_DEFAULT.to_owned(),
            video: VIDEO_DEFAULT.to_owned(),
            no_video: false,
            help: false,
        };
        let mut args = args;
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--mavlink" => {
                    options.mavlink = args
                        .next()
                        .ok_or("--mavlink requires a connection string")?;
                }
                "--video" => {
                    options.video = args.next().ok_or("--video requires a URL")?;
                }
                "--no-video" => options.no_video = true,
                "--help" | "-h" => options.help = true,
                unknown => return Err(format!("unknown option: {unknown}")),
            }
        }
        Ok(options)
    }
}

fn send_heartbeat(
    connection: &mut mavlink::Connection<MavMessage>,
    sequence: &mut u8,
) -> Result<(), mavlink::error::MessageWriteError> {
    connection
        .send(
            &header(sequence),
            &MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                mavtype: MavType::MAV_TYPE_GCS,
                autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
                ..Default::default()
            }),
        )
        .map(|_| ())
}

fn send_command(
    connection: &mut mavlink::Connection<MavMessage>,
    sequence: &mut u8,
    command: COMMAND_LONG_DATA,
) -> Result<(), mavlink::error::MessageWriteError> {
    connection
        .send(&header(sequence), &MavMessage::COMMAND_LONG(command))
        .map(|_| ())
}

fn handle_command(
    line: &str,
    connection: &mut mavlink::Connection<MavMessage>,
    sequence: &mut u8,
    options: &Options,
    video_player: &mut Option<Child>,
    manual_input: &mut Option<ManualInput>,
    last_manual_setpoint: &mut Instant,
    control_mode: &mut ControlMode,
    flight_tuning: &mut FlightTuning,
    status: &mut String,
) -> Result<bool, Box<dyn std::error::Error>> {
    let mut parts = line.split_whitespace();
    let Some(command) = parts.next() else {
        return Ok(false);
    };
    match command {
        "help" => {
            *status = "Commands: :arm :disarm :takeoff ALT :land :goto LAT LON ALT :manual DIR... :speed M/S :accel M/S2 :tilt DEG :hold :rtl :video :stop-video :quit".to_owned();
        }
        "quit" | "exit" => return Ok(true),
        "arm" | "disarm" => {
            let arm = command == "arm";
            if arm {
                send_velocity(connection, sequence, ManualInput::default())?;
            }
            send_command(
                connection,
                sequence,
                command_long(
                    MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                    if arm { 1.0 } else { 0.0 },
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ),
            )?;
            *manual_input = None;
            *control_mode = ControlMode::Hold;
        }
        "takeoff" => {
            let altitude = parse_number(parts.next(), "altitude")
                .map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error))?;
            if !altitude.is_finite() || altitude <= 0.0 {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "takeoff altitude must be finite and greater than zero",
                )
                .into());
            }
            send_command(
                connection,
                sequence,
                command_long(
                    MavCmd::MAV_CMD_NAV_TAKEOFF,
                    0.0,
                    0.0,
                    0.0,
                    f32::NAN,
                    0.0,
                    0.0,
                    altitude,
                ),
            )?;
            *manual_input = None;
            *control_mode = ControlMode::Guided;
        }
        "land" => {
            send_command(
                connection,
                sequence,
                command_long(
                    MavCmd::MAV_CMD_NAV_LAND,
                    0.0,
                    0.0,
                    0.0,
                    f32::NAN,
                    0.0,
                    0.0,
                    0.0,
                ),
            )?;
            *manual_input = None;
            *control_mode = ControlMode::Guided;
        }
        "goto" => {
            let latitude = parse_number(parts.next(), "latitude")
                .map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error))?;
            let longitude = parse_number(parts.next(), "longitude")
                .map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error))?;
            let altitude = parse_number(parts.next(), "altitude")
                .map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error))?;
            validate_coordinates(latitude, longitude, altitude)
                .map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error))?;
            *control_mode = ControlMode::Guided;
            send_command(
                connection,
                sequence,
                command_long(
                    MavCmd::MAV_CMD_DO_REPOSITION,
                    -1.0,
                    0.0,
                    0.0,
                    f32::NAN,
                    latitude,
                    longitude,
                    altitude,
                ),
            )?;
            *manual_input = None;
            *control_mode = ControlMode::Guided;
        }
        "manual" => {
            let input = parse_manual_input(parts.collect(), flight_tuning.max_speed_m_s)
                .map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error))?;
            send_velocity(connection, sequence, input)?;
            *manual_input = input.is_active().then_some(input);
            *last_manual_setpoint = Instant::now();
            *control_mode = if input.is_active() {
                ControlMode::Manual
            } else {
                ControlMode::Hold
            };
        }
        "speed" | "accel" | "tilt" => {
            let value = parse_number(parts.next(), command)
                .map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error))?;
            if parts.next().is_some() {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!(":{command} accepts exactly one value"),
                )
                .into());
            }
            let (parameter, range, label) = match command {
                "speed" => ("VYDE_MAX_SPEED", 0.5..=30.0, "max speed"),
                "accel" => ("VYDE_MAX_ACCEL", 0.5..=20.0, "max acceleration"),
                _ => ("VYDE_MAX_TILT", 5.0..=60.0, "max tilt"),
            };
            if !range.contains(&value) {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!(
                        "{label} must be between {} and {}",
                        range.start(),
                        range.end()
                    ),
                )
                .into());
            }
            send_flight_parameter(connection, sequence, parameter, value)?;
            match command {
                "speed" => flight_tuning.max_speed_m_s = value,
                "accel" => flight_tuning.max_acceleration_m_s2 = value,
                _ => flight_tuning.max_tilt_deg = value,
            }
            *status = format!("{label} set to {value:.1}");
        }
        "hold" | "stop" => {
            send_velocity(connection, sequence, ManualInput::default())?;
            *manual_input = None;
            *control_mode = ControlMode::Hold;
        }
        "rtl" => {
            send_command(
                connection,
                sequence,
                command_long(
                    MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ),
            )?;
            *manual_input = None;
            *control_mode = ControlMode::Guided;
        }
        "video" => {
            stop_video_player(video_player);
            *video_player = Some(start_video_player(&options.video)?);
            *status = format!("Video player started: {}", options.video);
        }
        "stop-video" => {
            stop_video_player(video_player);
            *status = "Video player stopped.".to_owned();
        }
        unknown => {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("Unknown command: {unknown}. Use :help."),
            )
            .into());
        }
    }
    Ok(false)
}

fn parse_manual_input(directions: Vec<&str>, speed_m_s: f32) -> Result<ManualInput, String> {
    let mut input = ManualInput::default();
    for direction in directions {
        match direction {
            "n" | "north" => input.north = speed_m_s,
            "s" | "south" => input.north = -speed_m_s,
            "east" | "a" => input.east = speed_m_s,
            "west" | "d" => input.east = -speed_m_s,
            "up" | "space" => input.down = -2.0,
            "down" | "shift" | "x" => input.down = 2.0,
            "yaw-left" | "q" => input.yaw_rate = -1.5,
            "yaw-right" | "e" => input.yaw_rate = 1.5,
            "stop" => return Ok(ManualInput::default()),
            other => return Err(format!("unknown manual direction: {other}")),
        }
    }
    let scale = (input.north * input.north + input.east * input.east)
        .sqrt()
        .max(speed_m_s);
    input.north = input.north / scale * speed_m_s;
    input.east = input.east / scale * speed_m_s;
    Ok(input)
}

fn send_flight_parameter(
    connection: &mut mavlink::Connection<MavMessage>,
    sequence: &mut u8,
    name: &str,
    value: f32,
) -> Result<(), mavlink::error::MessageWriteError> {
    let mut param_id = [0; 16];
    for (dst, src) in param_id.iter_mut().zip(name.bytes()) {
        *dst = src;
    }
    connection
        .send(
            &header(sequence),
            &MavMessage::PARAM_SET(PARAM_SET_DATA {
                param_value: value,
                target_system: TARGET_SYSTEM,
                target_component: TARGET_COMPONENT,
                param_id: mavlink::types::CharArray::new(param_id),
                param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
            }),
        )
        .map(|_| ())
}

fn send_velocity(
    connection: &mut mavlink::Connection<MavMessage>,
    sequence: &mut u8,
    input: ManualInput,
) -> Result<(), mavlink::error::MessageWriteError> {
    let type_mask = PositionTargetTypemask::POSITION_TARGET_TYPEMASK_X_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Y_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Z_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AX_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AY_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_IGNORE;
    connection
        .send(
            &header(sequence),
            &MavMessage::SET_POSITION_TARGET_LOCAL_NED(SET_POSITION_TARGET_LOCAL_NED_DATA {
                vx: input.north,
                vy: input.east,
                vz: input.down,
                yaw: 0.0,
                yaw_rate: input.yaw_rate,
                type_mask,
                target_system: TARGET_SYSTEM,
                target_component: TARGET_COMPONENT,
                coordinate_frame: MavFrame::MAV_FRAME_LOCAL_NED,
                ..Default::default()
            }),
        )
        .map(|_| ())
}

fn command_long(
    command: MavCmd,
    param1: f32,
    param2: f32,
    param3: f32,
    param4: f32,
    param5: f32,
    param6: f32,
    param7: f32,
) -> COMMAND_LONG_DATA {
    COMMAND_LONG_DATA {
        param1,
        param2,
        param3,
        param4,
        param5,
        param6,
        param7,
        command,
        target_system: TARGET_SYSTEM,
        target_component: TARGET_COMPONENT,
        confirmation: 0,
    }
}

fn header(sequence: &mut u8) -> MavHeader {
    let current = *sequence;
    *sequence = sequence.wrapping_add(1);
    MavHeader {
        system_id: u32::from(SYSTEM_ID),
        component_id: COMPONENT_ID,
        sequence: current,
    }
}

fn update_telemetry(
    message: MavMessage,
    telemetry: &mut TelemetrySnapshot,
    flight_tuning: &mut FlightTuning,
    status: &mut String,
) {
    match message {
        MavMessage::HEARTBEAT(data) => {
            telemetry.vehicle = Some(format!("{:?} / {:?}", data.mavtype, data.system_status));
            telemetry.armed = data
                .base_mode
                .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED);
        }
        MavMessage::ATTITUDE(data) => {
            telemetry.attitude_deg = Some([
                data.roll.to_degrees(),
                data.pitch.to_degrees(),
                data.yaw.to_degrees(),
            ]);
        }
        MavMessage::GLOBAL_POSITION_INT(data) => {
            telemetry.position = Some([
                f64::from(data.lat) / 1e7,
                f64::from(data.lon) / 1e7,
                f64::from(data.alt) / 1000.0,
                f64::from(data.relative_alt) / 1000.0,
                f64::from(data.vx) / 100.0,
                f64::from(data.vy) / 100.0,
                f64::from(data.vz) / 100.0,
            ]);
        }
        MavMessage::SYS_STATUS(data) => {
            telemetry.battery = Some([
                f32::from(data.voltage_battery) / 1000.0,
                f32::from(data.current_battery) / 100.0,
                f32::from(data.battery_remaining),
            ]);
        }
        MavMessage::HIGHRES_IMU(data) => {
            telemetry.imu = Some([
                data.xacc, data.yacc, data.zacc, data.xgyro, data.ygyro, data.zgyro,
            ]);
        }
        MavMessage::COMMAND_ACK(data) => {
            *status = format!("MAVLink ACK · {:?}: {:?}", data.command, data.result);
        }
        MavMessage::PARAM_VALUE(data) => {
            if let Ok(name) = data.param_id.to_str() {
                match name {
                    "VYDE_MAX_SPEED" => flight_tuning.max_speed_m_s = data.param_value,
                    "VYDE_MAX_ACCEL" => {
                        flight_tuning.max_acceleration_m_s2 = data.param_value;
                    }
                    "VYDE_MAX_TILT" => flight_tuning.max_tilt_deg = data.param_value,
                    _ => return,
                }
                *status = format!("Simulator confirmed {name} = {:.1}", data.param_value);
            }
        }
        _ => {}
    }
}

fn render_dashboard(
    telemetry: &TelemetrySnapshot,
    control_mode: ControlMode,
    manual_input: Option<ManualInput>,
    command_buffer: Option<&str>,
    flight_tuning: FlightTuning,
    status: &str,
) -> io::Result<()> {
    let vehicle = telemetry
        .vehicle
        .as_deref()
        .unwrap_or("Waiting for MAVLink heartbeat");
    let armed = if telemetry.armed { "ARMED" } else { "DISARMED" };
    let flight = if let Some(position) = telemetry.position {
        format!(
            "GPS {:10.7} {:11.7} · altitude {:6.2} m (relative {:6.2} m)\nVelocity NED  N {:6.2}  E {:6.2}  D {:6.2} m/s",
            position[0],
            position[1],
            position[2],
            position[3],
            position[4],
            position[5],
            position[6]
        )
    } else {
        "GPS: waiting for telemetry".to_owned()
    };
    let attitude = telemetry.attitude_deg.map_or_else(
        || "Attitude: waiting for telemetry".to_owned(),
        |[roll, pitch, yaw]| {
            format!("Attitude  roll {roll:6.1}°  pitch {pitch:6.1}°  yaw {yaw:6.1}°")
        },
    );
    let battery = telemetry.battery.map_or_else(
        || "Battery: waiting for telemetry".to_owned(),
        |[voltage, current, remaining]| {
            format!("Battery {voltage:5.2} V · {current:5.1} A · {remaining:3.0}%")
        },
    );
    let manual = manual_input.map_or("Idle".to_owned(), |input| {
        format!(
            "N {:+.1} · E {:+.1} · D {:+.1} m/s · yaw {:+.1} rad/s",
            input.north, input.east, input.down, input.yaw_rate
        )
    });
    let command = command_buffer.map_or_else(
        || {
            "Type : (Shift+; on US keyboard) for commands · Flight: W/S A east · D west · Space/X Q/E"
                .to_owned()
        },
        |buffer| format!("Command (:help for help): :{buffer}"),
    );
    let safe_status = status.replace(['\n', '\r'], " ");
    let mut stdout = io::stdout().lock();
    write!(
        stdout,
        "\x1b[2J\x1b[H\
         VYDE · MAVLINK GROUND CONTROLLER\r\n\
         ════════════════════════════════════════════════════════════════════════\r\n\
         VEHICLE  {vehicle}\r\n\
         STATE    {armed} · {mode}\r\n\
         {flight}\r\n\
         {attitude}\r\n\
         {battery}\r\n\
         MANUAL   {manual} · max {max_speed:.1} m/s · accel {max_acceleration:.1} m/s² · tilt {max_tilt:.0}°\r\n\
         ────────────────────────────────────────────────────────────────────────\r\n\
         {command}\r\n\
         Status: {safe_status}\r\n\
         Commands: :arm · :takeoff ALT · :goto LAT LON ALT · :speed · :accel · :tilt · :hold · :land · :rtl · :quit\r\n",
        mode = control_mode.label(),
        max_speed = flight_tuning.max_speed_m_s,
        max_acceleration = flight_tuning.max_acceleration_m_s2,
        max_tilt = flight_tuning.max_tilt_deg,
    )?;
    stdout.flush()
}

fn start_video_player(url: &str) -> io::Result<Child> {
    Command::new("ffplay")
        .args([
            "-hide_banner",
            "-loglevel",
            "warning",
            "-fflags",
            "nobuffer",
            "-flags",
            "low_delay",
            "-framedrop",
            url,
        ])
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
}

fn stop_video_player(player: &mut Option<Child>) {
    if let Some(mut child) = player.take() {
        let _ = child.kill();
        let _ = child.wait();
    }
}

fn parse_number(value: Option<&str>, name: &str) -> Result<f32, String> {
    value
        .ok_or_else(|| format!("missing {name}"))?
        .parse::<f32>()
        .map_err(|error| format!("invalid {name}: {error}"))
}

fn validate_coordinates(latitude: f32, longitude: f32, altitude: f32) -> Result<(), String> {
    if !latitude.is_finite() || !(-90.0..=90.0).contains(&latitude) {
        return Err("latitude must be finite and between -90 and 90".to_owned());
    }
    if !longitude.is_finite() || !(-180.0..=180.0).contains(&longitude) {
        return Err("longitude must be finite and between -180 and 180".to_owned());
    }
    if !altitude.is_finite() || altitude < 0.0 {
        return Err("altitude must be finite and non-negative".to_owned());
    }
    Ok(())
}

fn print_help() {
    println!(
        "Manual: arm first; W/S north/south, A east, D west, Space up, X/down-arrow down, Q/E yaw. Chorded keys share a 750 ms dead-man timeout; key release stops immediately when supported.\nCommands (prefix with `:`): arm, disarm, takeoff ALT_M, land, goto LAT LON ALT, manual north|south|east|west|up|down|yaw-left|yaw-right, speed M_S, accel M_S2, tilt DEG, hold, rtl, video, stop-video, help, quit.\n`:speed` sets the manual speed cap (0.5–30 m/s); `:accel` limits commanded acceleration (0.5–20 m/s²); `:tilt` sets max pitch/roll tilt (5–60°). Yaw rotates the heading around vertical; pitch controls forward/back tilt and roll controls lateral tilt. Velocity/altitude modes manage thrust automatically; do not use thrust to set horizontal speed. Increasing tilt raises available horizontal acceleration; it does not change yaw.\n`hold` captures current position; `rtl` returns home and lands. Video is decoded by ffplay. On another computer, set VYDE_VIDEO_OUTPUT=udp://<controller-ip>:5600 on simulator. Options: --mavlink udpout:127.0.0.1:14550 --video udp://0.0.0.0:5600 --no-video"
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::UdpSocket;

    #[test]
    fn parses_manual_velocity_in_ned_axes() {
        let input = parse_manual_input(vec!["north", "east", "up", "yaw-left"], 3.0).unwrap();
        assert!((input.north - 3.0 / 2.0_f32.sqrt()).abs() < 0.001);
        assert!((input.east - 3.0 / 2.0_f32.sqrt()).abs() < 0.001);
        assert_eq!(input.down, -2.0);
        assert_eq!(input.yaw_rate, -1.5);
        assert_eq!(parse_manual_input(vec!["a"], 3.0).unwrap().east, 3.0);
        assert_eq!(parse_manual_input(vec!["d"], 3.0).unwrap().east, -3.0);
        assert_eq!(
            parse_manual_input(vec!["stop"], 3.0).unwrap(),
            ManualInput::default()
        );
    }

    #[test]
    fn maps_original_flight_keys_to_ned_setpoints() {
        let now = Instant::now();
        let keys = HashMap::from([
            (KeyCode::Char('w'), now),
            (KeyCode::Char('d'), now),
            (KeyCode::Char(' '), now),
            (KeyCode::Char('q'), now),
        ]);
        let input = manual_from_keys(&keys, now, 3.0);
        assert!((input.north - 3.0 / 2.0_f32.sqrt()).abs() < 0.001);
        assert!((input.east + 3.0 / 2.0_f32.sqrt()).abs() < 0.001);
        assert_eq!(input.down, -2.0);
        assert_eq!(input.yaw_rate, -1.5);
        let east = HashMap::from([(KeyCode::Char('a'), now)]);
        let west = HashMap::from([(KeyCode::Char('d'), now)]);
        assert_eq!(manual_from_keys(&east, now, 3.0).east, 3.0);
        assert_eq!(manual_from_keys(&west, now, 3.0).east, -3.0);
    }

    #[test]
    fn simultaneous_axes_are_normalized_and_stale_keys_expire() {
        let now = Instant::now();
        let diagonal = HashMap::from([
            (KeyCode::Char('w'), now),
            (KeyCode::Char('a'), now),
            (KeyCode::Char(' '), now),
        ]);
        let input = manual_from_keys(&diagonal, now, 3.0);
        assert!((input.north - 3.0 / 2.0_f32.sqrt()).abs() < 0.001);
        assert!((input.east - 3.0 / 2.0_f32.sqrt()).abs() < 0.001);
        assert_eq!(input.down, -2.0);

        let independent = HashMap::from([
            (
                KeyCode::Char('w'),
                now - KEY_REPEAT_TIMEOUT - Duration::from_millis(1),
            ),
            (KeyCode::Char('a'), now),
        ]);
        let input = manual_from_keys(&independent, now, 3.0);
        assert_eq!(input.north, 0.0);
        assert_eq!(input.east, 3.0);
    }

    #[test]
    fn key_repeat_refreshes_active_chord_and_release_removes_only_that_key() {
        let mut command = None;
        let mut pressed = HashMap::new();
        let event = |code, kind| KeyEvent::new_with_kind(code, KeyModifiers::NONE, kind);
        read_terminal_key(
            event(KeyCode::Char('w'), KeyEventKind::Press),
            &mut command,
            &mut pressed,
            false,
        );
        read_terminal_key(
            event(KeyCode::Char('a'), KeyEventKind::Press),
            &mut command,
            &mut pressed,
            false,
        );
        assert!(pressed[&KeyCode::Char('w')].elapsed() < KEY_REPEAT_TIMEOUT);
        read_terminal_key(
            event(KeyCode::Char('w'), KeyEventKind::Release),
            &mut command,
            &mut pressed,
            true,
        );
        assert!(!pressed.contains_key(&KeyCode::Char('w')));
        assert!(pressed.contains_key(&KeyCode::Char('a')));
    }

    #[test]
    fn terminal_release_and_shift_space_are_normalized() {
        let mut command = None;
        let mut pressed = HashMap::new();
        let down =
            KeyEvent::new_with_kind(KeyCode::Char(' '), KeyModifiers::SHIFT, KeyEventKind::Press);
        read_terminal_key(down, &mut command, &mut pressed, true);
        assert!(pressed.contains_key(&KeyCode::Char('x')));
        let up = KeyEvent::new_with_kind(
            KeyCode::Char(' '),
            KeyModifiers::SHIFT,
            KeyEventKind::Release,
        );
        read_terminal_key(up, &mut command, &mut pressed, true);
        assert!(!pressed.contains_key(&KeyCode::Char('x')));
    }

    #[test]
    fn terminal_command_entry_is_buffered_until_enter() {
        let mut command = None;
        let mut pressed = HashMap::new();
        let press = |code| KeyEvent::new(code, KeyModifiers::NONE);
        let shifted_colon = KeyEvent::new(KeyCode::Char(';'), KeyModifiers::SHIFT);
        assert_eq!(
            read_terminal_key(shifted_colon, &mut command, &mut pressed, false),
            None
        );
        assert_eq!(command.as_deref(), Some(""));
        for character in "goto 37.4 -122.1 12".chars() {
            read_terminal_key(
                press(KeyCode::Char(character)),
                &mut command,
                &mut pressed,
                false,
            );
        }
        assert_eq!(
            read_terminal_key(press(KeyCode::Enter), &mut command, &mut pressed, false),
            Some("goto 37.4 -122.1 12".to_owned())
        );
    }

    #[test]
    fn rejects_invalid_guided_coordinates() {
        assert!(validate_coordinates(37.4, -122.1, 10.0).is_ok());
        assert!(validate_coordinates(91.0, 0.0, 10.0).is_err());
        assert!(validate_coordinates(0.0, f32::NAN, 10.0).is_err());
        assert!(validate_coordinates(0.0, 0.0, -1.0).is_err());
    }

    #[test]
    fn options_default_to_local_simulator_and_video() {
        let options = Options::parse(std::iter::empty()).unwrap();
        assert_eq!(options.mavlink, MAVLINK_DEFAULT);
        assert_eq!(options.video, VIDEO_DEFAULT);
        assert!(!options.no_video);
    }

    #[test]
    fn speed_command_sends_mavlink_parameter_set() {
        let probe = UdpSocket::bind("127.0.0.1:0").unwrap();
        let port = probe.local_addr().unwrap().port();
        drop(probe);

        let server = connect::<MavMessage>(&format!("udpin:0.0.0.0:{port}")).unwrap();
        let mut controller = connect::<MavMessage>(&format!("udpout:127.0.0.1:{port}")).unwrap();
        let mut sequence = 0;
        let options = Options::parse(std::iter::empty()).unwrap();
        let mut video_player = None;
        let mut manual_input = None;
        let mut last_manual_setpoint = Instant::now();
        let mut control_mode = ControlMode::Hold;
        let mut flight_tuning = FlightTuning::default();
        let mut status = String::new();

        handle_command(
            "speed 5",
            &mut controller,
            &mut sequence,
            &options,
            &mut video_player,
            &mut manual_input,
            &mut last_manual_setpoint,
            &mut control_mode,
            &mut flight_tuning,
            &mut status,
        )
        .unwrap();

        let deadline = Instant::now() + Duration::from_secs(2);
        let data = loop {
            match server.try_recv() {
                Ok((_, MavMessage::PARAM_SET(data))) => break data,
                Ok(_) => {}
                Err(mavlink::error::MessageReadError::Io(error))
                    if error.kind() == io::ErrorKind::WouldBlock && Instant::now() < deadline =>
                {
                    std::thread::sleep(Duration::from_millis(5));
                }
                Err(error) => panic!("controller did not send PARAM_SET: {error}"),
            }
            assert!(
                Instant::now() < deadline,
                "controller did not send PARAM_SET"
            );
        };
        assert_eq!(data.target_system, TARGET_SYSTEM);
        assert_eq!(data.target_component, TARGET_COMPONENT);
        assert_eq!(data.param_value, 5.0);
        assert_eq!(data.param_id.to_str().unwrap(), "VYDE_MAX_SPEED");
        assert_eq!(flight_tuning.max_speed_m_s, 5.0);
    }
}
