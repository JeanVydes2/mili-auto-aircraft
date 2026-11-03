use crate::aircraft::{
    Aircraft, AircraftCatalog, AircraftControls, AircraftModel, AircraftType, Airframe, Propulsion,
};
use crate::config::{BatteryConfig, BatteryState, FlightControlLimits};
use crate::motors::{CommandInput, DroneCommand, MotorCommand};
use crate::sensors::Telemetry;
use bevy::prelude::*;
use mavlink::dialects::common::*;
use mavlink::{MavConnection, MavHeader, Message, connect};
use std::collections::HashMap;
use std::io::ErrorKind;

const EARTH_RADIUS_M: f32 = 6_378_137.0;
const HOME_LATITUDE_DEG: f64 = 37.4;
const HOME_LONGITUDE_DEG: f64 = -122.1;
const VELOCITY_SETPOINT_TIMEOUT_SECS: f64 = 0.5;

#[derive(Resource, Clone, Debug)]
pub struct MavlinkConfig {
    pub connection_string: String,
    pub system_id: u8,
    pub component_id: u8,
}

impl Default for MavlinkConfig {
    fn default() -> Self {
        let connection_string = "udpin:0.0.0.0:14550";
        Self {
            connection_string: connection_string.to_owned(),
            system_id: 1,
            component_id: 1,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum GuidedTarget {
    Position {
        position: Vec3,
        yaw: Option<f32>,
    },
    Velocity {
        velocity: Vec3,
        yaw_rate: Option<f32>,
    },
}

#[derive(Resource)]
pub struct MavlinkInterface {
    connection: Option<mavlink::Connection<MavMessage>>,
    pub armed: bool,
    pub external_control: bool,
    pub guided_target: Option<GuidedTarget>,
    pub(crate) last_velocity_setpoint_secs: Option<f64>,
    message_intervals: HashMap<u32, Option<f64>>,
    last_message_times: HashMap<u32, f64>,
    sequence: u8,
    send_error_logged: bool,
}

impl Default for MavlinkInterface {
    fn default() -> Self {
        Self {
            connection: None,
            armed: false,
            external_control: false,
            guided_target: None,
            last_velocity_setpoint_secs: None,
            message_intervals: HashMap::new(),
            last_message_times: HashMap::new(),
            sequence: 0,
            send_error_logged: false,
        }
    }
}

impl MavlinkInterface {
    fn header(&mut self, config: &MavlinkConfig) -> MavHeader {
        let sequence = self.sequence;
        self.sequence = self.sequence.wrapping_add(1);
        MavHeader {
            system_id: u32::from(config.system_id),
            component_id: config.component_id,
            sequence,
        }
    }

    fn send(&mut self, config: &MavlinkConfig, message: MavMessage) {
        let header = self.header(config);
        if let Some(connection) = &mut self.connection {
            if let Err(error) = connection.send(&header, &message) {
                if !self.send_error_logged {
                    bevy::log::warn!("MAVLink send failed: {error}");
                    self.send_error_logged = true;
                }
            }
        }
    }

    fn send_scheduled(
        &mut self,
        config: &MavlinkConfig,
        message: MavMessage,
        default_rate_hz: f64,
        now: f64,
    ) {
        let message_id = message.message_id();
        let interval = match self.message_intervals.get(&message_id) {
            Some(Some(interval)) => *interval,
            Some(None) => return,
            None => 1.0 / default_rate_hz,
        };
        let last_sent = self
            .last_message_times
            .get(&message_id)
            .copied()
            .unwrap_or(f64::NEG_INFINITY);
        if now - last_sent >= interval {
            self.last_message_times.insert(message_id, now);
            self.send(config, message);
        }
    }
}

pub fn setup_mavlink(mut interface: ResMut<MavlinkInterface>, config: Res<MavlinkConfig>) {
    match connect::<MavMessage>(&config.connection_string) {
        Ok(connection) => {
            bevy::log::info!(
                "MAVLink listening on {} (system {}, component {})",
                config.connection_string,
                config.system_id,
                config.component_id
            );
            interface.connection = Some(connection);
        }
        Err(error) => {
            bevy::log::error!(
                "Could not open MAVLink endpoint {}: {error}",
                config.connection_string
            );
        }
    }
}

pub fn mavlink_io_system(
    time: Res<Time>,
    config: Res<MavlinkConfig>,
    telemetry: Res<Telemetry>,
    battery: Res<BatteryState>,
    battery_config: Res<BatteryConfig>,
    mut flight_limits: ResMut<FlightControlLimits>,
    aircraft_catalog: Res<AircraftCatalog>,
    mut interface: ResMut<MavlinkInterface>,
) {
    let now = time.elapsed_secs_f64();
    let mut incoming = Vec::new();
    if let Some(connection) = &mut interface.connection {
        for _ in 0..64 {
            match connection.try_recv() {
                Ok((header, message)) => incoming.push((header, message)),
                Err(mavlink::error::MessageReadError::Io(error))
                    if error.kind() == ErrorKind::WouldBlock =>
                {
                    break;
                }
                Err(error) => {
                    bevy::log::warn!("MAVLink receive failed: {error}");
                    break;
                }
            }
        }
    }

    for (_header, message) in incoming {
        match message {
            MavMessage::PARAM_SET(data)
                if target_matches(data.target_system, data.target_component, &config) =>
            {
                let name = data.param_id.to_str().unwrap_or("");
                let previous = flight_limit_value(&flight_limits, name);
                if let Some(current) = set_flight_limit(&mut flight_limits, name, data.param_value)
                {
                    if previous != Some(current) {
                        bevy::log::info!("Flight limit updated via MAVLink: {name} = {current:.1}");
                    }
                    let mut param_id = [0; 16];
                    for (dst, src) in param_id.iter_mut().zip(name.bytes()) {
                        *dst = src;
                    }
                    interface.send(
                        &config,
                        MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
                            param_value: current,
                            param_count: 3,
                            param_index: flight_limit_index(name).unwrap_or(u16::MAX),
                            param_id: mavlink::types::CharArray::new(param_id),
                            param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
                        }),
                    );
                }
            }
            MavMessage::SET_POSITION_TARGET_LOCAL_NED(data)
                if target_matches(data.target_system, data.target_component, &config) =>
            {
                if let Some(target) = local_ned_target(data, telemetry.position) {
                    interface.last_velocity_setpoint_secs =
                        matches!(target, GuidedTarget::Velocity { .. }).then_some(now);
                    interface.guided_target = Some(target);
                    interface.external_control = true;
                }
            }
            MavMessage::COMMAND_LONG(data)
                if target_matches(data.target_system, data.target_component, &config) =>
            {
                match data.command {
                    MavCmd::MAV_CMD_COMPONENT_ARM_DISARM => {
                        interface.armed = data.param1 >= 0.5;
                        interface.external_control = true;
                        interface.guided_target = None;
                        interface.last_velocity_setpoint_secs = None;
                        send_command_ack(
                            &mut interface,
                            &config,
                            data.command,
                            MavResult::MAV_RESULT_ACCEPTED,
                        );
                    }
                    MavCmd::MAV_CMD_NAV_TAKEOFF => {
                        interface.armed = true;
                        interface.external_control = true;
                        interface.last_velocity_setpoint_secs = None;
                        let height = data.param7.max(1.0);
                        interface.guided_target = Some(GuidedTarget::Position {
                            position: Vec3::new(telemetry.position.x, height, telemetry.position.z),
                            yaw: None,
                        });
                        send_command_ack(
                            &mut interface,
                            &config,
                            data.command,
                            MavResult::MAV_RESULT_ACCEPTED,
                        );
                    }
                    MavCmd::MAV_CMD_NAV_LAND => {
                        interface.external_control = true;
                        interface.last_velocity_setpoint_secs = None;
                        interface.guided_target = Some(GuidedTarget::Position {
                            position: Vec3::new(telemetry.position.x, 0.2, telemetry.position.z),
                            yaw: None,
                        });
                        send_command_ack(
                            &mut interface,
                            &config,
                            data.command,
                            MavResult::MAV_RESULT_ACCEPTED,
                        );
                    }
                    MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH => {
                        interface.external_control = true;
                        interface.last_velocity_setpoint_secs = None;
                        interface.guided_target = Some(GuidedTarget::Position {
                            position: Vec3::new(0.0, 0.2, 0.0),
                            yaw: None,
                        });
                        send_command_ack(
                            &mut interface,
                            &config,
                            data.command,
                            MavResult::MAV_RESULT_ACCEPTED,
                        );
                    }
                    MavCmd::MAV_CMD_DO_REPOSITION => {
                        let north = ((f64::from(data.param5) - HOME_LATITUDE_DEG).to_radians()
                            * f64::from(EARTH_RADIUS_M)) as f32;
                        let east = ((f64::from(data.param6) - HOME_LONGITUDE_DEG).to_radians()
                            * HOME_LATITUDE_DEG.to_radians().cos()
                            * f64::from(EARTH_RADIUS_M)) as f32;
                        interface.external_control = true;
                        interface.last_velocity_setpoint_secs = None;
                        interface.guided_target = Some(GuidedTarget::Position {
                            position: Vec3::new(east, data.param7, north),
                            yaw: data.param4.is_finite().then(|| data.param4.to_radians()),
                        });
                        send_command_ack(
                            &mut interface,
                            &config,
                            data.command,
                            MavResult::MAV_RESULT_ACCEPTED,
                        );
                    }
                    MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL => {
                        let message_id = data.param1;
                        let interval_us = data.param2;
                        if message_id.is_finite()
                            && message_id >= 0.0
                            && message_id.fract() == 0.0
                            && (interval_us == -1.0 || interval_us >= 0.0)
                        {
                            let message_id = message_id as u32;
                            if interval_us == -1.0 {
                                interface.message_intervals.insert(message_id, None);
                            } else if interval_us == 0.0 {
                                interface.message_intervals.remove(&message_id);
                            } else {
                                let interval =
                                    (f64::from(interval_us) / 1_000_000.0).clamp(0.01, 1.0);
                                interface
                                    .message_intervals
                                    .insert(message_id, Some(interval));
                            }
                            send_command_ack(
                                &mut interface,
                                &config,
                                data.command,
                                MavResult::MAV_RESULT_ACCEPTED,
                            );
                        } else {
                            send_command_ack(
                                &mut interface,
                                &config,
                                data.command,
                                MavResult::MAV_RESULT_DENIED,
                            );
                        }
                    }
                    _ => send_command_ack(
                        &mut interface,
                        &config,
                        data.command,
                        MavResult::MAV_RESULT_UNSUPPORTED,
                    ),
                }
            }
            _ => {}
        }
    }

    let now = time.elapsed_secs_f64();
    let heartbeat = HEARTBEAT_DATA {
        mavtype: mavtype_for_aircraft(&aircraft_catalog),
        autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode: MavModeFlag::from_bits_retain(if interface.armed { 136 } else { 8 }),
        system_status: if interface.armed {
            MavState::MAV_STATE_ACTIVE
        } else {
            MavState::MAV_STATE_STANDBY
        },
        ..Default::default()
    };
    interface.send_scheduled(&config, MavMessage::HEARTBEAT(heartbeat), 1.0, now);
    send_telemetry(
        &mut interface,
        &config,
        &telemetry,
        &battery,
        &battery_config,
        now,
    );
}

fn flight_limit_value(limits: &FlightControlLimits, name: &str) -> Option<f32> {
    match name {
        "VYDE_MAX_SPEED" => Some(limits.max_speed_m_s),
        "VYDE_MAX_ACCEL" => Some(limits.max_acceleration_m_s2),
        "VYDE_MAX_TILT" => Some(limits.max_tilt_deg),
        _ => None,
    }
}

fn flight_limit_index(name: &str) -> Option<u16> {
    match name {
        "VYDE_MAX_SPEED" => Some(0),
        "VYDE_MAX_ACCEL" => Some(1),
        "VYDE_MAX_TILT" => Some(2),
        _ => None,
    }
}

fn set_flight_limit(limits: &mut FlightControlLimits, name: &str, value: f32) -> Option<f32> {
    if !value.is_finite() {
        return flight_limit_value(limits, name);
    }
    match name {
        "VYDE_MAX_SPEED" if (0.5..=30.0).contains(&value) => limits.max_speed_m_s = value,
        "VYDE_MAX_ACCEL" if (0.5..=20.0).contains(&value) => {
            limits.max_acceleration_m_s2 = value;
        }
        "VYDE_MAX_TILT" if (5.0..=60.0).contains(&value) => limits.max_tilt_deg = value,
        _ => return flight_limit_value(limits, name),
    }
    Some(value)
}

pub fn apply_guided_control_system(
    time: Res<Time>,
    interface: Res<MavlinkInterface>,
    telemetry: Res<Telemetry>,
    flight_limits: Option<Res<FlightControlLimits>>,
    mut command_inputs: Query<&mut CommandInput>,
    mut aircraft: Query<
        (
            &Transform,
            &Airframe,
            &Propulsion,
            &mut AircraftControls,
            &AircraftModel,
        ),
        With<Aircraft>,
    >,
) {
    if let Ok(mut input) = command_inputs.single_mut() {
        let max_speed = flight_limits
            .as_ref()
            .map_or(FlightControlLimits::default().max_speed_m_s, |limits| {
                limits.max_speed_m_s
            });
        input.command = if interface.armed && interface.external_control {
            let target = if velocity_setpoint_timed_out(
                interface.last_velocity_setpoint_secs,
                time.elapsed_secs_f64(),
            ) {
                None
            } else {
                interface.guided_target
            };
            match target {
                Some(GuidedTarget::Position { position, yaw }) => DroneCommand::Position {
                    pos: position,
                    yaw: yaw.unwrap_or_else(|| telemetry.rotation.to_euler(EulerRot::YXZ).0),
                },
                Some(GuidedTarget::Velocity { velocity, yaw_rate }) => DroneCommand::Velocity {
                    vel: velocity.clamp_length_max(max_speed),
                    yaw_rate: yaw_rate.unwrap_or(0.0),
                },
                None => DroneCommand::None,
            }
        } else {
            DroneCommand::Motor(MotorCommand::default())
        };
    }

    for (transform, airframe, propulsion, mut controls, model) in &mut aircraft {
        if model.0 == AircraftType::Quad {
            continue;
        }
        if !interface.armed || !interface.external_control {
            controls.propulsion.fill(0.0);
            controls.surfaces.fill(0.0);
            continue;
        }
        let Some(target) = interface.guided_target else {
            continue;
        };
        let (desired_velocity, yaw_target) = match target {
            GuidedTarget::Position { position, yaw } => (
                ((position - transform.translation) * 0.8).clamp_length_max(8.0),
                yaw,
            ),
            GuidedTarget::Velocity {
                velocity,
                yaw_rate: _,
            } => (velocity, None),
        };
        let velocity_error = desired_velocity;
        let yaw_error = yaw_target
            .map(|yaw| {
                (yaw - transform.rotation.to_euler(EulerRot::YXZ).0 + std::f32::consts::PI)
                    .rem_euclid(std::f32::consts::TAU)
                    - std::f32::consts::PI
            })
            .unwrap_or(0.0);
        let is_fixed_wing = model.0 == AircraftType::FixedWing;
        let propulsor_count = controls.propulsion.len().max(1) as f32;
        for (index, throttle) in controls.propulsion.iter_mut().enumerate() {
            let hover = if is_fixed_wing {
                0.65
            } else {
                let max_thrust = propulsion
                    .0
                    .get(index)
                    .map(|propulsor| propulsor.max_thrust)
                    .unwrap_or(1.0)
                    .max(0.01);
                airframe.mass * 9.81 / propulsor_count / max_thrust
            };
            *throttle = (hover
                + velocity_error.y * 0.025
                + velocity_error.z * 0.008
                + (index as f32 - 0.5) * yaw_error * 0.015)
                .clamp(0.0, 1.0);
        }
        if is_fixed_wing {
            if let Some(elevator) = controls.surfaces.get_mut(0) {
                *elevator = (-velocity_error.y * 0.04).clamp(-1.0, 1.0);
            }
            if let Some(aileron) = controls.surfaces.get_mut(1) {
                *aileron = (-velocity_error.x * 0.04).clamp(-1.0, 1.0);
            }
            if let Some(rudder) = controls.surfaces.get_mut(2) {
                *rudder = yaw_error.clamp(-1.0, 1.0);
            }
        }
    }
}

fn velocity_setpoint_timed_out(last_setpoint: Option<f64>, now: f64) -> bool {
    last_setpoint.is_some_and(|last| now - last > VELOCITY_SETPOINT_TIMEOUT_SECS)
}

fn target_matches(target_system: u8, target_component: u8, config: &MavlinkConfig) -> bool {
    (target_system == 0 || target_system == config.system_id)
        && (target_component == 0 || target_component == config.component_id)
}

fn local_ned_target(
    data: SET_POSITION_TARGET_LOCAL_NED_DATA,
    current_position: Vec3,
) -> Option<GuidedTarget> {
    if !matches!(
        data.coordinate_frame,
        MavFrame::MAV_FRAME_LOCAL_NED | MavFrame::MAV_FRAME_LOCAL_OFFSET_NED
    ) {
        return None;
    }
    let mask = data.type_mask;
    let ignores = |flag| mask.contains(flag);
    let position_ignored = [
        PositionTargetTypemask::POSITION_TARGET_TYPEMASK_X_IGNORE,
        PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Y_IGNORE,
        PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Z_IGNORE,
    ]
    .into_iter()
    .all(ignores);
    if position_ignored {
        let velocity_ignored = [
            PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VX_IGNORE,
            PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VY_IGNORE,
            PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VZ_IGNORE,
        ];
        if velocity_ignored.into_iter().any(ignores) {
            return None;
        }
        let yaw_rate = (!ignores(PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE))
            .then_some(data.yaw_rate);
        Some(GuidedTarget::Velocity {
            velocity: Vec3::new(data.vy, -data.vz, data.vx),
            yaw_rate,
        })
    } else {
        if [
            PositionTargetTypemask::POSITION_TARGET_TYPEMASK_X_IGNORE,
            PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Y_IGNORE,
            PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Z_IGNORE,
        ]
        .into_iter()
        .any(ignores)
        {
            return None;
        }
        let mut position = Vec3::new(data.y, -data.z, data.x);
        if data.coordinate_frame == MavFrame::MAV_FRAME_LOCAL_OFFSET_NED {
            position += current_position;
        }
        let yaw = (!ignores(PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_IGNORE))
            .then_some(data.yaw);
        Some(GuidedTarget::Position { position, yaw })
    }
}

fn send_command_ack(
    interface: &mut MavlinkInterface,
    config: &MavlinkConfig,
    command: MavCmd,
    result: MavResult,
) {
    interface.send(
        config,
        MavMessage::COMMAND_ACK(COMMAND_ACK_DATA { command, result }),
    );
}

fn send_telemetry(
    interface: &mut MavlinkInterface,
    config: &MavlinkConfig,
    telemetry: &Telemetry,
    battery: &BatteryState,
    battery_config: &BatteryConfig,
    now: f64,
) {
    let attitude_ned = sim_to_ned_rotation(telemetry.rotation);
    let (yaw, pitch, roll) = attitude_ned.to_euler(EulerRot::ZYX);
    interface.send_scheduled(
        config,
        MavMessage::ATTITUDE(ATTITUDE_DATA {
            time_boot_ms: millis(now),
            roll,
            pitch,
            yaw,
            rollspeed: body_frd(telemetry.gyro_body).x,
            pitchspeed: body_frd(telemetry.gyro_body).y,
            yawspeed: body_frd(telemetry.gyro_body).z,
        }),
        10.0,
        now,
    );

    let latitude =
        HOME_LATITUDE_DEG + f64::from(telemetry.position.z / EARTH_RADIUS_M).to_degrees();
    let longitude = HOME_LONGITUDE_DEG
        + f64::from(
            telemetry.position.x / (EARTH_RADIUS_M * (HOME_LATITUDE_DEG.to_radians().cos() as f32)),
        )
        .to_degrees();
    interface.send_scheduled(
        config,
        MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA {
            time_boot_ms: millis(now),
            lat: (latitude * 1e7) as i32,
            lon: (longitude * 1e7) as i32,
            alt: (telemetry.position.y * 1000.0) as i32,
            relative_alt: (telemetry.position.y * 1000.0) as i32,
            vx: (telemetry.velocity.z * 100.0) as i16,
            vy: (telemetry.velocity.x * 100.0) as i16,
            vz: (-telemetry.velocity.y * 100.0) as i16,
            hdg: ((yaw.to_degrees().rem_euclid(360.0) * 100.0) as u16),
        }),
        10.0,
        now,
    );

    interface.send_scheduled(
        config,
        MavMessage::HIGHRES_IMU(HIGHRES_IMU_DATA {
            time_usec: (now * 1_000_000.0) as u64,
            xacc: body_frd(telemetry.accel_body).x,
            yacc: body_frd(telemetry.accel_body).y,
            zacc: body_frd(telemetry.accel_body).z,
            xgyro: body_frd(telemetry.gyro_body).x,
            ygyro: body_frd(telemetry.gyro_body).y,
            zgyro: body_frd(telemetry.gyro_body).z,
            xmag: body_frd(telemetry.mag_body).x,
            ymag: body_frd(telemetry.mag_body).y,
            zmag: body_frd(telemetry.mag_body).z,
            fields_updated: HighresImuUpdatedFlags::from_bits_retain(0x0fff),
            ..Default::default()
        }),
        10.0,
        now,
    );

    interface.send_scheduled(
        config,
        MavMessage::SYS_STATUS(SYS_STATUS_DATA {
            onboard_control_sensors_present: MavSysStatusSensor::from_bits_retain(0x2f),
            onboard_control_sensors_enabled: MavSysStatusSensor::from_bits_retain(0x2f),
            onboard_control_sensors_health: MavSysStatusSensor::from_bits_retain(0x2f),
            voltage_battery: (battery.voltage.max(0.0) * 1000.0) as u16,
            current_battery: (battery.current_a * 100.0) as i16,
            battery_remaining: (battery.soc.clamp(0.0, 1.0) * 100.0) as i8,
            ..Default::default()
        }),
        10.0,
        now,
    );

    let capacity = battery_config.capacity_mah.max(0.0);
    let mut voltages = [u16::MAX; 10];
    voltages[0] = (battery.voltage.max(0.0) * 1000.0).min(u16::MAX as f32) as u16;
    interface.send_scheduled(
        config,
        MavMessage::BATTERY_STATUS(BATTERY_STATUS_DATA {
            current_consumed: (capacity * (1.0 - battery.soc.clamp(0.0, 1.0))) as i32,
            energy_consumed: -1,
            temperature: (battery.temperature_c * 100.0) as i16,
            voltages,
            current_battery: (battery.current_a * 100.0) as i16,
            id: 0,
            battery_function: MavBatteryFunction::MAV_BATTERY_FUNCTION_ALL,
            mavtype: MavBatteryType::MAV_BATTERY_TYPE_LIPO,
            battery_remaining: (battery.soc.clamp(0.0, 1.0) * 100.0) as i8,
        }),
        10.0,
        now,
    );
}

fn millis(seconds: f64) -> u32 {
    (seconds.max(0.0) * 1000.0) as u32
}

fn mavtype_for_aircraft(catalog: &AircraftCatalog) -> MavType {
    match catalog
        .definitions
        .get(catalog.selected)
        .map(|profile| profile.kind)
    {
        Some(AircraftType::Quad) => MavType::MAV_TYPE_QUADROTOR,
        Some(AircraftType::Hex) => MavType::MAV_TYPE_HEXAROTOR,
        Some(AircraftType::FixedWing) => MavType::MAV_TYPE_FIXED_WING,
        Some(AircraftType::Vtol) => MavType::MAV_TYPE_VTOL_FIXEDROTOR,
        None => MavType::MAV_TYPE_GENERIC,
    }
}

fn sim_to_ned_rotation(rotation: Quat) -> Quat {
    let ned_to_sim = Mat3::from_cols(Vec3::Z, Vec3::X, Vec3::NEG_Y);
    Quat::from_mat3(&(ned_to_sim.transpose() * Mat3::from_quat(rotation) * ned_to_sim))
}

fn body_frd(vector: Vec3) -> Vec3 {
    Vec3::new(vector.z, vector.x, -vector.y)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::UdpSocket;
    use std::thread;
    use std::time::{Duration, Instant};

    #[test]
    fn mavlink_parameters_update_bounded_flight_limits() {
        let mut limits = FlightControlLimits::default();
        assert_eq!(
            set_flight_limit(&mut limits, "VYDE_MAX_SPEED", 12.0),
            Some(12.0)
        );
        assert_eq!(
            set_flight_limit(&mut limits, "VYDE_MAX_ACCEL", 4.0),
            Some(4.0)
        );
        assert_eq!(
            set_flight_limit(&mut limits, "VYDE_MAX_TILT", 35.0),
            Some(35.0)
        );
        assert_eq!(limits.max_speed_m_s, 12.0);
        assert_eq!(limits.max_acceleration_m_s2, 4.0);
        assert_eq!(limits.max_tilt_deg, 35.0);
        assert_eq!(
            set_flight_limit(&mut limits, "VYDE_MAX_TILT", 90.0),
            Some(35.0)
        );
        assert_eq!(set_flight_limit(&mut limits, "UNKNOWN", 1.0), None);
        assert_eq!(flight_limit_index("VYDE_MAX_TILT"), Some(2));
    }

    #[test]
    fn manual_velocity_control_expires_without_a_refresh() {
        assert!(!velocity_setpoint_timed_out(None, 10.0));
        assert!(!velocity_setpoint_timed_out(Some(1.0), 1.5));
        assert!(velocity_setpoint_timed_out(Some(1.0), 1.501));
    }

    #[test]
    fn local_ned_velocity_maps_to_world_axes() {
        let target = local_ned_target(
            SET_POSITION_TARGET_LOCAL_NED_DATA {
                vx: 3.0,
                vy: 4.0,
                vz: 5.0,
                yaw_rate: 0.5,
                type_mask: PositionTargetTypemask::POSITION_TARGET_TYPEMASK_X_IGNORE
                    | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Y_IGNORE
                    | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Z_IGNORE,
                coordinate_frame: MavFrame::MAV_FRAME_LOCAL_NED,
                ..Default::default()
            },
            Vec3::ZERO,
        );
        assert!(matches!(
            target,
            Some(GuidedTarget::Velocity { velocity, .. })
                if velocity == Vec3::new(4.0, -5.0, 3.0)
        ));
        assert!(matches!(
            target,
            Some(GuidedTarget::Velocity {
                yaw_rate: Some(0.5),
                ..
            })
        ));
    }

    #[test]
    fn local_ned_position_maps_to_world_axes() {
        let target = local_ned_target(
            SET_POSITION_TARGET_LOCAL_NED_DATA {
                x: 10.0,
                y: 20.0,
                z: -30.0,
                type_mask: PositionTargetTypemask::empty(),
                coordinate_frame: MavFrame::MAV_FRAME_LOCAL_NED,
                ..Default::default()
            },
            Vec3::ZERO,
        );
        assert!(matches!(
            target,
            Some(GuidedTarget::Position { position, .. })
                if position == Vec3::new(20.0, 30.0, 10.0)
        ));
    }

    #[test]
    fn rejects_non_local_frames_and_partial_position_masks() {
        let unsupported = SET_POSITION_TARGET_LOCAL_NED_DATA {
            coordinate_frame: MavFrame::MAV_FRAME_LOCAL_ENU,
            ..Default::default()
        };
        assert!(local_ned_target(unsupported, Vec3::ZERO).is_none());

        let partial = SET_POSITION_TARGET_LOCAL_NED_DATA {
            type_mask: PositionTargetTypemask::POSITION_TARGET_TYPEMASK_X_IGNORE,
            ..Default::default()
        };
        assert!(local_ned_target(partial, Vec3::ZERO).is_none());
    }

    #[test]
    fn udp_endpoint_accepts_and_replies_to_a_mavlink_crate_client() {
        let probe = UdpSocket::bind("127.0.0.1:0").unwrap();
        let port = probe.local_addr().unwrap().port();
        drop(probe);

        let server = mavlink::connect::<MavMessage>(&format!("udpin:0.0.0.0:{port}")).unwrap();
        let client = mavlink::connect::<MavMessage>(&format!("udpout:127.0.0.1:{port}")).unwrap();
        let header = MavHeader {
            system_id: 255,
            component_id: 190,
            sequence: 0,
        };
        let heartbeat = MavMessage::HEARTBEAT(HEARTBEAT_DATA::default());
        client.send(&header, &heartbeat).unwrap();

        let deadline = Instant::now() + Duration::from_secs(2);
        let received = loop {
            match server.try_recv() {
                Ok((_, message)) => break message,
                Err(mavlink::error::MessageReadError::Io(error))
                    if error.kind() == ErrorKind::WouldBlock && Instant::now() < deadline =>
                {
                    thread::sleep(Duration::from_millis(5));
                }
                Err(error) => panic!("server did not receive client packet: {error}"),
            }
        };
        assert!(matches!(received, MavMessage::HEARTBEAT(_)));

        server
            .send(
                &MavHeader {
                    system_id: 1,
                    component_id: 1,
                    sequence: 0,
                },
                &heartbeat,
            )
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        let received = loop {
            match client.try_recv() {
                Ok((_, message)) => break message,
                Err(mavlink::error::MessageReadError::Io(error))
                    if error.kind() == ErrorKind::WouldBlock && Instant::now() < deadline =>
                {
                    thread::sleep(Duration::from_millis(5));
                }
                Err(error) => panic!("client did not receive server packet: {error}"),
            }
        };
        assert!(matches!(received, MavMessage::HEARTBEAT(_)));
    }
}
