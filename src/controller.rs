use crate::config::DroneConfig;
use crate::motors::{DroneCommand, MotorCommand, mix_thrust_torque};
use crate::network::{ControllerOutput, ControllerTelemetry};
use crate::sensors::Telemetry;
use bevy::prelude::*;

pub trait DroneController: Send + Sync {
    fn update(&mut self, telemetry: &Telemetry, dt: f32, cfg: &DroneConfig) -> DroneCommand;
}

#[derive(Resource)]
pub struct ActiveController(pub Box<dyn DroneController>);

/// Example cascaded PID-ish attitude + altitude controller.
pub struct AttitudeAltitudeController {
    pub target_altitude: f32,
    pub target_position: Vec3,
    pub kp_alt: f32,
    pub kd_alt: f32,
    pub kp_pos: f32,
    pub kp_att: f32,
    pub kd_att: f32,
}

impl Default for AttitudeAltitudeController {
    fn default() -> Self {
        Self {
            target_altitude: 2.0,
            target_position: Vec3::new(0.0, 0.0, 0.0),
            kp_alt: 12.0,
            kd_alt: 6.0,
            kp_pos: 0.3,
            kp_att: 3.0,
            kd_att: 4.0,
        }
    }
}

impl DroneController for AttitudeAltitudeController {
    fn update(&mut self, telemetry: &Telemetry, _dt: f32, cfg: &DroneConfig) -> DroneCommand {
        // Altitude control -> thrust.
        let e_z = self.target_altitude - telemetry.position.y;
        let mut thrust = cfg.mass * 9.81 + self.kp_alt * e_z - self.kd_alt * telemetry.velocity.y;
        if telemetry.position.y < 0.5 {
            thrust *= 1.5;
        }
        let thrust = thrust.clamp(0.0, cfg.mass * 9.81 * 2.0);

        // Basic stabilizer: level attitude, hold altitude.
        let desired_pitch = 0.0;
        let desired_roll = 0.0;

        // Attitude: desired level + velocity damping.
        let (roll, pitch, _yaw) = telemetry.rotation.to_euler(EulerRot::XYZ);
        // Recovery if tilted too much.
        if roll.abs() > 0.5 || pitch.abs() > 0.5 {
            // Override desired to level.
        }
        let angvel_body = telemetry.rotation.inverse() * telemetry.angular_velocity;

        let gain = if roll.abs() > 0.5 || pitch.abs() > 0.5 {
            self.kp_att * 2.0
        } else {
            self.kp_att
        };
        let damp = if roll.abs() > 0.5 || pitch.abs() > 0.5 {
            self.kd_att * 2.0
        } else {
            self.kd_att
        };
        let torque_x = (gain * (roll - desired_roll) + damp * angvel_body.x).clamp(-1.0, 1.0);
        let torque_y = 0.0;
        let torque_z = (gain * (pitch - desired_pitch) + damp * angvel_body.z).clamp(-1.0, 1.0);

        let rpms = mix_thrust_torque(
            cfg,
            thrust.max(0.0),
            Vec3::new(torque_x, torque_y, torque_z),
        );
        DroneCommand::Motor(MotorCommand { rpms })
    }
}

/// System: run active controller or manual keyboard control.
pub fn run_controller_system(
    time: Res<Time>,
    cfg: Res<DroneConfig>,
    ctrl_tel: Res<ControllerTelemetry>,
    free_active: Res<crate::camera::FreeCamActive>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut active: ResMut<ActiveController>,
    mut output: ResMut<ControllerOutput>,
) {
    let dt = time.delta_secs();
    if !ctrl_tel.has_data {
        // No telemetry yet - output hover command as fallback.
        output.command = DroneCommand::Attitude {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            thrust: cfg.mass * 9.81,
        };
        return;
    }

    if !free_active.0 {
        let any_input = keyboard.pressed(KeyCode::KeyW)
            || keyboard.pressed(KeyCode::KeyA)
            || keyboard.pressed(KeyCode::KeyS)
            || keyboard.pressed(KeyCode::KeyD)
            || keyboard.pressed(KeyCode::Space)
            || keyboard.pressed(KeyCode::ShiftLeft)
            || keyboard.pressed(KeyCode::ShiftRight)
            || keyboard.pressed(KeyCode::KeyQ)
            || keyboard.pressed(KeyCode::KeyE);

        if !any_input {
            // Autopilot basic stabilizer (hover).
            let cmd = active.0.update(&ctrl_tel.data, dt, &cfg);
            output.command = cmd;
            return;
        }

        // Manual control (FPV mode): WASD + Space/Shift + Q/E or A/D yaw.
        let data = &ctrl_tel.data;
        let yaw = data.rotation.to_euler(EulerRot::XYZ).2;
        let (sy, cy) = yaw.sin_cos();
        let forward = Vec3::new(sy, 0.0, cy);
        let right = Vec3::new(cy, 0.0, -sy);

        let mut dir = Vec3::ZERO;
        if keyboard.pressed(KeyCode::KeyW) {
            dir += forward;
        }
        if keyboard.pressed(KeyCode::KeyS) {
            dir -= forward;
        }
        if keyboard.pressed(KeyCode::KeyA) {
            dir -= right;
        }
        if keyboard.pressed(KeyCode::KeyD) {
            dir += right;
        }
        if keyboard.pressed(KeyCode::Space) {
            dir.y += 1.0;
        }
        if keyboard.pressed(KeyCode::ShiftLeft) || keyboard.pressed(KeyCode::ShiftRight) {
            dir.y -= 1.0;
        }

        let speed = 3.0;
        let desired_vel = dir.normalize_or_zero() * speed;
        let yaw_rate = if keyboard.pressed(KeyCode::KeyA) || keyboard.pressed(KeyCode::KeyQ) {
            1.5
        } else if keyboard.pressed(KeyCode::KeyD) || keyboard.pressed(KeyCode::KeyE) {
            -1.5
        } else {
            0.0
        };

        output.command = DroneCommand::Velocity {
            vel: desired_vel,
            yaw_rate,
        };
        return;
    }

    let cmd = active.0.update(&ctrl_tel.data, dt, &cfg);
    output.command = cmd;
}
