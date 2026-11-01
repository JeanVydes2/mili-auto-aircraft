use crate::config::NetworkConfig;
use crate::motors::DroneCommand;
use crate::sensors::Telemetry;
use bevy::prelude::*;
use rand::Rng;

#[derive(Resource, Default)]
pub struct PendingCommands {
    pub queue: Vec<(f64, DroneCommand)>,
}

#[derive(Resource, Default)]
pub struct PendingTelemetry {
    pub queue: Vec<(f64, Telemetry)>,
}

#[derive(Resource, Clone, Default)]
pub struct ControllerOutput {
    pub command: DroneCommand,
}

#[derive(Resource, Clone, Default)]
pub struct ControllerTelemetry {
    pub data: Telemetry,
    pub has_data: bool,
}

#[derive(Resource, Clone, Default)]
pub struct DroneCommandBuffer {
    pub command: DroneCommand,
}

/// System: controller output -> network -> pending commands (with delay/loss).
pub fn send_commands_system(
    time: Res<Time>,
    net: Res<NetworkConfig>,
    output: Res<ControllerOutput>,
    mut pending: ResMut<PendingCommands>,
) {
    if !net.enabled {
        // Immediate: push with zero delay.
        pending
            .queue
            .push((time.elapsed_secs_f64(), output.command));
        return;
    }
    let mut rng = rand::thread_rng();
    if rng.r#gen::<f32>() < net.packet_loss {
        return; // drop
    }
    let jitter = if net.jitter_ms > 0.0 {
        rng.gen_range(-net.jitter_ms..=net.jitter_ms)
    } else {
        0.0
    };
    let delay = (net.latency_ms + jitter).max(0.0) / 1000.0;
    let deliver_at = time.elapsed_secs_f64() + delay as f64;
    pending.queue.push((deliver_at, output.command));
}

/// System: deliver due commands to drone CommandInput.
pub fn deliver_commands_system(
    time: Res<Time>,
    mut pending: ResMut<PendingCommands>,
    mut query: Query<&mut crate::motors::CommandInput, With<crate::motors::Drone>>,
) {
    let now = time.elapsed_secs_f64();
    let mut due = Vec::new();
    let mut remaining = Vec::new();
    for (t, cmd) in pending.queue.drain(..) {
        if t <= now {
            due.push(cmd);
        } else {
            remaining.push((t, cmd));
        }
    }
    pending.queue = remaining;
    if let Some(cmd) = due.last() {
        for mut input in &mut query {
            input.command = *cmd;
        }
    }
}

/// System: telemetry -> network -> controller (with delay/loss).
pub fn send_telemetry_system(
    time: Res<Time>,
    net: Res<NetworkConfig>,
    telemetry: Res<Telemetry>,
    mut pending: ResMut<PendingTelemetry>,
) {
    if !net.enabled {
        pending
            .queue
            .push((time.elapsed_secs_f64(), telemetry.clone()));
        return;
    }
    let mut rng = rand::thread_rng();
    if rng.r#gen::<f32>() < net.packet_loss {
        return;
    }
    let jitter = if net.jitter_ms > 0.0 {
        rng.gen_range(-net.jitter_ms..=net.jitter_ms)
    } else {
        0.0
    };
    let delay = (net.latency_ms + jitter).max(0.0) / 1000.0;
    let deliver_at = time.elapsed_secs_f64() + delay as f64;
    pending.queue.push((deliver_at, telemetry.clone()));
}

/// System: deliver due telemetry to controller.
pub fn deliver_telemetry_system(
    time: Res<Time>,
    mut pending: ResMut<PendingTelemetry>,
    mut ctrl: ResMut<ControllerTelemetry>,
) {
    let now = time.elapsed_secs_f64();
    let mut latest: Option<Telemetry> = None;
    let mut remaining = Vec::new();
    for (t, data) in pending.queue.drain(..) {
        if t <= now {
            latest = Some(data);
        } else {
            remaining.push((t, data));
        }
    }
    pending.queue = remaining;
    if let Some(d) = latest {
        ctrl.data = d;
        ctrl.has_data = true;
    }
}
