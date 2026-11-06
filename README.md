# Vyde Drone Simulation

Quadrotor simulator in Rust, Bevy and Avian. Control and physics run at **240 Hz**, independently of render FPS.

## Repository layout

```
.
├── simulation/            # drone_sim binary (Bevy + Avian physics)
│   ├── src/
│   │   ├── main.rs        # app setup, aircraft selection (keys 1-4)
│   │   ├── aircraft.rs    # generic rigid-body: propulsion, aero surfaces, wrench
│   │   ├── motors.rs      # quad inner loop: attitude/velocity/position control
│   │   ├── mavlink.rs     # MAVLink GCS link (telemetry in, commands out)
│   │   ├── stream.rs      # simulated video link (H.264 UDP) + profiles
│   │   ├── camera.rs      # FPV / chase / free cameras
│   │   ├── environment.rs # ruined city map + crash/explosion
│   │   ├── physics.rs     # quad spawn, gravity
│   │   ├── sensors.rs     # telemetry
│   │   ├── battery.rs     # battery model
│   │   ├── hud.rs         # in-sim HUD (English)
│   │   ├── visuals.rs     # procedural models and range decoration
│   │   ├── config.rs      # all tunable parameters
│   │   ├── domain_randomization.rs
│   │   ├── motor_rpms.rs
│   │   └── telemetry_log.rs
│   └── assets/aircraft_profiles.toml  # editable aircraft (quad/hex/fixed-wing/vtol)
├── controller/            # controller binary (Axum + MAVLink + web UI)
│   ├── src/backend.rs     # HTTP API, video receiver, multi-vehicle sessions
│   └── web/index.html     # flight interface (English)
├── README.md              # this file
└── Cargo.toml             # workspace (simulation + controller)
```

## Run

```sh
cargo run -p drone_sim

# Headless: the autonomous controller holds (0, 2, 0).
HEADLESS=1 cargo run -p drone_sim

# MAVLink backend and its web interface
cargo run -p controller
```

The backend serves the interface at `http://127.0.0.1:8080` and an HTTP API for other clients
(`GET /api/state`, `POST /api/command`, `POST /api/manual`, `POST /api/stop`). Commands receive
`{"command":"arm"}`; manual control receives NED velocities `{"north":0,"east":0,"down":0,"yaw_rate":0}`
(m/s and rad/s) and must be refreshed periodically: the backend releases the setpoint if no updates
arrive within 350 ms. The API validates the configured speed limits. The link is local-only by
default; for remote control, explicitly bind a trusted network interface, e.g.
`cargo run -p controller -- --bind 0.0.0.0:8080`. The API has no authentication: do not expose it
directly to the internet. To change the MAVLink destination, use `--mavlink udpout:127.0.0.1:14550`.
For multi-aircraft runs, use the same count in both processes, for example
`MULTI_COUNT=3 cargo run -p drone_sim` and
`VYDE_VEHICLE_COUNT=3 cargo run -p controller`. This provisions MAVLink IDs
1..3 and video feeds on UDP ports 5600..5602; the browser vehicle selector
switches telemetry, commands, MJPEG and WebRTC together.

The page includes the camera livestream: the simulator transmits H.264 over UDP to
`127.0.0.1:5600` by default; the backend receives it, decodes it with `ffmpeg` and serves it as
MJPEG from the same origin. Install `ffmpeg` on both machines and run the backend next to the
browser. If the simulator runs on another machine, point `VYDE_VIDEO_OUTPUT` at the backend IP
and configure `--video udp://0.0.0.0:5600` (the default receiver). You can also select
`--video rtsp://...` for an RTSP source.

The video link profile (Clean / Nominal / Poor / Lost) is selected with `V` in the
simulator, or at startup with `VYDE_VIDEO_PROFILE=clean|nominal|poor|lost`. The profile
affects the stream delivered to the web: latency, packet loss, corruption and interference.

## Controls

| Key | Action |
| --- | --- |
| W / S | Move forward / backward along the FPV heading |
| A / D | Strafe left / right without yawing |
| Space / Shift | Climb / descend |
| Q / E | Yaw left / right |
| Ctrl+F | Toggle FPV and free camera |
| R | Reset position, velocities, motors and stabilizer memory |

Releasing the keys makes the drone brake and hold position, altitude and heading. In free
camera, the keys move the camera and the drone holds its current position. Resets do not
recharge the battery.

## Aircraft

Select with keys **1-4** in the simulator or the HUD buttons:

- **1 - Quad**: multirotor inner loop (`motors.rs`), hovers, WASD strafes.
- **2 - Hex**: generic multirotor (`multirotor_controls`), 6 lift rotors.
- **3 - FixedWing**: throttle in manual (W = throttle, the aircraft accelerates by
  physics and takes off by itself past ~9 m/s); Q/E work on the ground to orient takeoff.
  Guided `takeoff` imposes a minimum cruise speed along the current heading.
- **4 - Vtol**: hover on lift rotors in manual; cruise pusher only in guided mode.

Physical properties are editable in `simulation/assets/aircraft_profiles.toml`.

## Flight model

- **Stabilizer:** quaternion attitude error in body coordinates, inertia-scaled angular
  damping, position and velocity loops, tilt compensation and a bounded integral to reject
  steady wind.
- **Motors:** mixing by geometry and individual coefficients; saturation that prioritizes
  collective thrust, response lag, RPM slew limit and a speed ceiling by available voltage.
- **Rigid body:** Avian integrates forces and torques, gravity, inertia and gyroscopic
  motion. Collisions use the drone collider and the same geometry in graphical and headless
  mode; the ground allows landing and obstacle contacts trigger a reset.
- **Aerodynamics:** linear and quadratic drag by air-relative speed, density, angular
  damping and a small aerodynamic moment about the center of mass. Gradual ground effect by
  altitude and tilt.
- **Wind:** constant speed, smooth gusts with configurable direction and period, and
  reproducible turbulence with spatial variation. Moderate wind by default.
- **Battery:** rotor power proportional to `Q·ω`, electrical losses, voltage sag under load,
  discharge-curve interpolation and gradual thermal response.
- **Network:** configurable latency, jitter and packet loss for commands and telemetry; the
  internal stabilizer keeps running at physics rate.

This is an approximate flight model, not CFD nor a calibrated model for a real drone.
Turbulence uses smooth deterministic signals, not a weather model. Multiple drones are supported with independent MAVLink system IDs, telemetry snapshots,
manual-control leases and video feeds. The browser controller currently exposes the feeds
through an MJPEG compatibility bridge; the simulator transport remains H.264 over UDP,
which is the same low-latency transport family used by real ground-control links.

## Map and collisions

The map is a ruined city (damaged buildings, collapsed blocks, rubble piles and craters),
all with real colliders. An impact against any obstacle (buildings, rubble, craters, test
cubes) explodes the drone only if the impact speed is at least **2.5 m/s**
(`CRASH_SPEED_M_S` in `environment.rs`); slower contacts are soft landings. The ground never
explodes: landing is always safe.

## Configuration

Parameters are in `simulation/src/config.rs`:

- `DroneConfig`: mass, inertia, motors, drag and ground effect.
- `EnvironmentConfig`: gravity, density and wind. Speeds in m/s, gust duration in seconds and
  turbulence length in metres.
- `BatteryConfig`, `SensorConfig`, `NetworkConfig`, `DomainRandomizationConfig`, `CameraConfig`, `SimConfig`.

For calm air, set `wind_constant`, `wind_gust_intensity` and `turbulence_intensity` to zero.
Domain randomization and network simulation are disabled by default.

`DroneController` allows implementing autonomous controllers via position, velocity,
attitude, thrust/torque or RPM commands. `DroneCommand::None` means hold the current
position; to cut motors use `MotorCommand { rpms: [0.0; 4] }`.

## Validation

```sh
cargo test -p drone_sim
cargo test -p controller
```

Includes unit tests for controls, mixing, motors, wind, aerodynamics and battery, plus
headless flights with real physics: tilt recovery, hover with wind and latency, braking and
position holding, unpowered landing, crashes, resets and FPS independence.
