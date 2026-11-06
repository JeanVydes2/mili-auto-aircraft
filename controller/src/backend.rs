use axum::{
    Json, Router,
    body::{Body, Bytes},
    extract::{Path, State},
    http::{StatusCode, header},
    response::{Html, Response},
    routing::{get, post},
};
use bytes::Bytes as MediaBytes;
use futures_util::stream;
use mavlink::dialects::common::*;
use mavlink::{MavConnection, MavHeader, connect};
use serde::{Deserialize, Serialize};
use std::io::{self, Read};
use std::net::SocketAddr;
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, RwLock, mpsc};
use std::thread::JoinHandle;
use std::time::Duration;
use std::time::Instant;
use tokio::sync::oneshot;
use webrtc::api::API;
use webrtc::api::APIBuilder;
use webrtc::api::interceptor_registry::register_default_interceptors;
use webrtc::interceptor::registry::Registry;
use webrtc::api::media_engine::MediaEngine;
use webrtc::peer_connection::RTCPeerConnection;
use webrtc::peer_connection::configuration::RTCConfiguration;
use webrtc::peer_connection::sdp::session_description::RTCSessionDescription;
use webrtc::rtp_transceiver::rtp_codec::RTCRtpCodecCapability;
use webrtc::track::track_local::TrackLocal;
use webrtc::track::track_local::track_local_static_sample::TrackLocalStaticSample;
use webrtc_media::Sample;

fn system_id_for(index: usize) -> u8 {
    (index + 1).min(255) as u8
}

/// Build the video URL for feed `index` from the base `VIDEO_DEFAULT`.
/// Feed 0 keeps the legacy `udp://0.0.0.0:5600`; feed 1 gets `udp://0.0.0.0:5601`,
/// etc. This mirrors the simulator side (`video_output_url` in stream.rs).
fn video_url_for_feed(base: &str, index: usize) -> String {
    if index == 0 {
        return base.to_owned();
    }
    if let Some(rest) = base.strip_prefix("udp://") {
        let (host_port, suffix) = match rest.split_once('?') {
            Some((hp, tail)) => (hp, Some(tail)),
            None => (rest, None),
        };
        if let Some((host, port)) = host_port.rsplit_once(':') {
            if let Ok(port_num) = port.parse::<u16>() {
                return format!(
                    "udp://{}:{}{}",
                    host,
                    port_num + index as u16,
                    suffix.map_or_else(String::new, |tail| format!("?{tail}"))
                );
            }
        }
    }
    base.to_owned()
}

const SYSTEM_ID: u8 = 255;
const COMPONENT_ID: u8 = 190;
const TARGET_SYSTEM: u8 = 1;
const TARGET_COMPONENT: u8 = 1;
const MAVLINK_DEFAULT: &str = "udpout:127.0.0.1:14550";
const BIND_DEFAULT: &str = "127.0.0.1:8080";
const VIDEO_DEFAULT: &str = "udp://0.0.0.0:5600";
const MANUAL_LEASE: Duration = Duration::from_millis(350);
const MAX_JPEG_BYTES: usize = 8 * 1024 * 1024;

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

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
struct ManualInput {
    north: f32,
    east: f32,
    down: f32,
    yaw_rate: f32,
}

impl ManualInput {
    fn is_active(self) -> bool {
        self.north != 0.0 || self.east != 0.0 || self.down != 0.0 || self.yaw_rate != 0.0
    }
}

#[derive(Clone, Copy, Debug, Serialize)]
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

/// Rotates a manual input (forward/right, FPV style) into the NED frame
/// using the vehicle's current heading. The drone forward is +Z in the sim
/// = North in NED when yaw is 0. With yaw ψ (clockwise seen from above,
/// same in Bevy +Y and in NED):
///   forward_unit = (N=cosψ, E=sinψ)
///   right_unit   = (N=-sinψ, E=cosψ)   // forward rotated 90° clockwise
fn body_to_ned(forward: f32, right: f32, yaw_rad: f32) -> (f32, f32) {
    let (s, c) = yaw_rad.sin_cos();
    // forward unit: (N=c, E=s); right unit: (N=-s, E=c)
    (forward * c - right * s, forward * s + right * c)
}

#[derive(Clone, Default, Serialize)]
struct TelemetrySnapshot {
    vehicle: Option<String>,
    armed: bool,
    attitude_deg: Option<[f32; 3]>,
    position: Option<[f64; 7]>,
    battery: Option<[f32; 3]>,
    imu: Option<[f32; 6]>,
}

#[derive(Clone, Serialize)]
struct BackendSnapshot {
    vehicle_id: u8,
    mavlink: String,
    linked: bool,
    last_seen_secs_ago: Option<u64>,
    telemetry: TelemetrySnapshot,
    control_mode: &'static str,
    manual_input: Option<ManualInput>,
    flight_tuning: FlightTuning,
    status: String,
}

struct VehicleSession {
    vehicle_id: u8,
    sender: mpsc::Sender<BackendMessage>,
    snapshot: Arc<RwLock<BackendSnapshot>>,
    video: Arc<RwLock<Option<VideoFrame>>>,
    webrtc_track: Arc<TrackLocalStaticSample>,
}

struct AppState {
    sessions: Vec<VehicleSession>,
    webrtc_api: Arc<API>,
    webrtc_peers: Arc<tokio::sync::Mutex<Vec<Arc<RTCPeerConnection>>>>,
}

impl AppState {
    fn session(&self, vehicle_id: u8) -> Result<&VehicleSession, (StatusCode, String)> {
        self.sessions
            .iter()
            .find(|session| session.vehicle_id == vehicle_id)
            .ok_or_else(|| {
                (
                    StatusCode::SERVICE_UNAVAILABLE,
                    format!("Vehicle {vehicle_id} is not configured"),
                )
            })
    }
}

#[derive(Clone)]
struct VideoFrame {
    jpeg: Arc<[u8]>,
    received_at: Instant,
}

struct VideoReceiverProcess {
    child: Child,
    reader: JoinHandle<()>,
}

fn build_webrtc_api() -> Result<Arc<API>, Box<dyn std::error::Error>> {
    let mut media_engine = MediaEngine::default();
    media_engine.register_default_codecs()?;
    let mut registry = Registry::new();
    registry = register_default_interceptors(registry, &mut media_engine)?;
    Ok(Arc::new(
        APIBuilder::new()
            .with_media_engine(media_engine)
            .with_interceptor_registry(registry)
            .build(),
    ))
}

enum BackendMessage {
    Command {
        line: String,
        reply: oneshot::Sender<Result<String, String>>,
    },
    Manual {
        input: ManualInput,
        reply: oneshot::Sender<Result<String, String>>,
    },
    Stop {
        reply: oneshot::Sender<Result<String, String>>,
    },
}

#[derive(Deserialize)]
struct CommandRequest {
    command: String,
}

#[derive(Deserialize)]
struct ManualRequest {
    north: f32,
    east: f32,
    down: f32,
    yaw_rate: f32,
}

#[derive(Serialize)]
struct ApiResponse {
    ok: bool,
    message: String,
}

struct Options {
    mavlink: Vec<String>,
    bind: String,
    video: Vec<String>,
    help: bool,
    mavlink_was_set: bool,
    video_was_set: bool,
}

impl Options {
    fn parse(args: impl Iterator<Item = String>) -> Result<Self, String> {
        let vehicle_count = std::env::var("VYDE_VEHICLE_COUNT")
            .or_else(|_| std::env::var("MULTI_COUNT"))
            .ok()
            .and_then(|value| value.parse::<usize>().ok())
            .unwrap_or(1)
            .clamp(1, 255);
        let mut options = Self {
            mavlink: (0..vehicle_count)
                .map(|_| MAVLINK_DEFAULT.to_owned())
                .collect(),
            bind: BIND_DEFAULT.to_owned(),
            video: vec![VIDEO_DEFAULT.to_owned()],
            help: false,
            mavlink_was_set: false,
            video_was_set: false,
        };
        let mut args = args;
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--mavlink" => {
                    let value = args
                        .next()
                        .ok_or("--mavlink requires a connection string")?;
                    // Split --mavlink a,b,c and allow repeated flags.
                    for part in value.split(',') {
                        let part = part.trim().to_owned();
                        if !part.is_empty() {
                            if !options.mavlink_was_set {
                                options.mavlink.clear();
                                options.mavlink_was_set = true;
                            }
                            options.mavlink.push(part);
                        }
                    }
                }
                "--bind" => {
                    options.bind = args.next().ok_or("--bind requires a socket address")?;
                }
                "--video" => {
                    let value = args.next().ok_or("--video requires a UDP or RTSP URL")?;
                    // Split --video a,b,c and allow repeated flags (per-vehicle feeds).
                    for part in value.split(',') {
                        let part = part.trim().to_owned();
                        if !part.is_empty() {
                            if !options.video_was_set {
                                options.video.clear();
                                options.video_was_set = true;
                            }
                            options.video.push(part);
                        }
                    }
                }
                "--help" | "-h" => options.help = true,
                unknown => return Err(format!("unknown option: {unknown}")),
            }
        }
        Ok(options)
    }
}

pub async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let options = Options::parse(std::env::args().skip(1))?;
    if options.help {
        println!(
            "VYDE controller backend\n\
             Usage: controller [--mavlink CONNECTION[,CONNECTION...]] [--bind IP:PORT] [--video URL]\n\
             Defaults: --mavlink {MAVLINK_DEFAULT} --bind {BIND_DEFAULT} --video {VIDEO_DEFAULT}; set VYDE_VEHICLE_COUNT=N for N vehicles.\n\
             Multi-vehicle: repeat --mavlink or comma-separate endpoints; vehicle 1,2,... map to MAVLink system_id 1,2,...\n\
             Each vehicle must heartbeat before it is linked; commands to unlinked vehicles are rejected.\n\
             Video: repeat --video or comma-separate URLs; vehicle 1,2,... map to video feeds 1,2,... (default ports 5600, 5601, ... when --video is omitted).\n\
             The HTTP API and browser interface are served at http://{BIND_DEFAULT}.\n\
             Vehicle API: GET /api/vehicles, GET /api/v/:id/state, POST /api/v/:id/command, POST /api/v/:id/manual, POST /api/v/:id/stop, GET /api/v/:id/video.mjpg, GET /api/v/:id/video/status\n\
             Legacy single-vehicle API (/api/state etc., /api/video.mjpg) maps to vehicle 1."
        );
        return Ok(());
    }

    let bind: SocketAddr = options.bind.parse()?;
    let listener = tokio::net::TcpListener::bind(bind).await?;
    let webrtc_api = build_webrtc_api()?;
    let webrtc_peers = Arc::new(tokio::sync::Mutex::new(Vec::new()));
    let mut sessions = Vec::new();
    let mut workers = Vec::new();
    let mut video_processes = Vec::new();
    let mut webrtc_processes = Vec::new();
    let video_count = options.mavlink.len().max(1);
    for (index, mavlink) in options.mavlink.iter().enumerate() {
        let vehicle_id = system_id_for(index);
        let connection = connect::<MavMessage>(mavlink)?;
        let initial_snapshot = BackendSnapshot {
            vehicle_id,
            mavlink: mavlink.clone(),
            linked: false,
            last_seen_secs_ago: None,
            telemetry: TelemetrySnapshot::default(),
            control_mode: ControlMode::Hold.label(),
            manual_input: None,
            flight_tuning: FlightTuning::default(),
            status: "Waiting for MAVLink heartbeat.".to_owned(),
        };
        let snapshot = Arc::new(RwLock::new(initial_snapshot));
        let (sender, receiver) = mpsc::channel();
        let worker_snapshot = Arc::clone(&snapshot);
        let mavlink = mavlink.clone();
        workers.push(std::thread::spawn(move || {
            run_backend_worker(connection, receiver, worker_snapshot, mavlink, vehicle_id);
        }));
        // Per-vehicle video feed: explicit --video[i] if given, else derive
        // udp://0.0.0.0:{5600+i} from the base default so multi-drone works
        // with zero extra flags. Single-drone setups see only feed 0.
        let video = Arc::new(RwLock::new(None));
        let webrtc_track = Arc::new(TrackLocalStaticSample::new(
            RTCRtpCodecCapability {
                mime_type: "video/H264".to_owned(),
                clock_rate: 90_000,
                channels: 0,
                sdp_fmtp_line:
                    "level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=42e01f"
                        .to_owned(),
                rtcp_feedback: vec![],
            },
            format!("video-{vehicle_id}"),
            format!("vyde-{vehicle_id}"),
        ));
        let video_url = if options.video_was_set {
            options
                .video
                .get(index)
                .cloned()
                .unwrap_or_else(|| video_url_for_feed(VIDEO_DEFAULT, index))
        } else {
            video_url_for_feed(VIDEO_DEFAULT, index)
        };
        let video_process = match start_video_receiver(&video_url, Arc::clone(&video)) {
            Ok(process) => Some(process),
            Err(error) => {
                eprintln!(
                    "Livestream receiver unavailable for vehicle {vehicle_id} (ffmpeg): {error}"
                );
                None
            }
        };
        video_processes.push(video_process);
        let webrtc_process = match start_webrtc_receiver(&video_url, Arc::clone(&webrtc_track)) {
            Ok(process) => Some(process),
            Err(error) => {
                eprintln!(
                    "WebRTC H.264 receiver unavailable for vehicle {vehicle_id}: {error}"
                );
                None
            }
        };
        webrtc_processes.push(webrtc_process);
        sessions.push(VehicleSession {
            vehicle_id,
            sender,
            snapshot,
            video,
            webrtc_track,
        });
    }
    let _ = video_count;

    let app_state = AppState {
        sessions,
        webrtc_api,
        webrtc_peers,
    };
    let app = Router::new()
        .route("/", get(index))
        .route("/api/vehicles", get(api_vehicles))
        .route("/api/state", get(api_state_legacy))
        .route("/api/video/status", get(api_video_status_legacy))
        .route("/api/video.mjpg", get(api_video_legacy))
        .route("/api/command", post(api_command_legacy))
        .route("/api/manual", post(api_manual_legacy))
        .route("/api/stop", post(api_stop_legacy))
        .route("/api/v/{id}/state", get(api_state))
        .route("/api/v/{id}/command", post(api_command))
        .route("/api/v/{id}/manual", post(api_manual))
        .route("/api/v/{id}/stop", post(api_stop))
        .route("/api/v/{id}/video/status", get(api_video_status))
        .route("/api/v/{id}/video.mjpg", get(api_video))
        .route("/api/v/{id}/webrtc/offer", post(api_webrtc_offer))
        .with_state(Arc::new(app_state));

    eprintln!("Controller backend listening on http://{bind}");
    let server_result = axum::serve(listener, app)
        .with_graceful_shutdown(async {
            if let Err(error) = tokio::signal::ctrl_c().await {
                eprintln!("Failed to listen for shutdown signal: {error}");
            }
            eprintln!("Ctrl+C received, shutting down...");
        })
        .await;

    // Watchdog: if any join hangs (e.g. the ffmpeg reader), do not
    // leave the user trapped unable to exit with Ctrl+C.
    std::thread::spawn(|| {
        std::thread::sleep(Duration::from_secs(3));
        eprintln!("Shutdown timed out, forcing exit.");
        std::process::exit(1);
    });

    for process in video_processes {
        stop_video_receiver(process);
    }
    for process in webrtc_processes {
        stop_video_receiver(process);
    }
    for worker in workers {
        if worker.join().is_err() {
            return Err("MAVLink worker thread panicked".into());
        }
    }
    server_result?;
    Ok(())
}

async fn index() -> Html<&'static str> {
    Html(include_str!("../web/index.html"))
}

async fn api_state(
    State(state): State<Arc<AppState>>,
    Path(vehicle_id): Path<u8>,
) -> Result<Json<BackendSnapshot>, (StatusCode, String)> {
    let session = state.session(vehicle_id)?;
    let snapshot = session.snapshot.read().map_err(|_| {
        (
            StatusCode::INTERNAL_SERVER_ERROR,
            "State lock poisoned".to_owned(),
        )
    })?;
    Ok(Json(snapshot.clone()))
}

#[derive(Serialize)]
struct VehicleSummary {
    vehicle_id: u8,
    mavlink: String,
    linked: bool,
    last_seen_secs_ago: Option<u64>,
    status: String,
}

async fn api_vehicles(State(state): State<Arc<AppState>>) -> Json<Vec<VehicleSummary>> {
    Json(
        state
            .sessions
            .iter()
            .map(|session| {
                let snapshot = session
                    .snapshot
                    .read()
                    .map(|s| s.clone())
                    .unwrap_or_else(|_| BackendSnapshot {
                        vehicle_id: session.vehicle_id,
                        mavlink: String::new(),
                        linked: false,
                        last_seen_secs_ago: None,
                        telemetry: TelemetrySnapshot::default(),
                        control_mode: ControlMode::Hold.label(),
                        manual_input: None,
                        flight_tuning: FlightTuning::default(),
                        status: "State lock poisoned".to_owned(),
                    });
                VehicleSummary {
                    vehicle_id: snapshot.vehicle_id,
                    mavlink: snapshot.mavlink,
                    linked: snapshot.linked,
                    last_seen_secs_ago: snapshot.last_seen_secs_ago,
                    status: snapshot.status,
                }
            })
            .collect(),
    )
}

async fn api_state_legacy(
    state: State<Arc<AppState>>,
) -> Result<Json<BackendSnapshot>, (StatusCode, String)> {
    api_state(state, Path(1)).await
}

#[derive(Serialize)]
struct VideoStatus {
    live: bool,
}

async fn api_video_status(
    State(state): State<Arc<AppState>>,
    Path(vehicle_id): Path<u8>,
) -> Result<Json<VideoStatus>, (StatusCode, String)> {
    let session = state.session(vehicle_id)?;
    let frame = session.video.read().map_err(|_| {
        (
            StatusCode::INTERNAL_SERVER_ERROR,
            "Video state lock poisoned".to_owned(),
        )
    })?;
    Ok(Json(VideoStatus {
        live: frame
            .as_ref()
            .is_some_and(|frame| frame.received_at.elapsed() < Duration::from_secs(2)),
    }))
}

async fn api_video_status_legacy(
    state: State<Arc<AppState>>,
) -> Result<Json<VideoStatus>, (StatusCode, String)> {
    api_video_status(state, Path(1)).await
}

async fn api_video(State(state): State<Arc<AppState>>, Path(vehicle_id): Path<u8>) -> Response {
    let frames = match state.session(vehicle_id) {
        Ok(session) => Arc::clone(&session.video),
        Err(_) => Arc::new(RwLock::new(None)),
    };
    let frame_stream = stream::unfold(frames, |frames| async move {
        tokio::time::sleep(Duration::from_millis(100)).await;
        let frame = frames.read().ok().and_then(|frame| frame.clone());
        let chunk = if let Some(frame) = frame {
            let mut chunk = format!(
                "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n",
                frame.jpeg.len()
            )
            .into_bytes();
            chunk.extend_from_slice(&frame.jpeg);
            chunk.extend_from_slice(b"\r\n");
            chunk
        } else {
            b"\r\n".to_vec()
        };
        Some((
            Ok::<Bytes, std::convert::Infallible>(Bytes::from(chunk)),
            frames,
        ))
    });
    let mut response = Response::new(Body::from_stream(frame_stream));
    response.headers_mut().insert(
        header::CONTENT_TYPE,
        header::HeaderValue::from_static("multipart/x-mixed-replace; boundary=frame"),
    );
    response.headers_mut().insert(
        header::CACHE_CONTROL,
        header::HeaderValue::from_static("no-store"),
    );
    response
}

async fn api_video_legacy(state: State<Arc<AppState>>) -> Response {
    api_video(state, Path(1)).await
}

async fn api_webrtc_offer(
    State(state): State<Arc<AppState>>,
    Path(vehicle_id): Path<u8>,
    Json(offer): Json<RTCSessionDescription>,
) -> Result<Json<RTCSessionDescription>, (StatusCode, String)> {
    let session = state.session(vehicle_id)?;
    if offer.sdp_type != webrtc::peer_connection::sdp::sdp_type::RTCSdpType::Offer {
        return Err((StatusCode::BAD_REQUEST, "Expected an SDP offer.".to_owned()));
    }
    let peer = Arc::new(
        state
            .webrtc_api
            .new_peer_connection(RTCConfiguration::default())
            .await
            .map_err(webrtc_error)?,
    );
    peer.add_track(session.webrtc_track.clone() as Arc<dyn TrackLocal + Send + Sync>)
        .await
        .map_err(webrtc_error)?;
    peer.set_remote_description(offer).await.map_err(webrtc_error)?;
    let mut gathering = peer.gathering_complete_promise().await;
    let answer = peer.create_answer(None).await.map_err(webrtc_error)?;
    peer.set_local_description(answer).await.map_err(webrtc_error)?;
    let _ = gathering.recv().await;
    let answer = peer
        .local_description()
        .await
        .ok_or_else(|| (StatusCode::INTERNAL_SERVER_ERROR, "WebRTC answer was not created.".to_owned()))?;
    state.webrtc_peers.lock().await.push(peer);
    Ok(Json(answer))
}

fn webrtc_error(error: impl std::fmt::Display) -> (StatusCode, String) {
    (
        StatusCode::BAD_REQUEST,
        format!("WebRTC negotiation failed: {error}"),
    )
}

async fn api_command(
    State(state): State<Arc<AppState>>,
    Path(vehicle_id): Path<u8>,
    Json(request): Json<CommandRequest>,
) -> Result<Json<ApiResponse>, (StatusCode, String)> {
    let session = state.session(vehicle_id)?;
    let (reply, result) = oneshot::channel();
    session
        .sender
        .send(BackendMessage::Command {
            line: request.command,
            reply,
        })
        .map_err(|_| {
            (
                StatusCode::SERVICE_UNAVAILABLE,
                "Controller worker stopped".to_owned(),
            )
        })?;
    let message = result
        .await
        .map_err(|_| {
            (
                StatusCode::SERVICE_UNAVAILABLE,
                "Controller worker stopped".to_owned(),
            )
        })?
        .map_err(|error| (StatusCode::BAD_REQUEST, error))?;
    Ok(Json(ApiResponse { ok: true, message }))
}

async fn api_command_legacy(
    state: State<Arc<AppState>>,
    Json(request): Json<CommandRequest>,
) -> Result<Json<ApiResponse>, (StatusCode, String)> {
    api_command(state, Path(1), Json(request)).await
}

fn start_video_receiver(
    url: &str,
    latest: Arc<RwLock<Option<VideoFrame>>>,
) -> io::Result<VideoReceiverProcess> {
    if !url.starts_with("udp://") && !url.starts_with("rtsp://") {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "video URL must use udp:// or rtsp://",
        ));
    }

    let mut args = vec![
            "-nostdin",
            "-hide_banner",
            "-loglevel",
            "error",
            "-fflags",
            "nobuffer",
            "-flags",
            "low_delay",
            "-probesize",
            "32",
            "-analyzeduration",
            "0",
            "-i",
            url,
            "-an",
            "-vf",
            "fps=10,scale=640:-2",
            "-c:v",
            "mjpeg",
            "-q:v",
            "7",
            "-flush_packets",
            "1",
            "-f",
            "image2pipe",
            "pipe:1",
        ];
    if url.starts_with("udp://") {
            let input = args.iter().position(|arg| *arg == "-i").unwrap_or(0);
            args.splice(input..input, ["-f", "mpegts"]);
        }
    let mut child = Command::new("ffmpeg")
        .args(args)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()?;
    let stdout = child.stdout.take().ok_or_else(|| {
        io::Error::other("ffmpeg did not provide a stdout pipe for decoded frames")
    })?;
    let reader = std::thread::Builder::new()
        .name("vyde-video-decoder".to_owned())
        .spawn(move || read_jpeg_frames(stdout, latest))?;
    eprintln!("Livestream receiver listening to {url}");
    Ok(VideoReceiverProcess { child, reader })
}

fn read_jpeg_frames(mut reader: impl Read, latest: Arc<RwLock<Option<VideoFrame>>>) {
    let mut buffer = Vec::with_capacity(64 * 1024);
    let mut chunk = [0; 8192];
    loop {
        let count = match reader.read(&mut chunk) {
            Ok(0) => break,
            Ok(count) => count,
            Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
            Err(error) => {
                eprintln!("Livestream decoder read error: {error}");
                break;
            }

        };
        buffer.extend_from_slice(&chunk[..count]);
        loop {
            let Some(start) = buffer.windows(2).position(|bytes| bytes == [0xff, 0xd8]) else {
                buffer.clear();
                break;
            };
            if start > 0 {
                buffer.drain(..start);
            }
            let Some(end) = buffer.windows(2).position(|bytes| bytes == [0xff, 0xd9]) else {
                if buffer.len() > MAX_JPEG_BYTES {
                    buffer.clear();
                }
                break;
            };
            let jpeg: Arc<[u8]> = buffer.drain(..end + 2).collect::<Vec<_>>().into();
            if let Ok(mut frame) = latest.write() {
                *frame = Some(VideoFrame {
                    jpeg,
                    received_at: Instant::now(),
                });
            }
        }
    }
}

fn start_webrtc_receiver(
    url: &str,
    track: Arc<TrackLocalStaticSample>,
) -> io::Result<VideoReceiverProcess> {
    let mut args = vec![
        "-nostdin", "-hide_banner", "-loglevel", "error",
        "-fflags", "nobuffer", "-flags", "low_delay",
    ];
    if url.starts_with("udp://") {
        args.extend(["-f", "mpegts"]);
    }
    args.extend([
        "-i", url,
        "-an", "-c:v", "copy", "-bsf:v", "h264_mp4toannexb",
        "-f", "h264", "pipe:1",
    ]);
    let mut child = Command::new("ffmpeg")
        .args(args)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()?;
    let stdout = child.stdout.take()
        .ok_or_else(|| io::Error::other("ffmpeg did not provide H.264 stdout"))?;
    let handle = tokio::runtime::Handle::try_current()
        .map_err(|_| io::Error::other("WebRTC receiver requires a Tokio runtime"))?;
    let reader = std::thread::Builder::new()
        .name("vyde-webrtc-h264".to_owned())
        .spawn(move || read_h264_samples(stdout, track, handle))?;
    eprintln!("WebRTC H.264 receiver listening to {url}");
    Ok(VideoReceiverProcess { child, reader })
}

fn read_h264_samples(
    mut reader: impl Read,
    track: Arc<TrackLocalStaticSample>,
    handle: tokio::runtime::Handle,
) {
    let mut buffer = Vec::with_capacity(256 * 1024);
    let mut chunk = [0; 16 * 1024];
    loop {
        let count = match reader.read(&mut chunk) {
            Ok(0) => break,
            Ok(count) => count,
            Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
            Err(error) => {
                eprintln!("WebRTC H.264 receiver read error: {error}");
                break;
            }
        };
        buffer.extend_from_slice(&chunk[..count]);
        while let Some((start, next)) = h264_nal_bounds(&buffer) {
            if start > 0 {
                buffer.drain(..start);
            }
            let payload_start = next - start;
            let Some((next_start, _)) = h264_nal_bounds(&buffer[payload_start..]) else {
                break;
            };
            let nal_end = payload_start + next_start;
            let nal = buffer[payload_start..nal_end].to_vec();
            buffer.drain(..nal_end);
            let sample = Sample {
                data: MediaBytes::from(nal),
                duration: Duration::from_millis(33),
                ..Default::default()
            };
            let _ = handle.block_on(track.write_sample(&sample));
        }
        if buffer.len() > 2 * 1024 * 1024 {
            buffer.clear();
        }
    }
}

fn h264_nal_bounds(data: &[u8]) -> Option<(usize, usize)> {
    let start = data.windows(4).position(|w| w == [0, 0, 0, 1])
        .or_else(|| data.windows(3).position(|w| w == [0, 0, 1]))?;
    let prefix = if data[start..].starts_with(&[0, 0, 0, 1]) { 4 } else { 3 };
    Some((start, start + prefix))
}

fn stop_video_receiver(receiver: Option<VideoReceiverProcess>) {
    if let Some(mut receiver) = receiver {
        if let Err(error) = receiver.child.kill()
            && error.kind() != io::ErrorKind::InvalidInput
        {
            eprintln!("Failed to stop ffmpeg livestream receiver: {error}");
        }
        if let Err(error) = receiver.child.wait() {
            eprintln!("Failed to reap ffmpeg livestream receiver: {error}");
        }
        if receiver.reader.join().is_err() {
            eprintln!("Livestream decoder thread panicked.");
        }
    }
}

async fn api_manual(
    State(state): State<Arc<AppState>>,
    Path(vehicle_id): Path<u8>,
    Json(request): Json<ManualRequest>,
) -> Result<Json<ApiResponse>, (StatusCode, String)> {
    let session = state.session(vehicle_id)?;
    let (reply, result) = oneshot::channel();
    session
        .sender
        .send(BackendMessage::Manual {
            input: ManualInput {
                north: request.north,
                east: request.east,
                down: request.down,
                yaw_rate: request.yaw_rate,
            },
            reply,
        })
        .map_err(|_| {
            (
                StatusCode::SERVICE_UNAVAILABLE,
                "Controller worker stopped".to_owned(),
            )
        })?;
    let message = result
        .await
        .map_err(|_| {
            (
                StatusCode::SERVICE_UNAVAILABLE,
                "Controller worker stopped".to_owned(),
            )
        })?
        .map_err(|error| (StatusCode::BAD_REQUEST, error))?;
    Ok(Json(ApiResponse { ok: true, message }))
}

async fn api_manual_legacy(
    state: State<Arc<AppState>>,
    Json(request): Json<ManualRequest>,
) -> Result<Json<ApiResponse>, (StatusCode, String)> {
    api_manual(state, Path(1), Json(request)).await
}

async fn api_stop(
    State(state): State<Arc<AppState>>,
    Path(vehicle_id): Path<u8>,
) -> Result<Json<ApiResponse>, (StatusCode, String)> {
    let session = state.session(vehicle_id)?;
    let (reply, result) = oneshot::channel();
    session
        .sender
        .send(BackendMessage::Stop { reply })
        .map_err(|_| {
            (
                StatusCode::SERVICE_UNAVAILABLE,
                "Controller worker stopped".to_owned(),
            )
        })?;
    let message = result
        .await
        .map_err(|_| {
            (
                StatusCode::SERVICE_UNAVAILABLE,
                "Controller worker stopped".to_owned(),
            )
        })?
        .map_err(|error| (StatusCode::INTERNAL_SERVER_ERROR, error))?;
    Ok(Json(ApiResponse { ok: true, message }))
}

async fn api_stop_legacy(
    state: State<Arc<AppState>>,
) -> Result<Json<ApiResponse>, (StatusCode, String)> {
    api_stop(state, Path(1)).await
}

fn run_backend_worker(
    mut connection: mavlink::Connection<MavMessage>,
    receiver: mpsc::Receiver<BackendMessage>,
    snapshot: Arc<RwLock<BackendSnapshot>>,
    mavlink: String,
    vehicle_id: u8,
) {
    let mut sequence = 0_u8;
    let mut last_heartbeat = Instant::now() - Duration::from_secs(2);
    let mut last_manual_setpoint = Instant::now() - Duration::from_secs(1);
    let mut manual_input = None;
    let mut manual_deadline = None;
    let mut telemetry = TelemetrySnapshot::default();
    let mut flight_tuning = FlightTuning::default();
    let mut control_mode = ControlMode::Hold;
    let mut status = "Waiting for MAVLink heartbeat.".to_owned();
    let mut linked = false;
    let mut last_seen: Option<Instant> = None;
    let mut stopping = false;

    while !stopping {
        loop {
            match receiver.try_recv() {
                Ok(BackendMessage::Command { line, reply }) => {
                    manual_input = None;
                    manual_deadline = None;
                    control_mode = ControlMode::Hold;
                    if let Err(error) = send_velocity(
                        &mut connection,
                        &mut sequence,
                        vehicle_id,
                        ManualInput::default(),
                    ) {
                        status = format!("Failed to release manual setpoint: {error}");
                        let _ = reply.send(Err(status.clone()));
                        continue;
                    }
                    let previous_status = status.clone();
                    match handle_command(
                        &line,
                        &mut connection,
                        &mut sequence,
                        vehicle_id,
                        &mut manual_input,
                        &mut control_mode,
                        &mut flight_tuning,
                        &mut status,
                    ) {
                        Ok(()) => {
                            if status == previous_status {
                                status = format!("Command accepted: {line}");
                            }
                            let _ = reply.send(Ok(status.clone()));
                        }
                        Err(error) => {
                            status = error.to_string();
                            let _ = reply.send(Err(status.clone()));
                        }
                    }
                    last_manual_setpoint = Instant::now();
                }
                Ok(BackendMessage::Manual { input, reply }) => {
                    match validate_manual_input(input, flight_tuning.max_speed_m_s) {
                        Ok(()) => {
                            if input.is_active() {
                                manual_input = Some(input);
                                manual_deadline = Some(Instant::now() + MANUAL_LEASE);
                                control_mode = ControlMode::Manual;
                                let _ = reply.send(Ok("Manual input updated.".to_owned()));
                            } else {
                                manual_input = None;
                                manual_deadline = None;
                                control_mode = ControlMode::Hold;
                                let result = send_velocity(
                                    &mut connection,
                                    &mut sequence,
                                    vehicle_id,
                                    ManualInput::default(),
                                )
                                .map(|()| "Manual input released; holding position.".to_owned())
                                .map_err(|error| {
                                    format!("Failed to release manual input: {error}")
                                });
                                match &result {
                                    Ok(message) => status = message.clone(),
                                    Err(error) => status = error.clone(),
                                }
                                let _ = reply.send(result);
                                last_manual_setpoint = Instant::now();
                            }
                        }
                        Err(error) => {
                            let _ = reply.send(Err(error));
                        }
                    }
                }
                Ok(BackendMessage::Stop { reply }) => {
                    manual_input = None;
                    manual_deadline = None;
                    control_mode = ControlMode::Hold;
                    let result = send_velocity(
                        &mut connection,
                        &mut sequence,
                        vehicle_id,
                        ManualInput::default(),
                    )
                    .map(|()| "Manual input stopped; holding position.".to_owned())
                    .map_err(|error| format!("Failed to stop manual input: {error}"));
                    if let Ok(message) = &result {
                        status = message.clone();
                    } else if let Err(error) = &result {
                        status = error.clone();
                    }
                    let _ = reply.send(result);
                    last_manual_setpoint = Instant::now();
                }
                Err(mpsc::TryRecvError::Empty) => break,
                Err(mpsc::TryRecvError::Disconnected) => {
                    stopping = true;
                    break;
                }
            }
        }

        if stopping {
            break;
        }
        if manual_deadline.is_some_and(|deadline| Instant::now() >= deadline) {
            manual_deadline = None;
            manual_input = None;
            control_mode = ControlMode::Hold;
            match send_velocity(
                &mut connection,
                &mut sequence,
                vehicle_id,
                ManualInput::default(),
            ) {
                Ok(()) => status = "Manual input lease expired; holding position.".to_owned(),
                Err(error) => status = format!("Failed to stop expired manual input: {error}"),
            }
            last_manual_setpoint = Instant::now();
        }

        if last_heartbeat.elapsed() >= Duration::from_secs(1) {
            if let Err(error) = send_heartbeat(&mut connection, &mut sequence) {
                status = format!("MAVLink heartbeat send error: {error}");
            }
            last_heartbeat = Instant::now();
        }
        match connection.try_recv() {
            Ok((_, message)) => {
                if matches!(message, MavMessage::HEARTBEAT(_)) {
                    linked = true;
                    last_seen = Some(Instant::now());
                    if status == "Waiting for MAVLink heartbeat." {
                        status = "Linked via MAVLink heartbeat.".to_owned();
                    }
                }
                update_telemetry(message, &mut telemetry, &mut flight_tuning, &mut status)
            }
            Err(mavlink::error::MessageReadError::Io(error))
                if error.kind() == io::ErrorKind::WouldBlock => {}
            Err(error) => status = format!("MAVLink receive error: {error}"),
        }
        if last_seen.is_some_and(|t| t.elapsed() >= Duration::from_secs(3)) {
            linked = false;
            status = "Link lost (no heartbeat for 3s).".to_owned();
        }

        if let Some(input) = manual_input
            && last_manual_setpoint.elapsed() >= Duration::from_millis(100)
        {
            // The web input is heading-relative (W=forward, D=right);
            // it is rotated into NED with the telemetry yaw before sending.
            let yaw_rad = telemetry
                .attitude_deg
                .map(|att| att[2].to_radians())
                .unwrap_or(0.0);
            let (north, east) = body_to_ned(input.north, input.east, yaw_rad);
            let rotated = ManualInput {
                north,
                east,
                ..input
            };
            if let Err(error) = send_velocity(&mut connection, &mut sequence, vehicle_id, rotated) {
                status = format!("MAVLink manual setpoint error: {error}");
            }
            last_manual_setpoint = Instant::now();
        }

        match snapshot.write() {
            Ok(mut current) => {
                *current = BackendSnapshot {
                    vehicle_id,
                    mavlink: mavlink.clone(),
                    linked,
                    last_seen_secs_ago: last_seen.map(|t| t.elapsed().as_secs()),
                    telemetry: telemetry.clone(),
                    control_mode: control_mode.label(),
                    manual_input,
                    flight_tuning,
                    status: status.clone(),
                };
            }
            Err(_) => {
                eprintln!("Controller state lock poisoned; stopping backend.");
                break;
            }
        }
        std::thread::sleep(Duration::from_millis(5));
    }

    if let Err(error) = send_velocity(
        &mut connection,
        &mut sequence,
        vehicle_id,
        ManualInput::default(),
    ) {
        eprintln!("Failed to release final manual setpoint: {error}");
    }
}

fn validate_manual_input(input: ManualInput, max_speed_m_s: f32) -> Result<(), String> {
    if !input.north.is_finite()
        || !input.east.is_finite()
        || !input.down.is_finite()
        || !input.yaw_rate.is_finite()
    {
        return Err("Manual input values must be finite numbers.".to_owned());
    }
    let max_speed = max_speed_m_s.max(0.1);
    if input.north.hypot(input.east) > max_speed + 0.001 {
        return Err(format!(
            "Horizontal input must not exceed {max_speed:.2} m/s."
        ));
    }
    if input.down.abs() > 2.0 {
        return Err("Vertical input must be between -2 and 2 m/s.".to_owned());
    }
    if input.yaw_rate.abs() > 1.5 {
        return Err("Yaw rate must be between -1.5 and 1.5 rad/s.".to_owned());
    }
    Ok(())
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
    vehicle_id: u8,
    command: COMMAND_LONG_DATA,
) -> Result<(), mavlink::error::MessageWriteError> {
    connection
        .send(
            &header(sequence),
            &MavMessage::COMMAND_LONG(command_long_with_target(vehicle_id, command)),
        )
        .map(|_| ())
}

fn command_long_with_target(vehicle_id: u8, mut command: COMMAND_LONG_DATA) -> COMMAND_LONG_DATA {
    command.target_system = vehicle_id;
    command
}

fn handle_command(
    line: &str,
    connection: &mut mavlink::Connection<MavMessage>,
    sequence: &mut u8,
    vehicle_id: u8,
    manual_input: &mut Option<ManualInput>,
    control_mode: &mut ControlMode,
    flight_tuning: &mut FlightTuning,
    status: &mut String,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut parts = line.split_whitespace();
    let Some(command) = parts.next() else {
        return Err(io::Error::new(io::ErrorKind::InvalidInput, "Command is empty").into());
    };
    match command {
        "quit" | "exit" => {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "The backend stays available; stop it with Ctrl+C.",
            )
            .into());
        }
        "arm" | "disarm" => {
            let arm = command == "arm";
            if arm {
                send_velocity(connection, sequence, vehicle_id, ManualInput::default())?;
            }
            send_command(
                connection,
                sequence,
                vehicle_id,
                command_long(
                    MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                    [if arm { 1.0 } else { 0.0 }, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
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
                vehicle_id,
                command_long(
                    MavCmd::MAV_CMD_NAV_TAKEOFF,
                    [0.0, 0.0, 0.0, f32::NAN, 0.0, 0.0, altitude],
                ),
            )?;
            *manual_input = None;
            *control_mode = ControlMode::Guided;
        }
        "land" => {
            send_command(
                connection,
                sequence,
                vehicle_id,
                command_long(
                    MavCmd::MAV_CMD_NAV_LAND,
                    [0.0, 0.0, 0.0, f32::NAN, 0.0, 0.0, 0.0],
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
            send_command(
                connection,
                sequence,
                vehicle_id,
                command_long(
                    MavCmd::MAV_CMD_DO_REPOSITION,
                    [-1.0, 0.0, 0.0, f32::NAN, latitude, longitude, altitude],
                ),
            )?;
            *manual_input = None;
            *control_mode = ControlMode::Guided;
        }
        "manual" => {
            let input = parse_manual_input(parts.collect(), flight_tuning.max_speed_m_s)
                .map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error))?;
            send_velocity(connection, sequence, vehicle_id, input)?;
            *manual_input = input.is_active().then_some(input);
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
            let (parameter, label, valid) = match command {
                "speed" => ("VYDE_MAX_SPEED", "max speed", value > 0.0),
                "accel" => ("VYDE_MAX_ACCEL", "max acceleration", value > 0.0),
                _ => ("VYDE_MAX_TILT", "max tilt", (0.0..=90.0).contains(&value)),
            };
            if !valid {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!("{label} must be a positive finite number"),
                )
                .into());
            }
            send_flight_parameter(connection, sequence, vehicle_id, parameter, value)?;
            match command {
                "speed" => flight_tuning.max_speed_m_s = value,
                "accel" => flight_tuning.max_acceleration_m_s2 = value,
                _ => flight_tuning.max_tilt_deg = value,
            }
            *status = format!("{label} set to {value:.1}");
        }
        "hold" | "stop" => {
            send_velocity(connection, sequence, vehicle_id, ManualInput::default())?;
            *manual_input = None;
            *control_mode = ControlMode::Hold;
        }
        "rtl" => {
            send_command(
                connection,
                sequence,
                vehicle_id,
                command_long(MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH, [0.0; 7]),
            )?;
            *manual_input = None;
            *control_mode = ControlMode::Guided;
        }
        unknown => {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("Unknown command: {unknown}"),
            )
            .into());
        }
    }
    Ok(())
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
            "yaw-left" | "q" => input.yaw_rate = 1.5,
            "yaw-right" | "e" => input.yaw_rate = -1.5,
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
    vehicle_id: u8,
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
                target_system: vehicle_id,
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
    vehicle_id: u8,
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
                target_system: vehicle_id,
                target_component: TARGET_COMPONENT,
                coordinate_frame: MavFrame::MAV_FRAME_LOCAL_NED,
                ..Default::default()
            }),
        )
        .map(|_| ())
}

fn command_long(command: MavCmd, params: [f32; 7]) -> COMMAND_LONG_DATA {
    COMMAND_LONG_DATA {
        param1: params[0],
        param2: params[1],
        param3: params[2],
        param4: params[3],
        param5: params[4],
        param6: params[5],
        param7: params[6],
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
                    "VYDE_MAX_ACCEL" => flight_tuning.max_acceleration_m_s2 = data.param_value,
                    "VYDE_MAX_TILT" => flight_tuning.max_tilt_deg = data.param_value,
                    _ => return,
                }
                *status = format!("Simulator confirmed {name} = {:.1}", data.param_value);
            }
        }
        _ => {}
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
        assert_eq!(input.yaw_rate, 1.5);
        assert_eq!(parse_manual_input(vec!["a"], 3.0).unwrap().east, 3.0);
        assert_eq!(parse_manual_input(vec!["d"], 3.0).unwrap().east, -3.0);
        assert_eq!(
            parse_manual_input(vec!["stop"], 3.0).unwrap(),
            ManualInput::default()
        );
    }

    #[test]
    fn manual_api_input_enforces_finite_values_and_limits() {
        assert!(
            validate_manual_input(
                ManualInput {
                    north: 3.0 / 2.0_f32.sqrt(),
                    east: 3.0 / 2.0_f32.sqrt(),
                    ..Default::default()
                },
                3.0,
            )
            .is_ok()
        );
        assert!(
            validate_manual_input(
                ManualInput {
                    north: 3.1,
                    ..Default::default()
                },
                3.0,
            )
            .is_err()
        );
        assert!(
            validate_manual_input(
                ManualInput {
                    down: f32::NAN,
                    ..Default::default()
                },
                3.0,
            )
            .is_err()
        );
    }

    #[test]
    fn manual_api_updates_and_releases_velocity_setpoints() {
        let probe = UdpSocket::bind("127.0.0.1:0").unwrap();
        let port = probe.local_addr().unwrap().port();
        drop(probe);
        let receiver_connection = connect::<MavMessage>(&format!("udpin:0.0.0.0:{port}")).unwrap();
        let controller_connection =
            connect::<MavMessage>(&format!("udpout:127.0.0.1:{port}")).unwrap();
        let snapshot = Arc::new(RwLock::new(BackendSnapshot {
            vehicle_id: 1,
            mavlink: MAVLINK_DEFAULT.to_owned(),
            linked: true,
            last_seen_secs_ago: Some(0),
            telemetry: TelemetrySnapshot::default(),
            control_mode: ControlMode::Hold.label(),
            manual_input: None,
            flight_tuning: FlightTuning::default(),
            status: String::new(),
        }));
        let (sender, receiver) = mpsc::channel();
        let worker_snapshot = Arc::clone(&snapshot);
        let worker = std::thread::spawn(move || {
            run_backend_worker(
                controller_connection,
                receiver,
                worker_snapshot,
                MAVLINK_DEFAULT.to_owned(),
                1,
            );
        });

        let (reply, result) = oneshot::channel();
        sender
            .send(BackendMessage::Manual {
                input: ManualInput {
                    north: 1.0,
                    ..Default::default()
                },
                reply,
            })
            .unwrap();
        result.blocking_recv().unwrap().unwrap();
        let active = receive_velocity_setpoint(&receiver_connection);
        assert_eq!(active.vx, 1.0);

        let (reply, result) = oneshot::channel();
        sender
            .send(BackendMessage::Manual {
                input: ManualInput::default(),
                reply,
            })
            .unwrap();
        result.blocking_recv().unwrap().unwrap();
        let stopped = receive_velocity_setpoint(&receiver_connection);
        assert_eq!(stopped.vx, 0.0);
        assert_eq!(stopped.vy, 0.0);
        assert_eq!(stopped.vz, 0.0);

        let (reply, result) = oneshot::channel();
        sender
            .send(BackendMessage::Manual {
                input: ManualInput {
                    north: 0.5,
                    ..Default::default()
                },
                reply,
            })
            .unwrap();
        result.blocking_recv().unwrap().unwrap();
        assert_eq!(receive_velocity_setpoint(&receiver_connection).vx, 0.5);
        std::thread::sleep(MANUAL_LEASE + Duration::from_millis(100));
        let deadline = Instant::now() + Duration::from_secs(2);
        let mut lease_expired = false;
        while Instant::now() < deadline {
            match receiver_connection.try_recv() {
                Ok((_, MavMessage::SET_POSITION_TARGET_LOCAL_NED(data))) if data.vx == 0.0 => {
                    lease_expired = true;
                    break;
                }
                Ok(_) => {}
                Err(mavlink::error::MessageReadError::Io(error))
                    if error.kind() == io::ErrorKind::WouldBlock =>
                {
                    std::thread::sleep(Duration::from_millis(5));
                }
                Err(error) => panic!("controller failed while waiting for lease expiry: {error}"),
            }
        }
        assert!(lease_expired, "manual lease did not stop the vehicle");
        drop(sender);
        worker.join().unwrap();
    }

    #[test]
    fn rejects_invalid_guided_coordinates() {
        assert!(validate_coordinates(37.4, -122.1, 10.0).is_ok());
        assert!(validate_coordinates(91.0, 0.0, 10.0).is_err());
        assert!(validate_coordinates(0.0, f32::NAN, 10.0).is_err());
        assert!(validate_coordinates(0.0, 0.0, -1.0).is_err());
    }

    #[test]
    fn body_to_ned_rotates_with_heading() {
        // yaw=0 (facing North): forward=north, right=east
        let (n, e) = body_to_ned(1.0, 0.0, 0.0);
        assert!((n - 1.0).abs() < 1e-6 && e.abs() < 1e-6);
        let (n, e) = body_to_ned(0.0, 1.0, 0.0);
        assert!(n.abs() < 1e-6 && (e - 1.0).abs() < 1e-6);
        // yaw=90° (facing East): forward=east, right=south
        let (n, e) = body_to_ned(1.0, 0.0, std::f32::consts::FRAC_PI_2);
        assert!(n.abs() < 1e-6 && (e - 1.0).abs() < 1e-6);
        let (n, e) = body_to_ned(0.0, 1.0, std::f32::consts::FRAC_PI_2);
        assert!((n + 1.0).abs() < 1e-6 && e.abs() < 1e-6);
        // yaw=180° (facing South): forward=south (before, W went "backward")
        let (n, e) = body_to_ned(1.0, 0.0, std::f32::consts::PI);
        assert!((n + 1.0).abs() < 1e-6 && e.abs() < 1e-6);
    }

    #[test]
    fn options_default_to_local_backend_and_simulator() {
        let options = Options::parse(std::iter::empty()).unwrap();
        assert_eq!(options.mavlink, vec![MAVLINK_DEFAULT.to_owned()]);
        assert_eq!(options.bind, BIND_DEFAULT);
        assert_eq!(options.video, vec![VIDEO_DEFAULT.to_owned()]);
    }

    #[test]
    fn jpeg_reader_extracts_complete_frames_from_chunked_stream() {
        let latest = Arc::new(RwLock::new(None));
        let bytes = b"junk\xff\xd8first\xff\xd9\xff\xd8second\xff\xd9trailing";
        read_jpeg_frames(&bytes[..], Arc::clone(&latest));
        let frame = latest.read().unwrap().clone().unwrap();
        assert_eq!(&*frame.jpeg, b"\xff\xd8second\xff\xd9");
    }

    #[test]
    fn speed_command_sends_mavlink_parameter_set() {
        let probe = UdpSocket::bind("127.0.0.1:0").unwrap();
        let port = probe.local_addr().unwrap().port();
        drop(probe);

        let server = connect::<MavMessage>(&format!("udpin:0.0.0.0:{port}")).unwrap();
        let mut controller = connect::<MavMessage>(&format!("udpout:127.0.0.1:{port}")).unwrap();
        let mut sequence = 0;
        let mut manual_input = None;
        let mut control_mode = ControlMode::Hold;
        let mut flight_tuning = FlightTuning::default();
        let mut status = String::new();

        handle_command(
            "speed 5",
            &mut controller,
            &mut sequence,
            1,
            &mut manual_input,
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

    fn receive_velocity_setpoint(
        connection: &mavlink::Connection<MavMessage>,
    ) -> SET_POSITION_TARGET_LOCAL_NED_DATA {
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            match connection.try_recv() {
                Ok((_, MavMessage::SET_POSITION_TARGET_LOCAL_NED(data))) => return data,
                Ok(_) => {}
                Err(mavlink::error::MessageReadError::Io(error))
                    if error.kind() == io::ErrorKind::WouldBlock && Instant::now() < deadline =>
                {
                    std::thread::sleep(Duration::from_millis(5));
                }
                Err(error) => panic!("controller did not send velocity setpoint: {error}"),
            }
            assert!(
                Instant::now() < deadline,
                "controller did not send velocity setpoint"
            );
        }
    }
}
