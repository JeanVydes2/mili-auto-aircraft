//! A bounded, simulated digital video link carrying actual GPU camera pixels.
//! Bitrate/packetization and decoder dependencies are modeled, not a real codec.
use crate::camera::{CameraFrameBuffers, FrameCamera};
use crate::config::SensorConfig;
use crate::domain_randomization::ResetGeneration;
use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::gpu_readback::{ReadbackComplete, ReadbackOnce};
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use rand::{Rng, SeedableRng, rngs::StdRng};
use std::collections::VecDeque;
use std::io::{self, Write};
use std::process::{Command, Stdio};
use std::sync::mpsc::{self, SyncSender, TrySendError};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VideoProfile {
    Clean,
    Nominal,
    Poor,
    Lost,
}

impl VideoProfile {
    /// Parses `clean|nominal|poor|lost` (case-insensitive); `None` if unknown.
    pub fn from_name(name: &str) -> Option<Self> {
        match name.to_ascii_lowercase().as_str() {
            "clean" => Some(Self::Clean),
            "nominal" => Some(Self::Nominal),
            "poor" => Some(Self::Poor),
            "lost" => Some(Self::Lost),
            _ => None,
        }
    }
}

#[derive(Resource, Clone)]
pub struct VideoLinkConfig {
    pub profile: VideoProfile,
    pub latency_ms: f32,
    pub jitter_ms: f32,
    /// Independent simulated packet loss; a frame spans multiple 1200-byte packets.
    pub packet_loss: f32,
    pub corruption: f32,
    pub interference: f32,
    pub bandwidth_kbps: f32,
    pub bitrate_kbps: f32,
    pub buffer_ms: f32,
    pub max_frames: usize,
    pub range_m: f32,
    pub keyframe_interval: u64,
    pub ground_station: Vec3,
}

impl VideoLinkConfig {
    pub fn for_profile(profile: VideoProfile) -> Self {
        let mut config = Self {
            profile,
            latency_ms: 95.0,
            jitter_ms: 25.0,
            packet_loss: 0.002,
            corruption: 0.025,
            interference: 0.08,
            bandwidth_kbps: 2600.0,
            bitrate_kbps: 1600.0,
            buffer_ms: 65.0,
            max_frames: 48,
            range_m: 160.0,
            keyframe_interval: 30,
            ground_station: Vec3::new(-3.0, 1.0, -3.0),
        };
        match profile {
            VideoProfile::Clean => {
                config.latency_ms = 0.0;
                config.jitter_ms = 0.0;
                config.packet_loss = 0.0;
                config.corruption = 0.0;
                config.interference = 0.0;
                config.buffer_ms = 0.0;
                config.bandwidth_kbps = 1_000_000.0;
                config.range_m = 1_000_000.0;
            }
            VideoProfile::Nominal => {}
            VideoProfile::Poor => {
                config.latency_ms = 190.0;
                config.jitter_ms = 100.0;
                config.packet_loss = 0.035;
                config.corruption = 0.18;
                config.interference = 0.5;
                config.bandwidth_kbps = 1300.0;
                config.buffer_ms = 150.0;
                config.range_m = 65.0;
            }
            VideoProfile::Lost => {
                config.bandwidth_kbps = 0.0;
            }
        }
        config
    }
}

impl Default for VideoLinkConfig {
    fn default() -> Self {
        // VYDE_VIDEO_PROFILE=clean|nominal|poor|lost (e.g. in remote
        // production, where there is no keyboard to press V). Without the
        // variable, the default profile is Nominal and is changed live with V.
        let profile = std::env::var("VYDE_VIDEO_PROFILE")
            .ok()
            .and_then(|name| VideoProfile::from_name(&name))
            .unwrap_or(VideoProfile::Nominal);
        Self::for_profile(profile)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinkState {
    Connecting,
    Buffering,
    Live,
    Degraded,
    Frozen,
    Lost,
}

#[derive(Resource)]
pub struct StreamStats {
    pub state: LinkState,
    pub signal_quality: f32,
    pub rssi_dbm: f32,
    pub distance_m: f32,
    pub latency_ms: f32,
    pub jitter_ms: f32,
    pub buffer_ms: f32,
    pub queue_frames: usize,
    pub rx_fps: f32,
    pub bitrate_kbps: f32,
    pub lost_frames: u64,
    pub received_frames: u64,
    pub dropped_frames: u64,
    pub frame_age_ms: f32,
    pub corrupted_frames: u64,
    pub profile: VideoProfile,
}

impl Default for StreamStats {
    fn default() -> Self {
        Self {
            state: LinkState::Connecting,
            signal_quality: 1.0,
            rssi_dbm: -40.0,
            distance_m: 0.0,
            latency_ms: 0.0,
            jitter_ms: 0.0,
            buffer_ms: 0.0,
            queue_frames: 0,
            rx_fps: 0.0,
            bitrate_kbps: 0.0,
            lost_frames: 0,
            received_frames: 0,
            dropped_frames: 0,
            frame_age_ms: 0.0,
            corrupted_frames: 0,
            profile: VideoProfile::Nominal,
        }
    }
}

#[derive(Resource)]
pub struct StreamDisplay {
    pub image: Handle<Image>,
    pub width: u32,
    pub height: u32,
}

/// Per-vehicle receiver displays (the image shown in the ground-station UI).
/// Index == `VehicleIndex`. Vehicle 0 mirrors the legacy `StreamDisplay`.
#[derive(Resource, Default)]
pub struct StreamDisplays {
    pub displays: Vec<StreamDisplay>,
}

#[derive(Resource, Default)]
pub struct ReceiverView(pub bool);

#[derive(Resource, Default)]
pub struct VideoOutput {
    // Index 0 is the legacy single feed (vehicle 1); additional entries are
    // per-vehicle feeds on successive UDP ports (see `video_output_url`).
    senders: Vec<Option<SyncSender<Vec<u8>>>>,
    disconnected: Vec<bool>,
}

impl VideoOutput {
    /// Number of encoder feeds started (1 = single drone legacy behavior).
    pub fn feed_count(&self) -> usize {
        self.senders.len()
    }

    fn try_send(&mut self, feed: usize, pixels: Vec<u8>) {
        let Some(sender) = self.senders.get(feed).and_then(Option::as_ref) else {
            return;
        };
        match sender.try_send(pixels) {
            Ok(()) | Err(TrySendError::Full(_)) => {}
            Err(TrySendError::Disconnected(_)) => {
                let already = self.disconnected.get(feed).copied().unwrap_or(true);
                if !already {
                    bevy::log::error!("FPV encoder worker {feed} disconnected");
                    if let Some(flag) = self.disconnected.get_mut(feed) {
                        *flag = true;
                    }
                }
            }
        }
    }
}

/// Resolve the ffmpeg output URL for a video feed.
/// Feed 0 keeps the legacy `VYDE_VIDEO_OUTPUT` / `udp://127.0.0.1:5600` default
/// so single-drone setups are unchanged. Feeds 1.. get `...:5601`, `...:5602`, ...
/// by bumping the port. Non-UDP URLs (e.g. RTSP) and explicit per-feed vars
/// (`VYDE_VIDEO_OUTPUT_2`, ...) are used verbatim.
fn video_output_url(feed: usize) -> String {
    if feed > 0 {
        if let Ok(url) = std::env::var(format!("VYDE_VIDEO_OUTPUT_{}", feed + 1)) {
            return url;
        }
    }
    let base = std::env::var("VYDE_VIDEO_OUTPUT")
        .unwrap_or_else(|_| "udp://127.0.0.1:5600?pkt_size=1316".to_owned());
    if feed == 0 {
        return base;
    }
    if let Some(rest) = base.strip_prefix("udp://") {
        let (host_port, suffix) = match rest.split_once('?') {
            Some((hp, tail)) => (hp, Some(tail)),
            None => (rest, None),
        };
        if let Some((host, port)) = host_port.rsplit_once(':') {
            if let Ok(port_num) = port.parse::<u16>() {
                let url = format!(
                    "udp://{}:{}{}",
                    host,
                    port_num + feed as u16,
                    suffix.map_or_else(String::new, |tail| format!("?{tail}"))
                );
                return url;
            }
        }
    }
    base
}

#[derive(Component)]
pub struct CaptureRequest {
    captured_at: f64,
    sequence: u64,
    epoch: u64,
    generation: u64,
    width: u32,
    height: u32,
    vehicle: usize,
}

struct VideoFrame {
    sequence: u64,
    captured_at: f64,
    present_at: f64,
    pixels: Option<Vec<u8>>,
    corrupted: bool,
    encoded_bits: f64,
}

#[derive(Resource)]
pub struct VideoReceiver {
    queue: Vec<VideoFrame>,
    rng: StdRng,
    next_capture: f64,
    sequence: u64,
    epoch: u64,
    generation: u64,
    profile: VideoProfile,
    next_transmission: f64,
    decoded_sequence: Option<u64>,
    decoder_synced: bool,
    last_pixels: Option<Vec<u8>>,
    last_capture: Option<f64>,
    last_paint: f64,
    previous_latency: Option<f32>,
    samples: VecDeque<(f64, f64)>,
}

/// Per-vehicle video receivers. Index == `VehicleIndex`.
/// Vehicle 0 is the legacy single receiver.
#[derive(Resource, Default)]
pub struct VideoReceivers {
    pub receivers: Vec<VideoReceiver>,
}

impl VideoReceivers {
    pub fn get_mut(&mut self, vehicle: usize) -> Option<&mut VideoReceiver> {
        self.receivers.get_mut(vehicle)
    }
    pub fn get(&self, vehicle: usize) -> Option<&VideoReceiver> {
        self.receivers.get(vehicle)
    }
    pub fn len(&self) -> usize {
        self.receivers.len()
    }
    pub fn is_empty(&self) -> bool {
        self.receivers.is_empty()
    }
}

/// Per-vehicle stream stats. Index == `VehicleIndex`.
#[derive(Resource, Default)]
pub struct StreamStatsList {
    pub stats: Vec<StreamStats>,
}

impl StreamStatsList {
    pub fn get_mut(&mut self, vehicle: usize) -> Option<&mut StreamStats> {
        self.stats.get_mut(vehicle)
    }
    pub fn get(&self, vehicle: usize) -> Option<&StreamStats> {
        self.stats.get(vehicle)
    }
    pub fn primary(&self) -> Option<&StreamStats> {
        self.stats.first()
    }
}

impl Default for VideoReceiver {
    fn default() -> Self {
        Self {
            queue: Vec::new(),
            rng: StdRng::seed_from_u64(0x56594445),
            next_capture: 0.0,
            sequence: 0,
            epoch: 0,
            generation: 0,
            profile: VideoProfile::Nominal,
            next_transmission: 0.0,
            decoded_sequence: None,
            decoder_synced: false,
            last_pixels: None,
            last_capture: None,
            last_paint: -1.0,
            previous_latency: None,
            samples: VecDeque::new(),
        }
    }
}

impl VideoReceiver {
    fn reset(&mut self, generation: u64, profile: VideoProfile) {
        let epoch = self.epoch.wrapping_add(1);
        *self = Self {
            epoch,
            generation,
            profile,
            ..default()
        };
    }

    fn new_for_vehicle(vehicle: usize) -> Self {
        // Distinct RNG seed per vehicle so link impairments differ.
        let mut receiver = Self::default();
        receiver.rng =
            StdRng::seed_from_u64(0x56594445 ^ (vehicle as u64).wrapping_mul(0x9E3779B97F4A7C15));
        receiver
    }

    fn enqueue(
        &mut self,
        pixels: Vec<u8>,
        sequence: u64,
        captured_at: f64,
        now: f64,
        fps: f32,
        config: &VideoLinkConfig,
        stats: &mut StreamStats,
    ) {
        let capacity = config.max_frames.clamp(1, 90);
        let quality = stats.signal_quality.clamp(0.0, 1.0);
        let bandwidth = config.bandwidth_kbps.max(0.0) * quality;
        if bandwidth < 1.0 || config.profile == VideoProfile::Lost {
            stats.lost_frames += 1;
            return;
        }
        let bits = config.bitrate_kbps.max(1.0) as f64 * 1000.0 / fps.max(1.0) as f64;
        let packet_count = (bits / (1200.0 * 8.0)).ceil().max(1.0);
        let packet_loss = (config.packet_loss + (1.0 - quality) * 0.07).clamp(0.0, 1.0);
        let frame_loss = 1.0 - (1.0 - packet_loss as f64).powf(packet_count);
        let lost = self.rng.gen_bool(frame_loss.clamp(0.0, 1.0));
        let corrupted = !lost
            && self
                .rng
                .gen_bool((config.corruption + (1.0 - quality) * 0.2).clamp(0.0, 1.0) as f64);
        self.next_transmission =
            self.next_transmission.max(now) + bits / (bandwidth as f64 * 1000.0);
        let jitter = self
            .rng
            .gen_range(-config.jitter_ms.max(0.0)..=config.jitter_ms.max(0.0))
            as f64
            / 1000.0;
        let present_at = self.next_transmission
            + (config.latency_ms as f64 / 1000.0 + jitter).max(0.0)
            + config.buffer_ms.max(0.0) as f64 / 1000.0;
        if lost {
            stats.lost_frames += 1;
        }
        self.queue.push(VideoFrame {
            sequence,
            captured_at,
            present_at,
            pixels: if lost { None } else { Some(pixels) },
            corrupted,
            encoded_bits: bits,
        });
        // Keep the newest bounded window rather than accumulate unbounded RAM.
        while self.queue.len() > capacity {
            self.queue.remove(0);
            stats.dropped_frames += 1;
            self.decoder_synced = false;
        }
        // Serialization backlog must also be bounded. Discard the backlog and
        // wait for a new keyframe if the link cannot carry this bitrate.
        if self.next_transmission - now > 2.0 {
            stats.dropped_frames += self.queue.len() as u64;
            self.queue.clear();
            self.next_transmission = now;
            self.decoder_synced = false;
        }
    }

    fn advance(&mut self, now: f64, config: &VideoLinkConfig, stats: &mut StreamStats) {
        self.queue
            .sort_by(|a, b| a.present_at.total_cmp(&b.present_at));
        let ready = self.queue.partition_point(|frame| frame.present_at <= now);
        let frames: Vec<_> = self.queue.drain(..ready).collect();
        for frame in frames {
            if self
                .decoded_sequence
                .is_some_and(|last| frame.sequence <= last)
            {
                stats.dropped_frames += 1;
                continue;
            }
            let keyframe = frame.sequence % config.keyframe_interval.max(1) == 0;
            let gap = self
                .decoded_sequence
                .is_some_and(|last| frame.sequence != last + 1);
            if gap {
                self.decoder_synced = false;
            }
            self.decoded_sequence = Some(frame.sequence);
            let Some(mut pixels) = frame.pixels else {
                self.decoder_synced = false;
                continue;
            };
            if keyframe {
                self.decoder_synced = true;
            }
            if !self.decoder_synced {
                stats.dropped_frames += 1;
                continue;
            }
            if frame.corrupted {
                stats.corrupted_frames += 1;
                // Block damage is applied during presentation, using the old image
                // as reference where possible, like concealment in a video decoder.
                if let Some(old) = &self.last_pixels {
                    let block = (pixels.len() / 12).max(4);
                    let start = self
                        .rng
                        .gen_range(0..pixels.len().saturating_sub(block).max(1))
                        / 4
                        * 4;
                    let end = (start + block).min(pixels.len()).min(old.len());
                    if start < end {
                        pixels[start..end].copy_from_slice(&old[start..end]);
                    }
                }
                for pixel in pixels.chunks_exact_mut(4).step_by(3) {
                    pixel[0] &= 0xe0;
                    pixel[1] &= 0xe0;
                    pixel[2] &= 0xe0;
                }
            }
            let latency = ((now - frame.captured_at).max(0.0) * 1000.0) as f32;
            if let Some(previous) = self.previous_latency {
                stats.jitter_ms += ((latency - previous).abs() - stats.jitter_ms) * 0.1;
            }
            self.previous_latency = Some(latency);
            stats.latency_ms = latency;
            stats.received_frames += 1;
            self.samples.push_back((now, frame.encoded_bits));
            self.last_capture = Some(frame.captured_at);
            self.last_pixels = Some(pixels);
        }
        while self
            .samples
            .front()
            .is_some_and(|sample| now - sample.0 > 1.0)
        {
            self.samples.pop_front();
        }
        stats.rx_fps = self.samples.len() as f32;
        stats.bitrate_kbps =
            self.samples.iter().map(|sample| sample.1).sum::<f64>() as f32 / 1000.0;
        stats.queue_frames = self.queue.len();
        stats.buffer_ms = self
            .queue
            .iter()
            .map(|frame| (frame.present_at - now).max(0.0))
            .fold(0.0_f64, f64::max) as f32
            * 1000.0;
        stats.frame_age_ms = self
            .last_capture
            .map_or(0.0, |capture| ((now - capture).max(0.0) * 1000.0) as f32);
        stats.state = if config.profile == VideoProfile::Lost || stats.signal_quality < 0.04 {
            LinkState::Lost
        } else if self.last_capture.is_none() {
            if self.queue.is_empty() {
                LinkState::Connecting
            } else {
                LinkState::Buffering
            }
        } else if stats.frame_age_ms > 2000.0 {
            LinkState::Lost
        } else if stats.frame_age_ms > 500.0 || !self.decoder_synced {
            LinkState::Frozen
        } else if stats.signal_quality < 0.65 || config.profile == VideoProfile::Poor {
            LinkState::Degraded
        } else {
            LinkState::Live
        };
    }
}

pub fn setup_stream(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    buffers: Res<CameraFrameBuffers>,
    sensor: Res<SensorConfig>,
    mut output: ResMut<VideoOutput>,
) {
    let feed_count = buffers.len().max(1);
    let width = buffers.buffers.first().map_or(640, |b| b.width);
    let height = buffers.buffers.first().map_or(360, |b| b.height);
    let mut displays = Vec::with_capacity(feed_count);
    for feed in 0..feed_count {
        let pixels = vec![12; width as usize * height as usize * 4];
        let mut image = Image::new_fill(
            Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
            TextureDimension::D2,
            &[8, 14, 22, 255],
            TextureFormat::Rgba8UnormSrgb,
            RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
        );
        // Keep the CPU copy: only this receiver image is uploaded, never the source.
        image.data = Some(pixels);
        displays.push(StreamDisplay {
            image: images.add(image),
            width,
            height,
        });
        match start_encoder(width, height, sensor.camera_fps, feed) {
            Ok(sender) => {
                output.senders.push(Some(sender));
                output.disconnected.push(false);
            }
            Err(error) => bevy::log::error!("FPV encoder {feed} is unavailable: {error}"),
        }
    }
    // Legacy single display mirrors vehicle 0.
    if let Some(first) = displays.first() {
        commands.insert_resource(StreamDisplay {
            image: first.image.clone(),
            width: first.width,
            height: first.height,
        });
    }
    commands.insert_resource(StreamDisplays { displays });
    commands.insert_resource(VideoReceivers {
        receivers: (0..feed_count)
            .map(VideoReceiver::new_for_vehicle)
            .collect(),
    });
    commands.insert_resource(StreamStatsList {
        stats: (0..feed_count).map(|_| StreamStats::default()).collect(),
    });
}

fn start_encoder(
    width: u32,
    height: u32,
    fps: f32,
    feed: usize,
) -> io::Result<SyncSender<Vec<u8>>> {
    let url = video_output_url(feed);
    if !url.starts_with("udp://") && !url.starts_with("rtsp://") {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "VYDE_VIDEO_OUTPUT must use udp:// or rtsp://",
        ));
    }
    let codec = std::env::var("VYDE_VIDEO_CODEC").unwrap_or_else(|_| "h264".to_owned());
    let encoder = match codec.as_str() {
        "h264" => "libx264",
        "h265" => "libx265",
        _ => {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "VYDE_VIDEO_CODEC must be h264 or h265",
            ));
        }
    };
    let mut args = vec![
        "-hide_banner".to_owned(),
        "-loglevel".to_owned(),
        "error".to_owned(),
        "-f".to_owned(),
        "rawvideo".to_owned(),
        "-pixel_format".to_owned(),
        "rgba".to_owned(),
        "-video_size".to_owned(),
        format!("{width}x{height}"),
        "-framerate".to_owned(),
        format!("{}", fps.clamp(1.0, 60.0)),
        "-i".to_owned(),
        "pipe:0".to_owned(),
        "-an".to_owned(),
        "-c:v".to_owned(),
        encoder.to_owned(),
        "-preset".to_owned(),
        "ultrafast".to_owned(),
        "-tune".to_owned(),
        "zerolatency".to_owned(),
        "-g".to_owned(),
        "30".to_owned(),
    ];
    if url.starts_with("rtsp://") {
        args.extend([
            "-f".to_owned(),
            "rtsp".to_owned(),
            "-rtsp_transport".to_owned(),
            "tcp".to_owned(),
        ]);
    } else {
        args.extend(["-f".to_owned(), "mpegts".to_owned()]);
    }
    args.push(url.clone());

    let mut child = Command::new("ffmpeg")
        .args(args)
        .stdin(Stdio::piped())
        .stdout(Stdio::null())
        .stderr(Stdio::inherit())
        .spawn()?;
    let mut stdin = child.stdin.take().ok_or_else(|| {
        io::Error::other("ffmpeg did not provide a stdin pipe for raw camera frames")
    })?;
    let (sender, receiver) = mpsc::sync_channel::<Vec<u8>>(2);
    std::thread::Builder::new()
        .name("vyde-fpv-encoder".to_owned())
        .spawn(move || {
            while let Ok(frame) = receiver.recv() {
                if let Err(error) = stdin.write_all(&frame) {
                    bevy::log::error!("FPV encoder stopped accepting frames: {error}");
                    break;
                }
            }
            let _ = child.kill();
            if let Err(error) = child.wait() {
                bevy::log::warn!("Could not reap FPV encoder process: {error}");
            }
        })?;
    bevy::log::info!("Software FPV encoder ({codec}) publishing to {url}");
    Ok(sender)
}

pub fn video_controls(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut config: ResMut<VideoLinkConfig>,
    mut view: ResMut<ReceiverView>,
) {
    if keyboard.just_pressed(KeyCode::Tab) {
        view.0 = !view.0;
    }
    if keyboard.just_pressed(KeyCode::KeyV) {
        let profile = match config.profile {
            VideoProfile::Clean => VideoProfile::Nominal,
            VideoProfile::Nominal => VideoProfile::Poor,
            VideoProfile::Poor => VideoProfile::Lost,
            VideoProfile::Lost => VideoProfile::Clean,
        };
        *config = VideoLinkConfig::for_profile(profile);
        bevy::log::info!(
            "Video link profile changed to {profile:?}; the encoded controller feed uses this profile"
        );
    }
}

pub fn capture_video(
    mut commands: Commands,
    time: Res<Time>,
    sensor: Res<SensorConfig>,
    buffers: Res<CameraFrameBuffers>,
    config: Res<VideoLinkConfig>,
    generation: Res<ResetGeneration>,
    mut receivers: ResMut<VideoReceivers>,
    mut stats_list: ResMut<StreamStatsList>,
    mut cameras: Query<(&mut Camera, &FrameCamera)>,
    requests: Query<(Entity, &CaptureRequest)>,
) {
    let now = time.elapsed_secs_f64();
    // Reset any receiver whose generation/profile changed.
    for (vehicle, receiver) in receivers.receivers.iter_mut().enumerate() {
        if generation.0 != receiver.generation || config.profile != receiver.profile {
            receiver.reset(generation.0, config.profile);
            if let Some(stats) = stats_list.get_mut(vehicle) {
                *stats = StreamStats {
                    profile: config.profile,
                    ..default()
                };
            }
        }
    }
    for (entity, request) in &requests {
        if now - request.captured_at > 2.0 {
            commands.entity(entity).despawn();
        }
    }
    // Group cameras by vehicle; each vehicle captures from its own buffer.
    let pending_per_vehicle: std::collections::HashMap<usize, usize> = requests
        .iter()
        .map(|(_, r)| r.vehicle)
        .fold(std::collections::HashMap::new(), |mut map, v| {
            *map.entry(v).or_default() += 1;
            map
        });
    let mut due_vehicles: Vec<usize> = Vec::new();
    for (vehicle, receiver) in receivers.receivers.iter().enumerate() {
        let pending = pending_per_vehicle.get(&vehicle).copied().unwrap_or(0);
        if now >= receiver.next_capture && pending < 3 {
            due_vehicles.push(vehicle);
        }
    }
    for (mut camera, frame_camera) in &mut cameras {
        camera.is_active = due_vehicles.contains(&frame_camera.vehicle);
    }
    if due_vehicles.is_empty() {
        return;
    }
    let interval = 1.0 / sensor.camera_fps.clamp(1.0, 60.0) as f64;
    for vehicle in due_vehicles {
        let Some(buffer) = buffers.get(vehicle) else {
            continue;
        };
        let Some(receiver) = receivers.get_mut(vehicle) else {
            continue;
        };
        receiver.next_capture = now + interval;
        let sequence = receiver.sequence;
        receiver.sequence = receiver.sequence.wrapping_add(1);
        commands.spawn((
            ReadbackOnce::texture(buffer.image.clone()),
            CaptureRequest {
                captured_at: now,
                sequence,
                epoch: receiver.epoch,
                generation: generation.0,
                width: buffer.width,
                height: buffer.height,
                vehicle,
            },
        ));
    }
}

/// WGPU aligns each row to 256 bytes; strip padding instead of skewing the image.
fn unpack_rgba(data: &[u8], width: u32, height: u32) -> Option<Vec<u8>> {
    let row = width as usize * 4;
    let stride = row.div_ceil(256) * 256;
    if row == 0 || height == 0 || data.len() < stride * height as usize {
        return None;
    }
    let mut pixels = Vec::with_capacity(row * height as usize);
    for line in data.chunks(stride).take(height as usize) {
        pixels.extend_from_slice(&line[..row]);
    }
    Some(pixels)
}

pub fn receive_gpu_frame(
    event: On<ReadbackComplete>,
    mut commands: Commands,
    requests: Query<&CaptureRequest>,
    generation: Res<ResetGeneration>,
    time: Res<Time>,
    sensor: Res<SensorConfig>,
    config: Res<VideoLinkConfig>,
    mut receivers: ResMut<VideoReceivers>,
    mut stats_list: ResMut<StreamStatsList>,
) {
    let Ok(request) = requests.get(event.entity) else {
        return;
    };
    commands.entity(event.entity).despawn();
    let Some(receiver) = receivers.get_mut(request.vehicle) else {
        return;
    };
    if request.epoch != receiver.epoch
        || request.generation != generation.0
        || config.profile != receiver.profile
    {
        return;
    }
    let Some(stats) = stats_list.get_mut(request.vehicle) else {
        return;
    };
    if let Some(pixels) = unpack_rgba(&event.data, request.width, request.height) {
        receiver.enqueue(
            pixels,
            request.sequence,
            request.captured_at,
            time.elapsed_secs_f64(),
            sensor.camera_fps.clamp(1.0, 60.0),
            &config,
            stats,
        );
    }
}

fn interference_pixels(
    pixels: &mut [u8],
    width: usize,
    time: f64,
    amount: f32,
    lost: bool,
    rng: &mut StdRng,
) {
    let amount = amount.clamp(0.0, 1.0);
    if amount == 0.0 && !lost {
        return;
    }
    let band = ((time * 67.0) as usize) % (pixels.len() / (width.max(1) * 4)).max(1);
    for (index, pixel) in pixels.chunks_exact_mut(4).enumerate() {
        let y = index / width.max(1);
        let noise = rng.gen_range(-1.0_f32..1.0);
        if lost {
            let value = (40.0 + noise * 32.0).clamp(0.0, 255.0) as u8;
            pixel[..3].fill(value);
        } else {
            let scan = if y % 3 == 0 { 1.0 - amount * 0.22 } else { 1.0 };
            let sync = if y.abs_diff(band) < 3 {
                -55.0 * amount
            } else {
                0.0
            };
            for channel in &mut pixel[..3] {
                *channel =
                    (*channel as f32 * scan + noise * amount * 45.0 + sync).clamp(0.0, 255.0) as u8;
            }
        }
        pixel[3] = 255;
    }
}

pub fn present_video(
    time: Res<Time>,
    config: Res<VideoLinkConfig>,
    displays: Res<StreamDisplays>,
    drone: Query<
        (&Transform, Option<&crate::aircraft::VehicleIndex>),
        With<crate::aircraft::Aircraft>,
    >,
    mut receivers: ResMut<VideoReceivers>,
    mut stats_list: ResMut<StreamStatsList>,
    mut images: ResMut<Assets<Image>>,
    mut video_output: ResMut<VideoOutput>,
) {
    let now = time.elapsed_secs_f64();
    // Distance per vehicle for signal quality.
    let mut distances: std::collections::HashMap<usize, f32> = std::collections::HashMap::new();
    for (tf, vehicle_index) in &drone {
        let vehicle = vehicle_index.map(|v| v.0).unwrap_or(0);
        distances.insert(vehicle, tf.translation.distance(config.ground_station));
    }
    let interference =
        config.interference.max(0.0) * (0.55 + 0.45 * (time.elapsed_secs() * 1.7).sin());
    // Collect presented pixels per vehicle first to avoid borrow conflicts.
    let mut presented: Vec<(usize, Vec<u8>)> = Vec::new();
    for vehicle in 0..receivers.len() {
        let Some(stats) = stats_list.get_mut(vehicle) else {
            continue;
        };
        let Some(receiver) = receivers.get_mut(vehicle) else {
            continue;
        };
        let display = displays.displays.get(vehicle);
        let (width, height) = display.map_or((640, 360), |d| (d.width, d.height));
        stats.distance_m = distances.get(&vehicle).copied().unwrap_or(0.0);
        let distance_loss = (stats.distance_m / config.range_m.max(1.0)).powi(2);
        stats.signal_quality = if config.profile == VideoProfile::Lost {
            0.0
        } else {
            (1.0 / (1.0 + distance_loss) - interference * 0.4).clamp(0.0, 1.0)
        };
        stats.rssi_dbm = -40.0 - (1.0 - stats.signal_quality) * 60.0;
        stats.profile = config.profile;
        receiver.advance(now, &config, stats);
        if now - receiver.last_paint < 1.0 / 30.0 {
            continue;
        }
        receiver.last_paint = now;
        let mut pixels = receiver
            .last_pixels
            .clone()
            .unwrap_or_else(|| vec![12; width as usize * height as usize * 4]);
        let noise = if config.profile == VideoProfile::Clean {
            0.0
        } else {
            (interference + (1.0 - stats.signal_quality) * 0.6).clamp(0.0, 1.0)
        };
        interference_pixels(
            &mut pixels,
            width as usize,
            now,
            noise,
            stats.state == LinkState::Lost,
            &mut receiver.rng,
        );
        // Always write opaque alpha, including startup and reset placeholders.
        for pixel in pixels.chunks_exact_mut(4) {
            pixel[3] = 255;
        }
        presented.push((vehicle, pixels));
    }
    for (vehicle, pixels) in presented {
        video_output.try_send(vehicle, pixels.clone());
        if let Some(display) = displays.displays.get(vehicle) {
            if let Some(mut image) = images.get_mut(&display.image) {
                image.data = Some(pixels);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn video_profile_names_parse_case_insensitively() {
        assert_eq!(VideoProfile::from_name("clean"), Some(VideoProfile::Clean));
        assert_eq!(VideoProfile::from_name("CLEAN"), Some(VideoProfile::Clean));
        assert_eq!(VideoProfile::from_name("Poor"), Some(VideoProfile::Poor));
        assert_eq!(VideoProfile::from_name("lost"), Some(VideoProfile::Lost));
        assert_eq!(VideoProfile::from_name("otro"), None);
    }

    #[test]
    fn video_hotkey_cycles_the_link_profile_used_by_the_encoded_feed() {
        let mut app = App::new();
        app.init_resource::<ButtonInput<KeyCode>>();
        app.init_resource::<VideoLinkConfig>();
        app.init_resource::<ReceiverView>();
        app.add_systems(Update, video_controls);

        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::KeyV);
        app.update();
        assert_eq!(
            app.world().resource::<VideoLinkConfig>().profile,
            VideoProfile::Poor
        );

        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .release(KeyCode::KeyV);
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .clear();
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::KeyV);
        app.update();
        assert_eq!(
            app.world().resource::<VideoLinkConfig>().profile,
            VideoProfile::Lost
        );
    }

    fn perfect() -> VideoLinkConfig {
        VideoLinkConfig {
            latency_ms: 100.0,
            buffer_ms: 50.0,
            bandwidth_kbps: 1000.0,
            bitrate_kbps: 100.0,
            jitter_ms: 0.0,
            packet_loss: 0.0,
            corruption: 0.0,
            keyframe_interval: 3,
            ..VideoLinkConfig::for_profile(VideoProfile::Clean)
        }
    }

    #[test]
    fn image_pixels_are_delayed_by_transport_and_playout_buffer() {
        let mut receiver = VideoReceiver::default();
        let mut stats = StreamStats::default();
        let config = perfect();
        receiver.enqueue(vec![77; 16], 0, 0.0, 0.0, 10.0, &config, &mut stats);
        receiver.advance(0.159, &config, &mut stats);
        assert!(receiver.last_pixels.is_none());
        assert_eq!(stats.state, LinkState::Buffering);
        receiver.advance(0.161, &config, &mut stats);
        assert_eq!(receiver.last_pixels.as_deref(), Some([77; 16].as_slice()));
        assert!(stats.latency_ms >= 160.0);
    }

    #[test]
    fn decoder_freezes_after_loss_and_recovers_only_on_keyframe() {
        let mut receiver = VideoReceiver::default();
        let mut stats = StreamStats::default();
        let config = perfect();
        for sequence in [0, 2, 3] {
            let t = sequence as f64 * 0.1;
            receiver.enqueue(
                vec![sequence as u8; 16],
                sequence,
                t,
                t,
                10.0,
                &config,
                &mut stats,
            );
            receiver.advance(t + 0.2, &config, &mut stats);
            if sequence == 2 {
                assert_eq!(receiver.last_pixels.as_ref().unwrap()[0], 0);
                assert_eq!(stats.state, LinkState::Frozen);
            }
        }
        assert_eq!(receiver.last_pixels.as_ref().unwrap()[0], 3);
        assert_eq!(stats.received_frames, 2);
    }

    #[test]
    fn limited_bandwidth_and_queue_never_grow_unbounded() {
        let mut receiver = VideoReceiver::default();
        let mut stats = StreamStats::default();
        let config = VideoLinkConfig {
            bandwidth_kbps: 300.0,
            bitrate_kbps: 1000.0,
            max_frames: 4,
            ..perfect()
        };
        for sequence in 0..300 {
            let now = sequence as f64 / 30.0;
            receiver.enqueue(vec![0; 16], sequence, now, now, 30.0, &config, &mut stats);
            assert!(receiver.queue.len() <= 4);
            assert!(receiver.next_transmission - now <= 2.0);
        }
        assert!(stats.dropped_frames > 0);
    }

    #[test]
    fn late_frames_never_rewind_receiver() {
        let mut receiver = VideoReceiver::default();
        let mut stats = StreamStats::default();
        let config = VideoLinkConfig {
            keyframe_interval: 1,
            ..perfect()
        };
        receiver.enqueue(vec![3; 16], 3, 0.0, 0.0, 10.0, &config, &mut stats);
        receiver.advance(0.2, &config, &mut stats);
        receiver.enqueue(vec![2; 16], 2, 0.0, 0.2, 10.0, &config, &mut stats);
        receiver.advance(0.4, &config, &mut stats);
        assert_eq!(receiver.last_pixels.as_ref().unwrap()[0], 3);
    }

    #[test]
    fn reset_discards_old_video_and_requires_fresh_keyframe() {
        let mut receiver = VideoReceiver::default();
        let mut stats = StreamStats::default();
        let config = perfect();
        receiver.enqueue(vec![1; 16], 0, 0.0, 0.0, 10.0, &config, &mut stats);
        receiver.reset(1, VideoProfile::Poor);
        assert!(receiver.queue.is_empty());
        assert!(receiver.last_pixels.is_none());
        assert!(!receiver.decoder_synced);
        assert_eq!(receiver.epoch, 1);
        assert_eq!(receiver.generation, 1);
    }

    #[test]
    fn strips_gpu_row_padding_and_rejects_truncated_data() {
        let mut bytes = vec![99; 512];
        bytes[..12].fill(1);
        bytes[256..268].fill(2);
        let rgba = unpack_rgba(&bytes, 3, 2).unwrap();
        assert_eq!(&rgba[..12], &[1; 12]);
        assert_eq!(&rgba[12..], &[2; 12]);
        assert!(unpack_rgba(&bytes[..300], 3, 2).is_none());
    }

    #[test]
    fn clean_pixels_are_unchanged_and_interference_changes_actual_pixels() {
        let mut rng = StdRng::seed_from_u64(1);
        let original = vec![128; 64];
        let mut pixels = original.clone();
        interference_pixels(&mut pixels, 4, 1.0, 0.0, false, &mut rng);
        assert_eq!(pixels, original);
        interference_pixels(&mut pixels, 4, 1.0, 0.8, false, &mut rng);
        assert_ne!(pixels, original);
        assert!(pixels.chunks_exact(4).all(|pixel| pixel[3] == 255));
    }
}
