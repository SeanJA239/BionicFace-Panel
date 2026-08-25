use std::collections::BTreeMap;
use std::fs::OpenOptions;
use std::io::{BufWriter, Write};
use std::net::{SocketAddr, UdpSocket};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{anyhow, bail, Context, Result};
use chrono::{DateTime, Utc};
use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};
use serde::{Deserialize, Serialize};
use tokio::sync::Mutex;
use tracing::{info, warn};

const MOTOR_COUNT: usize = 32;
const HEARTBEAT_HZ: u64 = 100;
const DEFAULT_EXTERNAL_INPUT_PORT: u16 = 6100;
const DEFAULT_EXTERNAL_INPUT_TIMEOUT_MS: u64 = 500;
// Smoothing factor for the external input fps estimate (EMA over inter-frame
// gaps), not a hard averaging window, so a stalled source's last reading
// simply stops updating rather than needing a ring buffer.
const EXTERNAL_FPS_EMA_ALPHA: f32 = 0.2;
// When all channels are settled the PCA9685 holds its PWM output, so frames
// are only resent as a low-rate keepalive instead of at the full tick rate.
const KEEPALIVE_TICK_DIVISOR: u64 = 10;
const LOG_FLUSH_INTERVAL: Duration = Duration::from_secs(1);
const MAX_STEP_PER_TICK_DEG: f32 = 2.0;
const CONFIG_PATH: &str = "config/motor_config.json";

// Nod action: neck motors 30/31 are a mirror pair. Motor 30 lifts up by
// increasing its angle; motor 31 mirrors it. Amplitude stays within the
// conservative neck limits, so a nod is ~±15 deg around neutral.
const NECK_UP_MOTOR: usize = 30;
const NECK_MIRROR_MOTOR: usize = 31;
// Neck motors 30/31 are symmetric (75..105, neutral 90), so a full ±1 norm
// swing reproduces the original ±15deg nod exactly, and stays proportional if
// the safety limits are ever recalibrated.
const NOD_AMPLITUDE_NORM: f32 = 1.0;
const NOD_CYCLES: usize = 2;
const NOD_PHASE_DWELL: Duration = Duration::from_millis(300);

// Wink holds the wink pose briefly, then returns to the center-all neutral.
const WINK_HOLD: Duration = Duration::from_millis(500);
const WINK_PRESET_ID: &str = "wink";

// Presets no longer snap directly to their target; this is the eased
// transition applied on top of the always-on rate limiter (step_towards),
// which remains the safety floor and is never bypassed or loosened.
const DEFAULT_PRESET_TRANSITION: Duration = Duration::from_millis(600);
const DEFAULT_PRESET_EASING: EasingKind = EasingKind::MinJerk;
const SEQUENCES_DIR: &str = "sequences";

// Idle blink: quick close/hold/open on the eyelid channels. Fixed durations
// per spec (not part of IdleBehaviorConfig, unlike the noise parameters).
const BLINK_CLOSE_DURATION: Duration = Duration::from_millis(80);
const BLINK_HOLD_DURATION: Duration = Duration::from_millis(60);
const BLINK_OPEN_DURATION: Duration = Duration::from_millis(120);
const BLINK_EYELID_CHANNELS: [usize; 4] = [9, 10, 11, 12];
// `+1` norm is "closed" for three of the four eyelid channels; channel 11
// (eye_right_upper) is mounted so its rotation direction is mirrored, so
// its closed end is `-1`. Confirmed on real hardware during idle-blink
// observation. Channel 12 (eye_right_lower) is left at `+1` for now --
// unconfirmed whether it needs the same mirroring, revisit if it looks
// wrong during idle blink too.
const BLINK_CLOSED_NORM: f32 = 1.0;
const BLINK_CLOSE_DIRECTIONS: [(usize, f32); 4] = [(9, 1.0), (10, 1.0), (11, -1.0), (12, 1.0)];

fn blink_close_direction(channel_id: usize) -> f32 {
    BLINK_CLOSE_DIRECTIONS
        .iter()
        .find(|(id, _)| *id == channel_id)
        .map(|(_, dir)| *dir)
        .unwrap_or(1.0)
}
// Idle noise's per-tick exponential smoothing rate towards its current
// random retarget value, tuned so it settles within roughly one retarget
// period rather than snapping.
const NOISE_SMOOTHING_PER_TICK: f32 = 0.03;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MotorChannel {
    pub id: usize,
    pub name: String,
    pub board: usize,
    pub channel: usize,
    pub board_address: u16,
    pub min_applied: f32,
    pub max_applied: f32,
    pub offset: f32,
    pub min_logical: f32,
    pub max_logical: f32,
    pub neutral_applied: f32,
    pub neutral_logical: f32,
    pub enabled: bool,
}

impl MotorChannel {
    fn logical_to_applied(&self, logical: f32) -> f32 {
        if !self.enabled {
            return self.neutral_applied;
        }
        let logical = logical.clamp(self.min_logical, self.max_logical);
        (logical + self.offset).clamp(self.min_applied, self.max_applied)
    }

    fn normalized_logical(&self, logical: f32) -> f32 {
        if !self.enabled {
            self.neutral_logical
        } else {
            logical.clamp(self.min_logical, self.max_logical)
        }
    }

    /// Map a bipolar normalized value in `[-1, 1]` to a physical applied angle.
    /// `0` -> neutral, `+1` -> `max_applied`, `-1` -> `min_applied`. The mapping
    /// is piecewise-linear about the neutral so both end anchors stay exact even
    /// when the neutral is off-center. A side with zero span (neutral pinned to a
    /// limit) simply produces no motion in that direction.
    ///
    fn norm_to_applied(&self, norm: f32) -> f32 {
        let norm = norm.clamp(-1.0, 1.0);
        let applied = if norm >= 0.0 {
            self.neutral_applied + norm * (self.max_applied - self.neutral_applied)
        } else {
            self.neutral_applied + norm * (self.neutral_applied - self.min_applied)
        };
        applied.clamp(self.min_applied, self.max_applied)
    }

    /// Inverse of [`norm_to_applied`]: map a physical applied angle to a bipolar
    /// normalized value in `[-1, 1]`. A zero-span side maps to `0` so there is no
    /// division by zero when the neutral is pinned to a limit.
    fn applied_to_norm(&self, applied: f32) -> f32 {
        let applied = applied.clamp(self.min_applied, self.max_applied);
        let norm = if applied >= self.neutral_applied {
            let span = self.max_applied - self.neutral_applied;
            if span <= f32::EPSILON {
                0.0
            } else {
                (applied - self.neutral_applied) / span
            }
        } else {
            let span = self.neutral_applied - self.min_applied;
            if span <= f32::EPSILON {
                0.0
            } else {
                (applied - self.neutral_applied) / span
            }
        };
        norm.clamp(-1.0, 1.0)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
struct ConfigFile {
    transport: TransportConfig,
    channels: Vec<MotorChannel>,
    jaw_coupling: Option<JawCouplingConfig>,
    expression_presets: Vec<ExpressionPreset>,
    #[serde(default)]
    external_input: Option<ExternalInputConfig>,
    #[serde(default)]
    idle_behavior: IdleBehaviorConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
struct TransportConfig {
    host: String,
    port: u16,
    board_addresses: Vec<u16>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
struct JawCouplingConfig {
    master_motor_id: usize,
    // Signed ratio per slave in normalized (-1..1) space: slave_norm = master_norm * ratio.
    // A ratio of -1.0 mirrors the master's full travel onto the slave's full travel,
    // regardless of how their physical degree ranges compare.
    slave_ratios: BTreeMap<usize, f32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ExternalInputConfig {
    #[serde(default = "default_external_input_port")]
    pub port: u16,
    #[serde(default = "default_external_input_timeout_ms")]
    pub timeout_ms: u64,
}

fn default_external_input_port() -> u16 {
    DEFAULT_EXTERNAL_INPUT_PORT
}

fn default_external_input_timeout_ms() -> u64 {
    DEFAULT_EXTERNAL_INPUT_TIMEOUT_MS
}

impl Default for ExternalInputConfig {
    fn default() -> Self {
        Self {
            port: DEFAULT_EXTERNAL_INPUT_PORT,
            timeout_ms: DEFAULT_EXTERNAL_INPUT_TIMEOUT_MS,
        }
    }
}

/// Arbitration between the three command sources described in the README's
/// "控制源仲裁": `Manual` (sliders/presets/sequences), `External` (the
/// task-4 UDP coefficient stream), and `Idle` (task-6 idle noise/blink,
/// entered automatically from Manual once targets are stationary and no
/// automated motion is running; any manual write exits it immediately).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum ControlSource {
    #[default]
    Manual,
    External,
    Idle,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ExternalInputStatus {
    pub port: u16,
    pub active: bool,
    pub last_seq: Option<u64>,
    pub fps: f32,
    pub timeout_ms: u64,
    pub control_source: ControlSource,
}

/// Idle-behavior tuning, exported from config.py's `IDLE_BEHAVIOR` block
/// into motor_config.json (see raspi/export_config_json.py). Every field has
/// a serde default matching the task spec's suggested defaults, so an older
/// motor_config.json without this block still loads.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct IdleBehaviorConfig {
    #[serde(default = "default_idle_enabled")]
    pub enabled: bool,
    #[serde(default = "default_idle_after_seconds")]
    pub idle_after_seconds: f32,
    #[serde(default = "default_noise_channel_ids")]
    pub noise_channel_ids: Vec<usize>,
    #[serde(default = "default_noise_amplitude")]
    pub noise_amplitude: f32,
    #[serde(default = "default_noise_freq_min_hz")]
    pub noise_freq_min_hz: f32,
    #[serde(default = "default_noise_freq_max_hz")]
    pub noise_freq_max_hz: f32,
    #[serde(default = "default_blink_min_interval_seconds")]
    pub blink_min_interval_seconds: f32,
    #[serde(default = "default_blink_max_interval_seconds")]
    pub blink_max_interval_seconds: f32,
}

fn default_idle_enabled() -> bool {
    true
}
fn default_idle_after_seconds() -> f32 {
    3.0
}
fn default_noise_channel_ids() -> Vec<usize> {
    vec![0, 1, 2, 3, 8, 13, 30, 31]
}
fn default_noise_amplitude() -> f32 {
    0.03
}
fn default_noise_freq_min_hz() -> f32 {
    0.2
}
fn default_noise_freq_max_hz() -> f32 {
    0.5
}
fn default_blink_min_interval_seconds() -> f32 {
    2.0
}
fn default_blink_max_interval_seconds() -> f32 {
    6.0
}

impl Default for IdleBehaviorConfig {
    fn default() -> Self {
        Self {
            enabled: default_idle_enabled(),
            idle_after_seconds: default_idle_after_seconds(),
            noise_channel_ids: default_noise_channel_ids(),
            noise_amplitude: default_noise_amplitude(),
            noise_freq_min_hz: default_noise_freq_min_hz(),
            noise_freq_max_hz: default_noise_freq_max_hz(),
            blink_min_interval_seconds: default_blink_min_interval_seconds(),
            blink_max_interval_seconds: default_blink_max_interval_seconds(),
        }
    }
}

#[derive(Debug, Clone)]
struct ChannelNoiseState {
    current: f32,
    target: f32,
    next_retarget_at: Instant,
}

#[derive(Debug, Clone)]
enum BlinkPhase {
    Closing,
    Holding { until: Instant },
    Opening,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ExpressionPreset {
    pub id: String,
    pub label: String,
    // Bipolar normalized (-1..1) target per channel, so a preset baked at one
    // calibration still lands at the same relative pose after recalibration.
    pub norm: Vec<f32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ExpressionPresetSummary {
    pub id: String,
    pub label: String,
}

/// Easing curves for the transition layer sitting on top of the base
/// constant-velocity rate limiter (`step_towards`). `min_jerk` is
/// `10t^3 - 15t^4 + 6t^5`, the standard minimum-jerk trajectory shape.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EasingKind {
    Linear,
    EaseInOutCubic,
    MinJerk,
}

impl EasingKind {
    fn ease(self, t: f32) -> f32 {
        let t = t.clamp(0.0, 1.0);
        match self {
            EasingKind::Linear => t,
            EasingKind::EaseInOutCubic => {
                if t < 0.5 {
                    4.0 * t * t * t
                } else {
                    1.0 - (-2.0 * t + 2.0).powi(3) / 2.0
                }
            }
            EasingKind::MinJerk => t * t * t * (10.0 - 15.0 * t + 6.0 * t * t),
        }
    }
}

/// A 32-channel eased transition from `start_applied` to `end_applied`,
/// recomputed into `target_applied` every heartbeat tick based on elapsed
/// wall time. The base rate limiter still runs underneath unconditionally:
/// if the eased curve ever demands a faster step than
/// `MAX_STEP_PER_TICK_DEG`, `current_applied` simply lags behind rather than
/// the safety cap being raised or skipped.
#[derive(Debug, Clone)]
struct ActiveTransition {
    start_applied: Vec<f32>,
    end_applied: Vec<f32>,
    started_at: Instant,
    duration: Duration,
    easing: EasingKind,
}

// Deliberately snake_case on the wire (not camelCase like the rest of the
// Rust<->frontend protocol): sequences/*.json is a hand-authored asset per
// the task spec's own example, using transition_ms/hold_ms verbatim.
#[derive(Debug, Clone, Deserialize)]
struct SequenceStepDef {
    preset: String,
    #[serde(default = "default_step_intensity")]
    intensity: f32,
    transition_ms: u64,
    easing: EasingKind,
    hold_ms: u64,
}

fn default_step_intensity() -> f32 {
    1.0
}

#[derive(Debug, Clone, Deserialize)]
struct SequenceDef {
    id: String,
    label: String,
    steps: Vec<SequenceStepDef>,
    #[serde(default, rename = "loop")]
    loop_playback: bool,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct SequenceSummary {
    pub id: String,
    pub label: String,
    pub step_count: usize,
    pub loop_playback: bool,
}

#[derive(Debug, Clone)]
struct SequencePlaybackState {
    sequence_id: String,
    label: String,
    step_index: usize,
    total_steps: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SequencePlaybackStatus {
    pub playing: bool,
    pub sequence_id: Option<String>,
    pub label: Option<String>,
    pub step_index: Option<usize>,
    pub total_steps: Option<usize>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RuntimeState {
    pub endpoint: Option<String>,
    pub heartbeat_hz: u64,
    pub disabled_motor_ids: Vec<usize>,
    pub target_logical: Vec<f32>,
    pub target_applied: Vec<f32>,
    pub current_applied: Vec<f32>,
    // Bipolar normalized views (-1..1) of the target/current applied angles,
    // derived per channel. Added in normalization phase 1 for the UI.
    pub target_norm: Vec<f32>,
    pub current_norm: Vec<f32>,
    pub control_source: ControlSource,
    pub idle_behavior_enabled: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct UdpControlFrame {
    pub frame_id: u64,
    pub timestamp_ns: u128,
    pub timestamp_rfc3339: String,
    pub source: String,
    pub angles: Vec<f32>,
}

// On-wire view of a frame: the RFC3339 timestamp only exists for humans
// reading logs, so it stays out of the UDP payload.
#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct WireFrame<'a> {
    frame_id: u64,
    timestamp_ns: u128,
    source: &'a str,
    angles: &'a [f32],
}

fn encode_wire(frame: &UdpControlFrame) -> Result<Vec<u8>> {
    Ok(serde_json::to_vec(&WireFrame {
        frame_id: frame.frame_id,
        timestamp_ns: frame.timestamp_ns,
        source: &frame.source,
        angles: &frame.angles,
    })?)
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TransportStatus {
    pub connected: bool,
    pub endpoint: Option<String>,
    pub heartbeat_hz: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MotorTargetUpdate {
    pub motor_id: usize,
    pub logical_value: f32,
}

struct FrameLogger {
    jsonl: Mutex<BufWriter<std::fs::File>>,
    csv: Mutex<csv::Writer<std::fs::File>>,
}

impl FrameLogger {
    async fn new(dir: &Path) -> Result<Self> {
        tokio::fs::create_dir_all(dir).await?;

        let jsonl_path = dir.join("udp_frames.jsonl");
        let csv_path = dir.join("udp_frames.csv");

        let jsonl_file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(jsonl_path)?;
        let csv_file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(csv_path)?;

        let mut csv_writer = csv::WriterBuilder::new()
            .has_headers(false)
            .from_writer(csv_file);
        write_csv_header_if_empty(dir.join("udp_frames.csv"), &mut csv_writer)?;

        Ok(Self {
            jsonl: Mutex::new(BufWriter::new(jsonl_file)),
            csv: Mutex::new(csv_writer),
        })
    }

    async fn append(&self, frame: &UdpControlFrame) -> Result<()> {
        let encoded = serde_json::to_vec(frame)?;
        {
            let mut writer = self.jsonl.lock().await;
            writer.write_all(&encoded)?;
            writer.write_all(b"\n")?;
        }

        {
            let mut csv_writer = self.csv.lock().await;
            let mut row = Vec::with_capacity(4 + frame.angles.len());
            row.push(frame.frame_id.to_string());
            row.push(frame.timestamp_ns.to_string());
            row.push(frame.timestamp_rfc3339.clone());
            row.push(frame.source.clone());
            row.extend(frame.angles.iter().map(|value| format!("{value:.4}")));
            csv_writer.write_record(row)?;
        }

        Ok(())
    }

    async fn flush(&self) -> Result<()> {
        self.jsonl.lock().await.flush()?;
        self.csv.lock().await.flush()?;
        Ok(())
    }
}

fn write_csv_header_if_empty(path: PathBuf, writer: &mut csv::Writer<std::fs::File>) -> Result<()> {
    let should_write = std::fs::metadata(&path)
        .map(|meta| meta.len() == 0)
        .unwrap_or(true);
    if should_write {
        let mut header = vec![
            "frame_id".to_string(),
            "timestamp_ns".to_string(),
            "timestamp_rfc3339".to_string(),
            "source".to_string(),
        ];
        header.extend((0..MOTOR_COUNT).map(|index| format!("motor_{index:02}")));
        writer.write_record(header)?;
        writer.flush()?;
    }
    Ok(())
}

struct InnerState {
    frame_seq: u64,
    channels: Vec<MotorChannel>,
    jaw_coupling: Option<JawCouplingConfig>,
    expression_presets: Vec<ExpressionPreset>,
    endpoint: Option<SocketAddr>,
    target_logical: Vec<f32>,
    target_applied: Vec<f32>,
    current_applied: Vec<f32>,
    last_frame: Option<UdpControlFrame>,
    control_source: ControlSource,
    external_input_port: u16,
    external_input_timeout: Duration,
    external_last_seq: Option<u64>,
    external_last_frame_at: Option<Instant>,
    external_fps_ema: f32,
    sequences: Vec<SequenceDef>,
    active_transition: Option<ActiveTransition>,
    sequence_playback: Option<SequencePlaybackState>,
    playback_generation: u64,
    idle_behavior: IdleBehaviorConfig,
    last_moving_at: Instant,
    idle_base_applied: Option<Vec<f32>>,
    noise_states: BTreeMap<usize, ChannelNoiseState>,
    blink_phase: Option<BlinkPhase>,
    next_blink_at: Instant,
    rng: StdRng,
}

pub struct ControlService {
    logger: Arc<FrameLogger>,
    state: Arc<Mutex<InnerState>>,
    socket: Arc<UdpSocket>,
}

impl ControlService {
    pub async fn new(log_dir: PathBuf, app_dir: PathBuf) -> Result<Self> {
        let logger = Arc::new(FrameLogger::new(&log_dir).await?);
        let socket = Arc::new(
            UdpSocket::bind("0.0.0.0:0").context("failed to bind UDP socket for control frames")?,
        );
        let config = load_config(&app_dir)?;
        let channels = normalize_channels(config.channels)?;
        let jaw_coupling = normalize_jaw_coupling(config.jaw_coupling, &channels)?;
        let expression_presets = normalize_expression_presets(config.expression_presets)?;

        let target_logical = channels
            .iter()
            .map(|channel| channel.neutral_logical)
            .collect();
        let target_applied = channels
            .iter()
            .map(|channel| channel.neutral_applied)
            .collect();
        let current_applied = channels
            .iter()
            .map(|channel| channel.neutral_applied)
            .collect();

        let external_input_config = config.external_input.unwrap_or_default();
        let sequences = load_sequences(&app_dir);
        let idle_behavior = config.idle_behavior;
        let now = Instant::now();
        let mut rng = StdRng::from_rng(&mut rand::rng());
        let next_blink_at = now
            + random_seconds(
                &mut rng,
                idle_behavior.blink_min_interval_seconds,
                idle_behavior.blink_max_interval_seconds,
            );

        let state = Arc::new(Mutex::new(InnerState {
            frame_seq: 0,
            channels,
            jaw_coupling,
            expression_presets,
            endpoint: None,
            target_logical,
            target_applied,
            current_applied,
            last_frame: None,
            control_source: ControlSource::Manual,
            external_input_port: external_input_config.port,
            external_input_timeout: Duration::from_millis(external_input_config.timeout_ms),
            external_last_seq: None,
            external_last_frame_at: None,
            external_fps_ema: 0.0,
            sequences,
            active_transition: None,
            sequence_playback: None,
            playback_generation: 0,
            idle_behavior,
            last_moving_at: now,
            idle_base_applied: None,
            noise_states: BTreeMap::new(),
            blink_phase: None,
            next_blink_at,
            rng,
        }));

        spawn_udp_heartbeat(Arc::clone(&state), Arc::clone(&logger), Arc::clone(&socket));
        spawn_external_input_listener(Arc::clone(&state), external_input_config.port);

        Ok(Self {
            logger,
            state,
            socket,
        })
    }

    pub async fn connect(&self, endpoint: String) -> Result<TransportStatus> {
        let endpoint: SocketAddr = endpoint
            .parse()
            .with_context(|| format!("invalid UDP endpoint: {endpoint}"))?;
        {
            let mut state = self.state.lock().await;
            state.endpoint = Some(endpoint);
        }
        info!("UDP executor connected to {}", endpoint);
        // Safe-access handshake: push the already-settled current state out
        // immediately rather than waiting for the next keepalive tick, so
        // real hardware never sits on a stale pose after switching out of
        // sim mode. This never changes any target, so there is no jump.
        self.flush_current_frame().await?;
        Ok(TransportStatus {
            connected: true,
            endpoint: Some(endpoint.to_string()),
            heartbeat_hz: HEARTBEAT_HZ,
        })
    }

    pub async fn disconnect(&self) -> Result<()> {
        let mut state = self.state.lock().await;
        state.endpoint = None;
        Ok(())
    }

    pub async fn transport_status(&self) -> TransportStatus {
        let state = self.state.lock().await;
        TransportStatus {
            connected: state.endpoint.is_some(),
            endpoint: state.endpoint.map(|value| value.to_string()),
            heartbeat_hz: HEARTBEAT_HZ,
        }
    }

    pub async fn channels(&self) -> Vec<MotorChannel> {
        let state = self.state.lock().await;
        state.channels.clone()
    }

    pub async fn expression_presets(&self) -> Vec<ExpressionPresetSummary> {
        let state = self.state.lock().await;
        state
            .expression_presets
            .iter()
            .map(|preset| ExpressionPresetSummary {
                id: preset.id.clone(),
                label: preset.label.clone(),
            })
            .collect()
    }

    pub async fn set_motor_target(&self, update: MotorTargetUpdate) -> Result<RuntimeState> {
        let mut state = self.state.lock().await;
        ensure_manual_writable(&state)?;
        cancel_automated_motion(&mut state);
        if update.motor_id >= MOTOR_COUNT {
            bail!("motor_id {} out of range", update.motor_id);
        }
        apply_motor_target(&mut state, update.motor_id, update.logical_value);
        maybe_apply_jaw_coupling(&mut state, update.motor_id);
        Ok(build_runtime_state(&state))
    }

    pub async fn set_all_targets(&self, logical_values: Vec<f32>) -> Result<RuntimeState> {
        if logical_values.len() != MOTOR_COUNT {
            bail!("logical_values must contain exactly 32 items");
        }

        let mut state = self.state.lock().await;
        ensure_manual_writable(&state)?;
        cancel_automated_motion(&mut state);
        for (motor_id, logical) in logical_values.into_iter().enumerate() {
            apply_motor_target(&mut state, motor_id, logical);
        }
        maybe_apply_jaw_coupling_from_master(&mut state);
        Ok(build_runtime_state(&state))
    }

    pub async fn set_motor_target_norm(&self, motor_id: usize, norm: f32) -> Result<RuntimeState> {
        let mut state = self.state.lock().await;
        ensure_manual_writable(&state)?;
        cancel_automated_motion(&mut state);
        if motor_id >= MOTOR_COUNT {
            bail!("motor_id {} out of range", motor_id);
        }
        apply_motor_target_norm(&mut state, motor_id, norm);
        maybe_apply_jaw_coupling(&mut state, motor_id);
        Ok(build_runtime_state(&state))
    }

    pub async fn set_all_targets_norm(&self, norm_values: Vec<f32>) -> Result<RuntimeState> {
        if norm_values.len() != MOTOR_COUNT {
            bail!("norm_values must contain exactly 32 items");
        }

        let mut state = self.state.lock().await;
        ensure_manual_writable(&state)?;
        cancel_automated_motion(&mut state);
        for (motor_id, norm) in norm_values.into_iter().enumerate() {
            apply_motor_target_norm(&mut state, motor_id, norm);
        }
        maybe_apply_jaw_coupling_from_master(&mut state);
        Ok(build_runtime_state(&state))
    }

    pub async fn center_all(&self) -> Result<RuntimeState> {
        let mut state = self.state.lock().await;
        ensure_manual_writable(&state)?;
        cancel_automated_motion(&mut state);
        let channels = state.channels.clone();
        for (index, channel) in channels.iter().enumerate() {
            state.target_logical[index] = channel.neutral_logical;
            state.target_applied[index] = channel.neutral_applied;
        }
        Ok(build_runtime_state(&state))
    }

    /// Applies a preset via the eased transition layer (default 600ms
    /// min_jerk) instead of snapping `target_applied` straight to the
    /// preset's values; the base rate limiter still runs underneath.
    pub async fn apply_expression_preset(&self, preset_id: &str) -> Result<RuntimeState> {
        let mut state = self.state.lock().await;
        ensure_manual_writable(&state)?;
        cancel_automated_motion(&mut state);
        let preset = state
            .expression_presets
            .iter()
            .find(|preset| preset.id == preset_id)
            .cloned()
            .ok_or_else(|| anyhow!("expression preset not found: {preset_id}"))?;

        let end_applied = compute_preset_target_applied(&state.channels, &preset.norm);
        start_transition(
            &mut state,
            end_applied,
            DEFAULT_PRESET_TRANSITION,
            DEFAULT_PRESET_EASING,
        );

        Ok(build_runtime_state(&state))
    }

    /// Apply a preset scaled towards the `rest` preset's own norm vector
    /// (or towards 0 per channel if there is no `rest` preset), instead of
    /// scaling towards each channel's calibrated neutral (norm 0). The two
    /// are not the same: `rest`'s per-channel norm values are themselves
    /// mostly non-zero (see README's "表情预设系统"), so `intensity=0` lands
    /// on the declared rest pose rather than each motor's independent
    /// calibration midpoint.
    pub async fn apply_expression_preset_scaled(
        &self,
        preset_id: &str,
        intensity: f32,
    ) -> Result<RuntimeState> {
        let mut state = self.state.lock().await;
        ensure_manual_writable(&state)?;
        cancel_automated_motion(&mut state);
        let preset = state
            .expression_presets
            .iter()
            .find(|preset| preset.id == preset_id)
            .cloned()
            .ok_or_else(|| anyhow!("expression preset not found: {preset_id}"))?;
        let neutral_norm = state
            .expression_presets
            .iter()
            .find(|preset| preset.id == "rest")
            .map(|preset| preset.norm.clone());

        let scaled_norm = scale_preset_norm(&preset.norm, neutral_norm.as_deref(), intensity);
        let end_applied = compute_preset_target_applied(&state.channels, &scaled_norm);
        start_transition(
            &mut state,
            end_applied,
            DEFAULT_PRESET_TRANSITION,
            DEFAULT_PRESET_EASING,
        );

        Ok(build_runtime_state(&state))
    }

    async fn set_neck_targets(&self, norm_up_motor: f32, norm_mirror_motor: f32) {
        let mut state = self.state.lock().await;
        for (motor_id, norm) in [
            (NECK_UP_MOTOR, norm_up_motor),
            (NECK_MIRROR_MOTOR, norm_mirror_motor),
        ] {
            apply_motor_target_norm(&mut state, motor_id, norm);
        }
    }

    /// Wink: apply the wink pose, hold briefly, then return everything to the
    /// center-all neutral. Pose angles live in the "wink" expression preset.
    pub async fn wink(&self) -> Result<RuntimeState> {
        self.apply_expression_preset(WINK_PRESET_ID).await?;
        tokio::time::sleep(WINK_HOLD).await;
        self.center_all().await
    }

    /// Nod the head: oscillate the neck mirror pair up/down for a couple of
    /// cycles, then return to neutral. Targets are set over time and the
    /// heartbeat interpolates + dispatches them.
    pub async fn nod(&self) -> Result<RuntimeState> {
        {
            let mut state = self.state.lock().await;
            ensure_manual_writable(&state)?;
            cancel_automated_motion(&mut state);
        }

        // "Up" lifts motor 30 by increasing its angle; motor 31 mirrors it.
        let up = (NOD_AMPLITUDE_NORM, -NOD_AMPLITUDE_NORM);
        let down = (-NOD_AMPLITUDE_NORM, NOD_AMPLITUDE_NORM);

        for _ in 0..NOD_CYCLES {
            self.set_neck_targets(up.0, up.1).await;
            tokio::time::sleep(NOD_PHASE_DWELL).await;
            self.set_neck_targets(down.0, down.1).await;
            tokio::time::sleep(NOD_PHASE_DWELL).await;
        }
        self.set_neck_targets(0.0, 0.0).await;
        tokio::time::sleep(NOD_PHASE_DWELL).await;

        Ok(self.runtime_state().await)
    }

    pub async fn list_sequences(&self) -> Vec<SequenceSummary> {
        let state = self.state.lock().await;
        state
            .sequences
            .iter()
            .map(|sequence| SequenceSummary {
                id: sequence.id.clone(),
                label: sequence.label.clone(),
                step_count: sequence.steps.len(),
                loop_playback: sequence.loop_playback,
            })
            .collect()
    }

    pub async fn sequence_playback_status(&self) -> SequencePlaybackStatus {
        let state = self.state.lock().await;
        match &state.sequence_playback {
            Some(playback) => SequencePlaybackStatus {
                playing: true,
                sequence_id: Some(playback.sequence_id.clone()),
                label: Some(playback.label.clone()),
                step_index: Some(playback.step_index),
                total_steps: Some(playback.total_steps),
            },
            None => SequencePlaybackStatus {
                playing: false,
                sequence_id: None,
                label: None,
                step_index: None,
                total_steps: None,
            },
        }
    }

    /// Starts playback in the background (driven by `run_sequence_playback`,
    /// itself paced by the tick loop's transition layer). Not playable while
    /// External is active -- sequences are a Manual-source behavior.
    pub async fn play_sequence(&self, sequence_id: &str) -> Result<()> {
        let (sequence, generation) = {
            let mut state = self.state.lock().await;
            ensure_manual_writable(&state)?;
            let sequence = state
                .sequences
                .iter()
                .find(|sequence| sequence.id == sequence_id)
                .cloned()
                .ok_or_else(|| anyhow!("sequence not found: {sequence_id}"))?;

            cancel_automated_motion(&mut state);
            state.playback_generation = state.playback_generation.wrapping_add(1);
            let generation = state.playback_generation;
            state.sequence_playback = Some(SequencePlaybackState {
                sequence_id: sequence.id.clone(),
                label: sequence.label.clone(),
                step_index: 0,
                total_steps: sequence.steps.len(),
            });
            (sequence, generation)
        };

        let state_handle = Arc::clone(&self.state);
        tauri::async_runtime::spawn(run_sequence_playback(state_handle, sequence, generation));
        Ok(())
    }

    /// Stops playback (if any) and clears whatever transition it was
    /// mid-way through; targets stay exactly where they are.
    pub async fn stop_sequence(&self) -> RuntimeState {
        let mut state = self.state.lock().await;
        cancel_automated_motion(&mut state);
        build_runtime_state(&state)
    }

    pub async fn runtime_state(&self) -> RuntimeState {
        let state = self.state.lock().await;
        build_runtime_state(&state)
    }

    /// Immediately drop back to Manual regardless of whether the external
    /// stream is still sending. If it keeps sending, its next accepted frame
    /// re-claims External -- this is a point-in-time override for regaining
    /// the UI, not a way to permanently block a misbehaving external source.
    pub async fn force_manual_control(&self) -> RuntimeState {
        let mut state = self.state.lock().await;
        if state.control_source == ControlSource::Idle {
            exit_idle(&mut state);
        }
        state.control_source = ControlSource::Manual;
        build_runtime_state(&state)
    }

    /// Runtime master switch for idle behavior (the config file's
    /// `idleBehavior.enabled` is just the startup default). Turning it off
    /// while currently Idle drops straight back to Manual.
    pub async fn set_idle_behavior_enabled(&self, enabled: bool) -> RuntimeState {
        let mut state = self.state.lock().await;
        state.idle_behavior.enabled = enabled;
        if !enabled && state.control_source == ControlSource::Idle {
            state.control_source = ControlSource::Manual;
            exit_idle(&mut state);
        }
        build_runtime_state(&state)
    }

    pub async fn external_input_status(&self) -> ExternalInputStatus {
        let state = self.state.lock().await;
        ExternalInputStatus {
            port: state.external_input_port,
            active: state.control_source == ControlSource::External,
            last_seq: state.external_last_seq,
            fps: state.external_fps_ema,
            timeout_ms: state.external_input_timeout.as_millis() as u64,
            control_source: state.control_source,
        }
    }

    pub async fn last_frame(&self) -> Option<UdpControlFrame> {
        let state = self.state.lock().await;
        state.last_frame.clone()
    }

    pub async fn flush_current_frame(&self) -> Result<Option<UdpControlFrame>> {
        let mut state = self.state.lock().await;
        let endpoint = match state.endpoint {
            Some(endpoint) => endpoint,
            None => return Ok(None),
        };
        state.frame_seq += 1;
        let frame = build_frame(
            state.frame_seq,
            "manual-flush".to_string(),
            state.current_applied.clone(),
        )?;
        state.last_frame = Some(frame.clone());
        drop(state);

        let payload = encode_wire(&frame)?;
        self.socket.send_to(&payload, endpoint)?;
        self.logger.append(&frame).await?;
        self.logger.flush().await?;
        Ok(Some(frame))
    }
}

pub struct AppState {
    service: ControlService,
}

impl AppState {
    pub fn new(service: ControlService) -> Self {
        Self { service }
    }

    pub async fn connect(&self, endpoint: String) -> Result<TransportStatus> {
        self.service.connect(endpoint).await
    }

    pub async fn disconnect(&self) -> Result<()> {
        self.service.disconnect().await
    }

    pub async fn transport_status(&self) -> TransportStatus {
        self.service.transport_status().await
    }

    pub async fn channels(&self) -> Vec<MotorChannel> {
        self.service.channels().await
    }

    pub async fn expression_presets(&self) -> Vec<ExpressionPresetSummary> {
        self.service.expression_presets().await
    }

    pub async fn set_motor_target(&self, update: MotorTargetUpdate) -> Result<RuntimeState> {
        self.service.set_motor_target(update).await
    }

    pub async fn set_all_targets(&self, logical_values: Vec<f32>) -> Result<RuntimeState> {
        self.service.set_all_targets(logical_values).await
    }

    pub async fn set_motor_target_norm(&self, motor_id: usize, norm: f32) -> Result<RuntimeState> {
        self.service.set_motor_target_norm(motor_id, norm).await
    }

    pub async fn set_all_targets_norm(&self, norm_values: Vec<f32>) -> Result<RuntimeState> {
        self.service.set_all_targets_norm(norm_values).await
    }

    pub async fn center_all(&self) -> Result<RuntimeState> {
        self.service.center_all().await
    }

    pub async fn apply_expression_preset(&self, preset_id: &str) -> Result<RuntimeState> {
        self.service.apply_expression_preset(preset_id).await
    }

    pub async fn apply_expression_preset_scaled(
        &self,
        preset_id: &str,
        intensity: f32,
    ) -> Result<RuntimeState> {
        self.service
            .apply_expression_preset_scaled(preset_id, intensity)
            .await
    }

    pub async fn nod(&self) -> Result<RuntimeState> {
        self.service.nod().await
    }

    pub async fn wink(&self) -> Result<RuntimeState> {
        self.service.wink().await
    }

    pub async fn list_sequences(&self) -> Vec<SequenceSummary> {
        self.service.list_sequences().await
    }

    pub async fn sequence_playback_status(&self) -> SequencePlaybackStatus {
        self.service.sequence_playback_status().await
    }

    pub async fn play_sequence(&self, sequence_id: &str) -> Result<()> {
        self.service.play_sequence(sequence_id).await
    }

    pub async fn stop_sequence(&self) -> RuntimeState {
        self.service.stop_sequence().await
    }

    pub async fn runtime_state(&self) -> RuntimeState {
        self.service.runtime_state().await
    }

    pub async fn force_manual_control(&self) -> RuntimeState {
        self.service.force_manual_control().await
    }

    pub async fn set_idle_behavior_enabled(&self, enabled: bool) -> RuntimeState {
        self.service.set_idle_behavior_enabled(enabled).await
    }

    pub async fn external_input_status(&self) -> ExternalInputStatus {
        self.service.external_input_status().await
    }

    pub async fn last_frame(&self) -> Option<UdpControlFrame> {
        self.service.last_frame().await
    }

    pub async fn flush_current_frame(&self) -> Result<Option<UdpControlFrame>> {
        self.service.flush_current_frame().await
    }
}

fn load_config(app_dir: &Path) -> Result<ConfigFile> {
    let config_path = app_dir.join(CONFIG_PATH);
    let raw = std::fs::read_to_string(&config_path)
        .with_context(|| format!("failed to read motor config {}", config_path.display()))?;
    let config = serde_json::from_str::<ConfigFile>(&raw)
        .with_context(|| format!("failed to parse motor config {}", config_path.display()))?;
    Ok(config)
}

/// Loads every `sequences/*.json` file. Unlike `motor_config.json`, a
/// missing directory or an individual malformed file doesn't abort startup
/// -- these are user-authored during a session, so one bad file should not
/// take down the rest of the (otherwise working) app; it's logged and
/// skipped instead.
fn load_sequences(app_dir: &Path) -> Vec<SequenceDef> {
    let dir = app_dir.join(SEQUENCES_DIR);
    let entries = match std::fs::read_dir(&dir) {
        Ok(entries) => entries,
        Err(error) => {
            info!("no sequences directory at {}: {error}", dir.display());
            return Vec::new();
        }
    };

    let mut sequences = Vec::new();
    let mut ids = std::collections::BTreeSet::new();
    for entry in entries.flatten() {
        let path = entry.path();
        if path.extension().and_then(|ext| ext.to_str()) != Some("json") {
            continue;
        }
        let raw = match std::fs::read_to_string(&path) {
            Ok(raw) => raw,
            Err(error) => {
                warn!("failed to read sequence file {}: {error}", path.display());
                continue;
            }
        };
        let sequence = match serde_json::from_str::<SequenceDef>(&raw) {
            Ok(sequence) => sequence,
            Err(error) => {
                warn!("failed to parse sequence file {}: {error}", path.display());
                continue;
            }
        };
        if sequence.steps.is_empty() {
            warn!(
                "sequence '{}' in {} has no steps, skipping",
                sequence.id,
                path.display()
            );
            continue;
        }
        if !ids.insert(sequence.id.clone()) {
            warn!(
                "duplicate sequence id '{}' in {}, skipping",
                sequence.id,
                path.display()
            );
            continue;
        }
        sequences.push(sequence);
    }
    sequences
}

fn normalize_channels(channels: Vec<MotorChannel>) -> Result<Vec<MotorChannel>> {
    if channels.len() != MOTOR_COUNT {
        bail!("motor config must contain exactly 32 channels");
    }

    let mut slots = vec![None; MOTOR_COUNT];
    for channel in channels {
        let channel_id = channel.id;
        if channel.id >= MOTOR_COUNT {
            bail!("channel id {} out of range", channel.id);
        }
        if channel.min_applied > channel.max_applied {
            bail!("channel {} has inverted applied range", channel.id);
        }
        if channel.min_logical > channel.max_logical {
            bail!("channel {} has inverted logical range", channel.id);
        }
        if slots[channel_id].is_some() {
            bail!("duplicate channel id {}", channel_id);
        }
        slots[channel_id] = Some(channel);
    }

    slots
        .into_iter()
        .collect::<Option<Vec<_>>>()
        .ok_or_else(|| anyhow!("motor config missing channel ids"))
}

fn normalize_jaw_coupling(
    jaw_coupling: Option<JawCouplingConfig>,
    channels: &[MotorChannel],
) -> Result<Option<JawCouplingConfig>> {
    let Some(jaw_coupling) = jaw_coupling else {
        return Ok(None);
    };

    if jaw_coupling.master_motor_id >= channels.len() {
        bail!(
            "jaw coupling master motor {} out of range",
            jaw_coupling.master_motor_id
        );
    }
    if jaw_coupling.slave_ratios.is_empty() {
        bail!("jaw coupling must define at least one slave motor");
    }

    for (&slave_motor_id, &ratio) in &jaw_coupling.slave_ratios {
        if slave_motor_id >= channels.len() {
            bail!("jaw coupling slave motor {} out of range", slave_motor_id);
        }
        if slave_motor_id == jaw_coupling.master_motor_id {
            bail!("jaw coupling slave motor cannot equal master motor");
        }
        if !ratio.is_finite() {
            bail!(
                "jaw coupling ratio for slave {} must be finite",
                slave_motor_id
            );
        }
    }

    Ok(Some(jaw_coupling))
}

fn normalize_expression_presets(
    expression_presets: Vec<ExpressionPreset>,
) -> Result<Vec<ExpressionPreset>> {
    let mut ids = std::collections::BTreeSet::new();

    for preset in &expression_presets {
        if preset.id.trim().is_empty() {
            bail!("expression preset id cannot be empty");
        }
        if preset.label.trim().is_empty() {
            bail!("expression preset label cannot be empty");
        }
        if preset.norm.len() != MOTOR_COUNT {
            bail!(
                "expression preset '{}' must contain exactly {} norm values",
                preset.id,
                MOTOR_COUNT
            );
        }
        if !ids.insert(preset.id.clone()) {
            bail!("duplicate expression preset id '{}'", preset.id);
        }
    }

    Ok(expression_presets)
}

/// Manual command entry points (sliders/presets/nod/wink/center) call this
/// first: while an external coefficient stream (task 4) is actively driving
/// targets, manual writes are rejected rather than silently fighting it, so
/// there is always exactly one source of truth for the target arrays. The
/// frontend also greys out its controls, but this backend guard is the real
/// enforcement per the architecture's "no process bypasses ControlService"
/// rule extended to arbitration between command sources.
fn ensure_manual_writable(state: &InnerState) -> Result<()> {
    if state.control_source == ControlSource::External {
        bail!("control source is External; call force_manual_control to regain manual control");
    }
    Ok(())
}

fn apply_motor_target(state: &mut InnerState, motor_id: usize, logical_value: f32) {
    let channel = state.channels[motor_id].clone();
    state.target_logical[motor_id] = channel.normalized_logical(logical_value);
    state.target_applied[motor_id] = channel.logical_to_applied(logical_value);
}

fn apply_motor_target_with_applied(
    state: &mut InnerState,
    motor_id: usize,
    logical_value: f32,
    applied_value: f32,
) {
    let channel = state.channels[motor_id].clone();
    state.target_logical[motor_id] = channel.normalized_logical(logical_value);
    state.target_applied[motor_id] = applied_value.clamp(channel.min_applied, channel.max_applied);
}

/// Set a channel target from a bipolar normalized value (-1..1). Disabled
/// channels hold their neutral, mirroring the degree-based path.
fn apply_motor_target_norm(state: &mut InnerState, motor_id: usize, norm: f32) {
    let channel = state.channels[motor_id].clone();
    if !channel.enabled {
        state.target_logical[motor_id] = channel.neutral_logical;
        state.target_applied[motor_id] = channel.neutral_applied;
        return;
    }
    let applied = channel.norm_to_applied(norm);
    let logical = channel.normalized_logical(applied - channel.offset);
    apply_motor_target_with_applied(state, motor_id, logical, applied);
}

/// Bipolar norm -> applied per channel, mirroring `apply_motor_target_norm`'s
/// per-channel semantics (disabled channels pin to neutral) but returning a
/// plain vector instead of mutating state, so callers can compute a
/// transition's end point before committing to it.
fn compute_preset_target_applied(channels: &[MotorChannel], norm: &[f32]) -> Vec<f32> {
    channels
        .iter()
        .zip(norm.iter())
        .map(|(channel, &value)| {
            if channel.enabled {
                channel.norm_to_applied(value)
            } else {
                channel.neutral_applied
            }
        })
        .collect()
}

/// Scales a preset's norm vector towards a neutral reference (the `rest`
/// preset's own norm, or 0 per channel if there is no `rest` preset) by
/// `intensity` -- the same semantics as `apply_expression_preset_scaled`,
/// shared here so the sequence player can reuse it per step.
fn scale_preset_norm(norm: &[f32], neutral_norm: Option<&[f32]>, intensity: f32) -> Vec<f32> {
    let intensity = intensity.clamp(0.0, 1.0);
    norm.iter()
        .enumerate()
        .map(|(motor_id, &target)| {
            let neutral = neutral_norm.map(|values| values[motor_id]).unwrap_or(0.0);
            neutral + intensity * (target - neutral)
        })
        .collect()
}

/// Starts (or replaces) the eased transition layer. Manual writes bypass
/// this entirely; only preset application and the sequence player use it.
fn start_transition(
    state: &mut InnerState,
    end_applied: Vec<f32>,
    duration: Duration,
    easing: EasingKind,
) {
    state.active_transition = Some(ActiveTransition {
        start_applied: state.current_applied.clone(),
        end_applied,
        started_at: Instant::now(),
        duration: duration.max(Duration::from_millis(1)),
        easing,
    });
}

/// Every manual command entry point calls this first (after the
/// External-source guard): a fresh manual command always wins over whatever
/// automated motion -- an easing preset transition and/or a running
/// sequence -- was in flight. Bumping `playback_generation` is how the
/// sequence player's background task notices it has been superseded and
/// exits instead of continuing to drive targets underneath the user.
fn cancel_automated_motion(state: &mut InnerState) {
    state.active_transition = None;
    if state.sequence_playback.take().is_some() {
        state.playback_generation = state.playback_generation.wrapping_add(1);
    }
    if state.control_source == ControlSource::Idle {
        state.control_source = ControlSource::Manual;
        exit_idle(state);
    }
}

fn maybe_apply_jaw_coupling(state: &mut InnerState, updated_motor_id: usize) {
    let Some(jaw_coupling) = state.jaw_coupling.clone() else {
        return;
    };
    // One-way coupling: only the master drives the slaves. Dragging a slave
    // moves it alone (it is already set by apply_motor_target), so a
    // misaligned slave can be fine-tuned without disturbing the master.
    if updated_motor_id == jaw_coupling.master_motor_id {
        apply_jaw_coupling(state, &jaw_coupling);
    }
}

fn maybe_apply_jaw_coupling_from_master(state: &mut InnerState) {
    let Some(jaw_coupling) = state.jaw_coupling.clone() else {
        return;
    };
    apply_jaw_coupling(state, &jaw_coupling);
}

// Coupling runs in normalized (-1..1) space so a slave's travel stays
// proportional to the master's across the pair's full range, regardless of
// how their physical degree spans compare (see MotorChannel::norm_to_applied).
fn apply_jaw_coupling(state: &mut InnerState, jaw_coupling: &JawCouplingConfig) {
    let master_channel = state.channels[jaw_coupling.master_motor_id].clone();
    let master_norm =
        master_channel.applied_to_norm(state.target_applied[jaw_coupling.master_motor_id]);

    for (&slave_motor_id, &ratio) in &jaw_coupling.slave_ratios {
        let slave_channel = state.channels[slave_motor_id].clone();
        let slave_applied = slave_channel.norm_to_applied(master_norm * ratio);
        // Derive the logical view from the applied value so the two stay
        // consistent; computing it independently dropped the `direction` and
        // made a -1 slave's slider snap back on release.
        let slave_logical = slave_applied - slave_channel.offset;
        apply_motor_target_with_applied(state, slave_motor_id, slave_logical, slave_applied);
    }
}

fn build_runtime_state(state: &InnerState) -> RuntimeState {
    RuntimeState {
        endpoint: state.endpoint.map(|value| value.to_string()),
        heartbeat_hz: HEARTBEAT_HZ,
        disabled_motor_ids: state
            .channels
            .iter()
            .filter(|channel| !channel.enabled)
            .map(|channel| channel.id)
            .collect(),
        target_logical: state.target_logical.clone(),
        target_applied: state.target_applied.clone(),
        current_applied: state.current_applied.clone(),
        target_norm: normalized_view(&state.channels, &state.target_applied),
        current_norm: normalized_view(&state.channels, &state.current_applied),
        control_source: state.control_source,
        idle_behavior_enabled: state.idle_behavior.enabled,
    }
}

/// Map a vector of applied angles to their per-channel bipolar normalized view.
fn normalized_view(channels: &[MotorChannel], applied: &[f32]) -> Vec<f32> {
    channels
        .iter()
        .zip(applied.iter())
        .map(|(channel, value)| channel.applied_to_norm(*value))
        .collect()
}

fn build_frame(frame_id: u64, source: String, angles: Vec<f32>) -> Result<UdpControlFrame> {
    let timestamp_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .context("system clock before UNIX_EPOCH")?
        .as_nanos();
    let timestamp_rfc3339 = DateTime::<Utc>::from(SystemTime::now())
        .to_rfc3339_opts(chrono::SecondsFormat::Micros, true);
    Ok(UdpControlFrame {
        frame_id,
        timestamp_ns,
        timestamp_rfc3339,
        source,
        angles,
    })
}

fn step_towards(current: f32, target: f32) -> f32 {
    let delta = target - current;
    if delta.abs() <= MAX_STEP_PER_TICK_DEG {
        target
    } else {
        current + delta.signum() * MAX_STEP_PER_TICK_DEG
    }
}

/// Recomputes `target_applied`/`target_logical` from the active transition's
/// eased curve at the current wall-clock time, then clears the transition
/// once it reaches its end. `step_towards` (the base rate limiter) still
/// runs afterwards every tick regardless -- this only ever *sets a target*,
/// never a raw applied value.
fn advance_active_transition(state: &mut InnerState) {
    let Some(transition) = state.active_transition.clone() else {
        return;
    };

    let elapsed = transition.started_at.elapsed().as_secs_f32();
    let duration = transition.duration.as_secs_f32();
    let t = (elapsed / duration).clamp(0.0, 1.0);
    let eased = transition.easing.ease(t);

    for motor_id in 0..MOTOR_COUNT {
        let start = transition.start_applied[motor_id];
        let end = transition.end_applied[motor_id];
        let applied = start + (end - start) * eased;
        let offset = state.channels[motor_id].offset;
        state.target_applied[motor_id] = applied;
        state.target_logical[motor_id] = applied - offset;
    }

    if t >= 1.0 {
        state.active_transition = None;
    }
}

fn random_seconds(rng: &mut StdRng, min: f32, max: f32) -> Duration {
    let (min, max) = (min.max(0.0), max.max(0.0));
    let (low, high) = if min <= max { (min, max) } else { (max, min) };
    let secs = if high > low {
        rng.random_range(low..high)
    } else {
        low
    };
    Duration::from_secs_f32(secs)
}

/// Entry point called every heartbeat tick to drive Manual<->Idle
/// transitions and, while already Idle, the noise/blink behavior itself.
/// `moving` is this tick's "is anything still chasing a target" signal,
/// already computed by the rate-limiter loop.
fn update_idle_behavior(state: &mut InnerState, moving: bool) {
    match state.control_source {
        ControlSource::External => {}
        ControlSource::Manual => {
            let automated_motion_running =
                state.sequence_playback.is_some() || state.active_transition.is_some();
            if !state.idle_behavior.enabled || moving || automated_motion_running {
                state.last_moving_at = Instant::now();
                return;
            }
            let idle_after =
                Duration::from_secs_f32(state.idle_behavior.idle_after_seconds.max(0.0));
            if state.last_moving_at.elapsed() >= idle_after {
                enter_idle(state);
            }
        }
        ControlSource::Idle => {
            if !state.idle_behavior.enabled {
                state.control_source = ControlSource::Manual;
                exit_idle(state);
                return;
            }
            update_blink(state);
            update_idle_noise(state);
        }
    }
}

fn enter_idle(state: &mut InnerState) {
    state.control_source = ControlSource::Idle;
    state.idle_base_applied = Some(state.target_applied.clone());
    let now = Instant::now();
    state.noise_states = state
        .idle_behavior
        .noise_channel_ids
        .clone()
        .into_iter()
        .map(|channel_id| {
            (
                channel_id,
                ChannelNoiseState {
                    current: 0.0,
                    target: 0.0,
                    next_retarget_at: now,
                },
            )
        })
        .collect();
    state.next_blink_at = now
        + random_seconds(
            &mut state.rng,
            state.idle_behavior.blink_min_interval_seconds,
            state.idle_behavior.blink_max_interval_seconds,
        );
    state.blink_phase = None;
}

/// Leaves Idle bookkeeping clean; does NOT touch `control_source` itself --
/// callers (a manual write, External claiming control, or the behavior
/// being disabled mid-idle) decide what it becomes next.
fn exit_idle(state: &mut InnerState) {
    state.idle_base_applied = None;
    state.noise_states.clear();
    state.blink_phase = None;
    state.last_moving_at = Instant::now();
}

fn update_blink(state: &mut InnerState) {
    let now = Instant::now();
    match state.blink_phase.clone() {
        None => {
            if now >= state.next_blink_at {
                start_blink_transition(state, BLINK_CLOSED_NORM, BLINK_CLOSE_DURATION);
                state.blink_phase = Some(BlinkPhase::Closing);
            }
        }
        Some(BlinkPhase::Closing) => {
            if state.active_transition.is_none() {
                state.blink_phase = Some(BlinkPhase::Holding {
                    until: now + BLINK_HOLD_DURATION,
                });
            }
        }
        Some(BlinkPhase::Holding { until }) => {
            if now >= until {
                start_blink_reopen_transition(state);
                state.blink_phase = Some(BlinkPhase::Opening);
            }
        }
        Some(BlinkPhase::Opening) => {
            if state.active_transition.is_none() {
                state.blink_phase = None;
                state.next_blink_at = now
                    + random_seconds(
                        &mut state.rng,
                        state.idle_behavior.blink_min_interval_seconds,
                        state.idle_behavior.blink_max_interval_seconds,
                    );
            }
        }
    }
}

/// Starts a transition that only moves the eyelid channels (to `target_norm`
/// on each of their own scales); every other channel's start==end so it
/// simply holds still. This shares the single global `active_transition`
/// slot with presets/sequences, which is safe here because blinks only ever
/// run while Idle, and Idle is only entered when nothing else is animating.
fn start_blink_transition(state: &mut InnerState, target_norm: f32, duration: Duration) {
    let mut end = state.target_applied.clone();
    for &channel_id in &BLINK_EYELID_CHANNELS {
        let signed_norm = target_norm * blink_close_direction(channel_id);
        end[channel_id] = state.channels[channel_id].norm_to_applied(signed_norm);
    }
    start_transition(state, end, duration, EasingKind::EaseInOutCubic);
}

/// Reopens back to the pose the eyelids held right before the blink started
/// (their entry in `idle_base_applied`), not to a fixed "open" norm value --
/// idle noise may have nudged them since, though by default the eyelid
/// channels aren't in `noise_channel_ids` at all.
fn start_blink_reopen_transition(state: &mut InnerState) {
    let mut end = state.target_applied.clone();
    if let Some(base) = state.idle_base_applied.clone() {
        for &channel_id in &BLINK_EYELID_CHANNELS {
            end[channel_id] = base[channel_id];
        }
    }
    start_transition(state, end, BLINK_OPEN_DURATION, EasingKind::EaseInOutCubic);
}

/// Adds a smoothed random-walk offset (in this channel's own norm space) on
/// top of the pose `idle_base_applied` captured when Idle was entered.
/// Paused per-channel while that channel is mid-blink, per spec.
fn update_idle_noise(state: &mut InnerState) {
    let Some(base) = state.idle_base_applied.clone() else {
        return;
    };
    let now = Instant::now();
    let amplitude = state.idle_behavior.noise_amplitude.max(0.0);
    let freq_min = state.idle_behavior.noise_freq_min_hz.max(0.01);
    let freq_max = state.idle_behavior.noise_freq_max_hz.max(freq_min);

    let channel_ids: Vec<usize> = state.noise_states.keys().copied().collect();
    for channel_id in channel_ids {
        if state.blink_phase.is_some() && BLINK_EYELID_CHANNELS.contains(&channel_id) {
            continue;
        }

        let needs_retarget = state
            .noise_states
            .get(&channel_id)
            .is_some_and(|entry| now >= entry.next_retarget_at);
        if needs_retarget {
            let new_target = state.rng.random_range(-amplitude..=amplitude);
            let period_hz = state.rng.random_range(freq_min..=freq_max);
            let period = Duration::from_secs_f32(1.0 / period_hz);
            if let Some(entry) = state.noise_states.get_mut(&channel_id) {
                entry.target = new_target;
                entry.next_retarget_at = now + period;
            }
        }

        let Some(entry) = state.noise_states.get_mut(&channel_id) else {
            continue;
        };
        entry.current += (entry.target - entry.current) * NOISE_SMOOTHING_PER_TICK;
        let noise_value = entry.current;

        let channel = state.channels[channel_id].clone();
        let base_norm = channel.applied_to_norm(base[channel_id]);
        let noisy_norm = (base_norm + noise_value).clamp(-1.0, 1.0);
        let applied = channel.norm_to_applied(noisy_norm);
        state.target_applied[channel_id] = applied;
        state.target_logical[channel_id] = applied - channel.offset;
    }
}

/// Drives one `play_sequence` call end to end. Every step starts a
/// transition (reusing the exact same path as a manual preset click) then
/// sleeps out `transition_ms + hold_ms`; before starting each new step (and
/// right after waking from the sleep) it re-checks `playback_generation` so
/// a manual write, `stop_sequence`, or a competing `play_sequence` call
/// preempts it immediately rather than fighting for control on the next
/// tick.
async fn run_sequence_playback(
    state: Arc<Mutex<InnerState>>,
    sequence: SequenceDef,
    generation: u64,
) {
    loop {
        for (index, step) in sequence.steps.iter().enumerate() {
            let wait = {
                let mut guard = state.lock().await;
                if guard.playback_generation != generation {
                    return;
                }
                if guard.control_source == ControlSource::External {
                    guard.sequence_playback = None;
                    return;
                }

                let Some(preset) = guard
                    .expression_presets
                    .iter()
                    .find(|preset| preset.id == step.preset)
                    .cloned()
                else {
                    warn!(
                        "sequence '{}' step {} references unknown preset '{}', stopping playback",
                        sequence.id, index, step.preset
                    );
                    guard.sequence_playback = None;
                    return;
                };

                let neutral_norm = guard
                    .expression_presets
                    .iter()
                    .find(|preset| preset.id == "rest")
                    .map(|preset| preset.norm.clone());
                let scaled_norm =
                    scale_preset_norm(&preset.norm, neutral_norm.as_deref(), step.intensity);
                let end_applied = compute_preset_target_applied(&guard.channels, &scaled_norm);
                start_transition(
                    &mut guard,
                    end_applied,
                    Duration::from_millis(step.transition_ms),
                    step.easing,
                );

                if let Some(playback) = &mut guard.sequence_playback {
                    playback.step_index = index;
                }

                Duration::from_millis(step.transition_ms) + Duration::from_millis(step.hold_ms)
            };

            tokio::time::sleep(wait).await;

            let guard = state.lock().await;
            if guard.playback_generation != generation {
                return;
            }
        }

        let mut guard = state.lock().await;
        if guard.playback_generation != generation {
            return;
        }
        if !sequence.loop_playback {
            guard.sequence_playback = None;
            return;
        }
        // Looping: fall through to the outer loop and replay from step 0.
    }
}

fn spawn_udp_heartbeat(
    state: Arc<Mutex<InnerState>>,
    logger: Arc<FrameLogger>,
    socket: Arc<UdpSocket>,
) {
    tauri::async_runtime::spawn(async move {
        let mut ticker = tokio::time::interval(Duration::from_millis(1000 / HEARTBEAT_HZ));
        let mut tick: u64 = 0;
        let mut was_moving = false;
        let mut log_dirty = false;
        let mut last_flush = tokio::time::Instant::now();

        loop {
            ticker.tick().await;
            tick = tick.wrapping_add(1);

            let mut moving = false;
            let (endpoint, maybe_frame) = {
                let mut state = state.lock().await;
                let state = &mut *state;

                if update_external_source_timeout(state) {
                    info!("external control source timed out, falling back to Manual");
                }

                advance_active_transition(state);

                for index in 0..MOTOR_COUNT {
                    let current = state.current_applied[index];
                    let target = state.target_applied[index];
                    if current != target {
                        moving = true;
                        state.current_applied[index] = step_towards(current, target);
                    }
                }

                // Sim mode is a live digital twin, not a paused one: tick,
                // interpolation, idle behavior, frame building, and the log
                // below all run identically whether or not a real endpoint
                // is connected. Only the wire send at the bottom of this
                // loop is gated on `endpoint`.
                update_idle_behavior(state, moving);

                let maybe_frame = if moving || tick.is_multiple_of(KEEPALIVE_TICK_DIVISOR) {
                    state.frame_seq += 1;
                    let source = if moving {
                        "udp-heartbeat"
                    } else {
                        "udp-keepalive"
                    };
                    match build_frame(
                        state.frame_seq,
                        source.to_string(),
                        state.current_applied.clone(),
                    ) {
                        Ok(frame) => {
                            state.last_frame = Some(frame.clone());
                            Some(frame)
                        }
                        Err(error) => {
                            warn!("failed to build UDP frame: {error}");
                            None
                        }
                    }
                } else {
                    None
                };

                (state.endpoint, maybe_frame)
            };

            if let Some(frame) = maybe_frame {
                // Keepalive frames repeat unchanged angles, so only motion
                // frames go to the log -- true in both modes, so an
                // idle-noise/preset/sequence run stays reviewable even
                // without a Pi attached.
                if moving {
                    if let Err(error) = logger.append(&frame).await {
                        warn!("failed to append UDP frame log: {error}");
                    }
                    log_dirty = true;
                }

                if let Some(endpoint) = endpoint {
                    match encode_wire(&frame) {
                        Ok(payload) => {
                            if let Err(error) = socket.send_to(&payload, endpoint) {
                                warn!("failed to send UDP frame to {endpoint}: {error}");
                            }
                        }
                        Err(error) => warn!("failed to encode UDP frame: {error}"),
                    }
                }
            }

            let settled = was_moving && !moving;
            if log_dirty && (settled || last_flush.elapsed() >= LOG_FLUSH_INTERVAL) {
                if let Err(error) = logger.flush().await {
                    warn!("failed to flush UDP frame log: {error}");
                }
                log_dirty = false;
                last_flush = tokio::time::Instant::now();
            }
            was_moving = moving;
        }
    });
}

/// Wire frame for the task-4 external coefficient input channel (distinct
/// from the outgoing UdpControlFrame/WireFrame protocol to the Pi). A `null`
/// entry in `coefficients` means "this channel is not driven by this frame,"
/// e.g. a driver that only maps a subset of the 32 channels.
#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
struct ExternalInputFrame {
    seq: u64,
    coefficients: Vec<Option<f32>>,
}

/// Binds the external-input UDP socket and hands off received frames to
/// `apply_external_frame`. Runs on a dedicated OS thread with a blocking
/// recv + read timeout (frames arrive at low rate, e.g. 30Hz from
/// tools/mediapipe_driver.py, so a tokio task isn't warranted) rather than
/// sharing the 100Hz heartbeat's async loop.
fn spawn_external_input_listener(state: Arc<Mutex<InnerState>>, port: u16) {
    let socket = match UdpSocket::bind(("0.0.0.0", port)) {
        Ok(socket) => socket,
        Err(error) => {
            warn!("failed to bind external input UDP socket on port {port}: {error}");
            return;
        }
    };
    if let Err(error) = socket.set_read_timeout(Some(Duration::from_millis(200))) {
        warn!("failed to set external input socket read timeout: {error}");
    }
    info!("external coefficient input listening on 0.0.0.0:{port}");

    std::thread::spawn(move || {
        let mut buffer = [0u8; 65536];
        loop {
            match socket.recv_from(&mut buffer) {
                Ok((len, _addr)) => {
                    let frame = match serde_json::from_slice::<ExternalInputFrame>(&buffer[..len]) {
                        Ok(frame) => frame,
                        Err(error) => {
                            warn!("skipping invalid external input frame: {error}");
                            continue;
                        }
                    };
                    tauri::async_runtime::block_on(apply_external_frame(&state, frame));
                }
                Err(error)
                    if matches!(
                        error.kind(),
                        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                    ) => {}
                Err(error) => warn!("external input socket recv error: {error}"),
            }
        }
    });
}

/// Drops the control source back to `Manual` once the external source has gone
/// quiet for longer than its configured timeout. Returns whether it fell back,
/// so the caller can log it.
///
/// Split out of the heartbeat loop for the same reason as
/// `apply_external_frame_locked`: arbitration timing is the part worth testing,
/// and it should not require a tokio runtime to reach.
///
/// The per-session sequence tracking is cleared here on purpose. `seq` only
/// orders frames *within* one continuous run of one source, and a restarted
/// driver begins again at 1. Keeping the previous run's high-water mark would
/// make every frame of the new run look like a stale duplicate, so the source
/// could not reclaim control until it had counted back up past the old value --
/// several seconds of apparently dead input after each restart, getting worse
/// every time. Same reasoning for the frame timestamp and the fps estimate:
/// both describe a session that has ended.
fn update_external_source_timeout(state: &mut InnerState) -> bool {
    if state.control_source != ControlSource::External {
        return false;
    }
    let timed_out = state
        .external_last_frame_at
        .is_none_or(|last| last.elapsed() > state.external_input_timeout);
    if !timed_out {
        return false;
    }
    // Targets are left exactly where they are: falling back to Manual must not
    // snap the pose back.
    state.control_source = ControlSource::Manual;
    state.external_last_seq = None;
    state.external_last_frame_at = None;
    state.external_fps_ema = 0.0;
    true
}

async fn apply_external_frame(state: &Arc<Mutex<InnerState>>, frame: ExternalInputFrame) {
    let mut state = state.lock().await;
    apply_external_frame_locked(&mut state, frame);
}

/// Core logic behind `apply_external_frame`, split out so it is testable
/// against a plain `&mut InnerState` without needing a tokio runtime (the
/// caller already holds the lock).
fn apply_external_frame_locked(state: &mut InnerState, frame: ExternalInputFrame) {
    if frame.coefficients.len() != MOTOR_COUNT {
        warn!(
            "external input frame has {} coefficients, expected {}",
            frame.coefficients.len(),
            MOTOR_COUNT
        );
        return;
    }

    if let Some(last_seq) = state.external_last_seq {
        if frame.seq <= last_seq {
            // Out-of-order or duplicate: a newer frame already applied (or
            // will apply) a more current target, so drop this one.
            return;
        }
    }

    let now = Instant::now();
    if let Some(previous) = state.external_last_frame_at {
        let delta = now.duration_since(previous).as_secs_f32();
        if delta > 0.0 {
            let instant_fps = 1.0 / delta;
            state.external_fps_ema = if state.external_fps_ema <= 0.0 {
                instant_fps
            } else {
                state.external_fps_ema * (1.0 - EXTERNAL_FPS_EMA_ALPHA)
                    + instant_fps * EXTERNAL_FPS_EMA_ALPHA
            };
        }
    }
    state.external_last_seq = Some(frame.seq);
    state.external_last_frame_at = Some(now);
    if state.control_source == ControlSource::Idle {
        exit_idle(state);
    }
    state.control_source = ControlSource::External;

    for (motor_id, maybe_coefficient) in frame.coefficients.into_iter().enumerate() {
        let Some(coefficient) = maybe_coefficient else {
            continue;
        };
        let channel = state.channels[motor_id].clone();
        if !channel.enabled {
            continue;
        }
        // Unlike presets' bipolar, neutral-anchored norm space, external
        // input coefficients are a flat unipolar [0, 1] across the full
        // applied range (0 -> minApplied, 1 -> maxApplied), per this
        // channel's own wire format -- simple and matches typical ML
        // driver outputs (e.g. MediaPipe blendshapes) directly.
        let coefficient = coefficient.clamp(0.0, 1.0);
        let applied =
            channel.min_applied + coefficient * (channel.max_applied - channel.min_applied);
        let logical = applied - channel.offset;
        apply_motor_target_with_applied(state, motor_id, logical, applied);
    }

    maybe_apply_jaw_coupling_from_master(state);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ch(min_applied: f32, max_applied: f32, neutral_applied: f32) -> MotorChannel {
        MotorChannel {
            id: 0,
            name: "test".to_string(),
            board: 0,
            channel: 0,
            board_address: 0x40,
            min_applied,
            max_applied,
            offset: 0.0,
            min_logical: min_applied,
            max_logical: max_applied,
            neutral_applied,
            neutral_logical: neutral_applied,
            enabled: true,
        }
    }

    fn approx(actual: f32, expected: f32) {
        assert!(
            (actual - expected).abs() < 1e-3,
            "expected {expected}, got {actual}"
        );
    }

    #[test]
    fn norm_anchors_hit_exact_endpoints() {
        // motor 9-like: off-center neutral (min 35, max 150, neutral 118)
        let c = ch(35.0, 150.0, 118.0);
        approx(c.norm_to_applied(1.0), 150.0);
        approx(c.norm_to_applied(0.0), 118.0);
        approx(c.norm_to_applied(-1.0), 35.0);
    }

    #[test]
    fn norm_piecewise_is_asymmetric_about_neutral() {
        let c = ch(35.0, 150.0, 118.0);
        // +0.5 uses the up span (32), -0.5 uses the down span (83)
        approx(c.norm_to_applied(0.5), 134.0);
        approx(c.norm_to_applied(-0.5), 76.5);
    }

    #[test]
    fn applied_to_norm_is_the_inverse() {
        let c = ch(35.0, 150.0, 118.0);
        approx(c.applied_to_norm(150.0), 1.0);
        approx(c.applied_to_norm(118.0), 0.0);
        approx(c.applied_to_norm(35.0), -1.0);
        approx(c.applied_to_norm(134.0), 0.5);
        approx(c.applied_to_norm(76.5), -0.5);
    }

    #[test]
    fn norm_applied_roundtrip() {
        let c = ch(35.0, 150.0, 118.0);
        for &n in &[-1.0f32, -0.7, -0.3, 0.0, 0.25, 0.6, 1.0] {
            approx(c.applied_to_norm(c.norm_to_applied(n)), n);
        }
    }

    #[test]
    fn out_of_range_norm_clamps_to_limits() {
        let c = ch(35.0, 150.0, 118.0);
        approx(c.norm_to_applied(2.0), 150.0);
        approx(c.norm_to_applied(-2.0), 35.0);
    }

    #[test]
    fn degenerate_neutral_at_min_is_one_sided() {
        // motor 27-like: neutral == min, zero down span -> only [0, 1] moves
        let c = ch(60.0, 135.0, 60.0);
        approx(c.norm_to_applied(1.0), 135.0);
        approx(c.norm_to_applied(0.0), 60.0);
        approx(c.norm_to_applied(-1.0), 60.0);
        approx(c.applied_to_norm(60.0), 0.0);
        approx(c.applied_to_norm(135.0), 1.0);
    }

    #[test]
    fn degenerate_neutral_at_max_is_one_sided() {
        // motor 26-like: neutral == max, zero up span -> only [-1, 0] moves
        let c = ch(60.0, 135.0, 135.0);
        approx(c.norm_to_applied(-1.0), 60.0);
        approx(c.norm_to_applied(0.0), 135.0);
        approx(c.norm_to_applied(1.0), 135.0);
        approx(c.applied_to_norm(135.0), 0.0);
        approx(c.applied_to_norm(60.0), -1.0);
    }

    fn ch_with_id(
        id: usize,
        min_applied: f32,
        max_applied: f32,
        neutral_applied: f32,
    ) -> MotorChannel {
        MotorChannel {
            id,
            ..ch(min_applied, max_applied, neutral_applied)
        }
    }

    // Real jaw pair: motor 26 (master) has a 75deg down-span with neutral
    // pinned at its max; motor 27 (slave) has a 45/30deg asymmetric span
    // around its own neutral. A degree-based ratio clips the slave before
    // the master reaches its own limit; norm-space coupling must not.
    fn jaw_pair_state(jaw_coupling: JawCouplingConfig) -> InnerState {
        let mut channels: Vec<MotorChannel> = (0..MOTOR_COUNT)
            .map(|id| ch_with_id(id, 0.0, 1.0, 0.0))
            .collect();
        channels[26] = ch_with_id(26, 60.0, 135.0, 135.0);
        channels[27] = ch_with_id(27, 45.0, 120.0, 75.0);

        let target_applied: Vec<f32> = channels.iter().map(|c| c.neutral_applied).collect();
        InnerState {
            frame_seq: 0,
            target_logical: channels.iter().map(|c| c.neutral_logical).collect(),
            current_applied: target_applied.clone(),
            target_applied,
            channels,
            jaw_coupling: Some(jaw_coupling),
            expression_presets: Vec::new(),
            endpoint: None,
            last_frame: None,
            control_source: ControlSource::Manual,
            external_input_port: DEFAULT_EXTERNAL_INPUT_PORT,
            external_input_timeout: Duration::from_millis(DEFAULT_EXTERNAL_INPUT_TIMEOUT_MS),
            external_last_seq: None,
            external_last_frame_at: None,
            external_fps_ema: 0.0,
            sequences: Vec::new(),
            active_transition: None,
            sequence_playback: None,
            playback_generation: 0,
            idle_behavior: IdleBehaviorConfig::default(),
            last_moving_at: Instant::now(),
            idle_base_applied: None,
            noise_states: BTreeMap::new(),
            blink_phase: None,
            next_blink_at: Instant::now(),
            rng: StdRng::seed_from_u64(42),
        }
    }

    #[test]
    fn jaw_coupling_norm_space_avoids_asymmetric_span_clipping() {
        let jaw_coupling = JawCouplingConfig {
            master_motor_id: 26,
            slave_ratios: BTreeMap::from([(27, -1.0)]),
        };
        let mut state = jaw_pair_state(jaw_coupling.clone());

        // Master fully closed (its only direction of travel) should carry the
        // slave to its own limit exactly, not clip early or fall short.
        state.target_applied[26] = 60.0;
        apply_jaw_coupling(&mut state, &jaw_coupling);
        approx(state.target_applied[27], 120.0);

        // Master at neutral -> slave at its own neutral.
        state.target_applied[26] = 135.0;
        apply_jaw_coupling(&mut state, &jaw_coupling);
        approx(state.target_applied[27], 75.0);

        // Master halfway through its travel -> slave halfway through its own.
        state.target_applied[26] = 97.5;
        apply_jaw_coupling(&mut state, &jaw_coupling);
        approx(state.target_applied[27], 97.5);
    }

    fn plain_state() -> InnerState {
        let channels: Vec<MotorChannel> = (0..MOTOR_COUNT)
            .map(|id| ch_with_id(id, 0.0, 1.0, 0.0))
            .collect();
        let target_applied: Vec<f32> = channels.iter().map(|c| c.neutral_applied).collect();
        InnerState {
            frame_seq: 0,
            target_logical: channels.iter().map(|c| c.neutral_logical).collect(),
            current_applied: target_applied.clone(),
            target_applied,
            channels,
            jaw_coupling: None,
            expression_presets: Vec::new(),
            endpoint: None,
            last_frame: None,
            control_source: ControlSource::Manual,
            external_input_port: DEFAULT_EXTERNAL_INPUT_PORT,
            external_input_timeout: Duration::from_millis(DEFAULT_EXTERNAL_INPUT_TIMEOUT_MS),
            external_last_seq: None,
            external_last_frame_at: None,
            external_fps_ema: 0.0,
            sequences: Vec::new(),
            active_transition: None,
            sequence_playback: None,
            playback_generation: 0,
            idle_behavior: IdleBehaviorConfig::default(),
            last_moving_at: Instant::now(),
            idle_base_applied: None,
            noise_states: BTreeMap::new(),
            blink_phase: None,
            next_blink_at: Instant::now(),
            rng: StdRng::seed_from_u64(42),
        }
    }

    fn null_coefficients() -> Vec<Option<f32>> {
        vec![None; MOTOR_COUNT]
    }

    #[test]
    fn external_frame_maps_coefficient_to_applied_range_and_claims_control_source() {
        let mut state = plain_state();
        state.channels[5] = ch_with_id(5, 20.0, 220.0, 120.0);
        state.target_applied[5] = state.channels[5].neutral_applied;

        let mut coefficients = null_coefficients();
        coefficients[5] = Some(0.25);
        apply_external_frame_locked(
            &mut state,
            ExternalInputFrame {
                seq: 1,
                coefficients,
            },
        );

        approx(state.target_applied[5], 20.0 + 0.25 * (220.0 - 20.0));
        assert_eq!(state.control_source, ControlSource::External);
        assert_eq!(state.external_last_seq, Some(1));
    }

    #[test]
    fn external_frame_clamps_out_of_range_coefficients() {
        let mut state = plain_state();
        state.channels[3] = ch_with_id(3, 10.0, 50.0, 30.0);
        state.target_applied[3] = state.channels[3].neutral_applied;

        let mut over = null_coefficients();
        over[3] = Some(5.0);
        apply_external_frame_locked(
            &mut state,
            ExternalInputFrame {
                seq: 1,
                coefficients: over,
            },
        );
        approx(state.target_applied[3], 50.0);

        let mut under = null_coefficients();
        under[3] = Some(-5.0);
        apply_external_frame_locked(
            &mut state,
            ExternalInputFrame {
                seq: 2,
                coefficients: under,
            },
        );
        approx(state.target_applied[3], 10.0);
    }

    #[test]
    fn external_frame_ignores_disabled_and_null_channels() {
        let mut state = plain_state();
        state.channels[7] = MotorChannel {
            enabled: false,
            ..ch_with_id(7, 0.0, 180.0, 90.0)
        };
        let disabled_original = state.target_applied[7];
        let untouched_original = state.target_applied[0];

        let mut coefficients = null_coefficients();
        coefficients[7] = Some(1.0); // disabled channel: must stay put despite a coefficient
        apply_external_frame_locked(
            &mut state,
            ExternalInputFrame {
                seq: 1,
                coefficients,
            },
        );
        approx(state.target_applied[7], disabled_original);
        approx(state.target_applied[0], untouched_original); // null: also untouched
    }

    #[test]
    fn external_frame_drops_out_of_order_seq() {
        let mut state = plain_state();
        state.channels[2] = ch_with_id(2, 0.0, 100.0, 50.0);
        state.target_applied[2] = 50.0;

        let mut newer = null_coefficients();
        newer[2] = Some(1.0);
        apply_external_frame_locked(
            &mut state,
            ExternalInputFrame {
                seq: 5,
                coefficients: newer,
            },
        );
        approx(state.target_applied[2], 100.0);

        let mut stale = null_coefficients();
        stale[2] = Some(0.0);
        apply_external_frame_locked(
            &mut state,
            ExternalInputFrame {
                seq: 3,
                coefficients: stale,
            },
        );
        approx(state.target_applied[2], 100.0);
        assert_eq!(state.external_last_seq, Some(5));
    }

    /// Simulates a source whose last frame arrived `age` ago.
    fn age_external_source(state: &mut InnerState, age: Duration) {
        state.external_last_frame_at = Some(Instant::now() - age);
    }

    fn drive_external(state: &mut InnerState, seq: u64, motor_id: usize, coefficient: f32) {
        let mut coefficients = null_coefficients();
        coefficients[motor_id] = Some(coefficient);
        apply_external_frame_locked(state, ExternalInputFrame { seq, coefficients });
    }

    #[test]
    fn external_source_times_out_to_manual_without_moving_targets() {
        let mut state = plain_state();
        drive_external(&mut state, 1, 2, 1.0);
        assert_eq!(state.control_source, ControlSource::External);
        let held = state.target_applied.clone();

        age_external_source(&mut state, Duration::from_millis(600));
        assert!(update_external_source_timeout(&mut state));

        assert_eq!(state.control_source, ControlSource::Manual);
        // The whole point of falling back rather than resetting: the pose must
        // stay where the external source left it.
        assert_eq!(state.target_applied, held);
    }

    #[test]
    fn external_source_reclaims_control_after_a_restart_resets_its_seq() {
        let mut state = plain_state();
        // A first run long enough to push the sequence counter well up.
        for seq in 1..=300 {
            drive_external(&mut state, seq, 2, 1.0);
        }
        assert_eq!(state.external_last_seq, Some(300));

        age_external_source(&mut state, Duration::from_millis(600));
        assert!(update_external_source_timeout(&mut state));
        assert_eq!(state.control_source, ControlSource::Manual);

        // The driver is restarted, so its sequence numbering begins again at 1.
        // Those frames must not be mistaken for stale duplicates of the run
        // that already ended.
        drive_external(&mut state, 1, 2, 0.0);
        assert_eq!(state.control_source, ControlSource::External);
        assert_eq!(state.external_last_seq, Some(1));
        approx(state.target_applied[2], 0.0);
    }

    #[test]
    fn external_source_survives_ten_restarts_without_losing_a_frame() {
        let mut state = plain_state();
        for restart in 0..10 {
            for seq in 1..=5 {
                drive_external(&mut state, seq, 2, 1.0);
                assert_eq!(
                    state.control_source,
                    ControlSource::External,
                    "restart {restart} seq {seq} was dropped"
                );
            }
            age_external_source(&mut state, Duration::from_millis(600));
            assert!(update_external_source_timeout(&mut state));
            assert_eq!(state.control_source, ControlSource::Manual);
        }
    }

    #[test]
    fn external_source_holds_through_frame_rate_jitter() {
        let mut state = plain_state();
        // 20..40Hz means gaps of 25..50ms, all far inside the 500ms timeout, so
        // the source must never flap back to Manual mid-stream.
        let gaps_ms = [25u64, 50, 33, 41, 27, 48, 31, 44, 26, 50];
        for (index, gap) in gaps_ms.iter().enumerate() {
            drive_external(&mut state, index as u64 + 1, 2, 1.0);
            age_external_source(&mut state, Duration::from_millis(*gap));
            assert!(
                !update_external_source_timeout(&mut state),
                "gap of {gap}ms should not have timed out"
            );
            assert_eq!(state.control_source, ControlSource::External);
        }
        assert!(state.external_fps_ema > 0.0);
    }

    #[test]
    fn external_source_suppresses_idle_and_sequence_playback() {
        let mut state = idle_ready_state();
        // Reach Idle first, so the test shows External displacing an already
        // running idle behavior rather than merely preventing entry.
        update_idle_behavior(&mut state, false);
        state.last_moving_at = Instant::now() - Duration::from_secs(10);
        update_idle_behavior(&mut state, false);
        assert_eq!(state.control_source, ControlSource::Idle);

        drive_external(&mut state, 1, 2, 1.0);
        assert_eq!(state.control_source, ControlSource::External);

        // Idle must not resume while External holds the source, however long
        // the channels sit still.
        state.last_moving_at = Instant::now() - Duration::from_secs(10);
        update_idle_behavior(&mut state, false);
        assert_eq!(state.control_source, ControlSource::External);
        assert!(state.blink_phase.is_none());

        // Sequences are a Manual-source behavior, so manual writes are refused.
        assert!(ensure_manual_writable(&state).is_err());
    }

    #[test]
    fn external_frame_rejects_wrong_length() {
        let mut state = plain_state();
        let original_source = state.control_source;
        apply_external_frame_locked(
            &mut state,
            ExternalInputFrame {
                seq: 1,
                coefficients: vec![Some(1.0); 5],
            },
        );
        assert_eq!(state.control_source, original_source);
        assert_eq!(state.external_last_seq, None);
    }

    #[test]
    fn external_frame_triggers_jaw_coupling() {
        let jaw_coupling = JawCouplingConfig {
            master_motor_id: 26,
            slave_ratios: BTreeMap::from([(27, -1.0)]),
        };
        let mut state = jaw_pair_state(jaw_coupling);

        let mut coefficients = null_coefficients();
        coefficients[26] = Some(0.0); // motor 26 range 60..135 -> min_applied (fully closed)
        apply_external_frame_locked(
            &mut state,
            ExternalInputFrame {
                seq: 1,
                coefficients,
            },
        );

        approx(state.target_applied[26], 60.0);
        // 26 fully closed is norm -1 on its own scale; slave 27 (ratio -1.0)
        // goes to its own +1, i.e. max_applied (120.0).
        approx(state.target_applied[27], 120.0);
    }

    #[test]
    fn ensure_manual_writable_blocks_only_when_external() {
        let mut state = plain_state();
        assert!(ensure_manual_writable(&state).is_ok());
        state.control_source = ControlSource::External;
        assert!(ensure_manual_writable(&state).is_err());
    }

    // End-to-end exercise of the real UDP listener thread and the heartbeat
    // loop's timeout fallback (the unit tests above only cover the sync
    // `apply_external_frame_locked` core, not spawn_external_input_listener
    // or the tick-loop's timeout check). Talks to the actual configured
    // external input port from src-tauri/config/motor_config.json, so it
    // would conflict with a second concurrent instance bound to the same
    // port -- acceptable here since it is the only test doing so.
    #[tokio::test]
    async fn external_input_listener_claims_and_times_out_control_source() {
        let log_dir = std::env::temp_dir().join(format!(
            "bionic_face_test_logs_{}_{}",
            std::process::id(),
            "external_input_listener_claims_and_times_out_control_source"
        ));
        let app_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let service = ControlService::new(log_dir, app_dir)
            .await
            .expect("failed to init ControlService against the real motor_config.json");

        let status_before = service.external_input_status().await;
        assert!(!status_before.active);

        let sender = std::net::UdpSocket::bind("127.0.0.1:0").expect("bind sender socket");
        let mut coefficients = vec![serde_json::Value::Null; MOTOR_COUNT];
        coefficients[0] = serde_json::Value::from(1.0);
        let frame = serde_json::json!({
            "seq": 1,
            "timestampNs": 0,
            "coefficients": coefficients,
        });
        sender
            .send_to(
                serde_json::to_vec(&frame).unwrap().as_slice(),
                ("127.0.0.1", status_before.port),
            )
            .expect("send external frame");

        // Give the listener thread a moment to receive and apply the frame.
        tokio::time::sleep(Duration::from_millis(100)).await;
        let status_active = service.external_input_status().await;
        assert!(
            status_active.active,
            "external frame should claim control_source"
        );
        assert_eq!(status_active.last_seq, Some(1));

        // Without further frames, the heartbeat loop's timeout check (500ms
        // default) should fall back to Manual on its own.
        let wait = Duration::from_millis(status_active.timeout_ms) + Duration::from_millis(200);
        tokio::time::sleep(wait).await;
        let status_timed_out = service.external_input_status().await;
        assert!(
            !status_timed_out.active,
            "should fall back to Manual after the timeout"
        );
    }

    #[test]
    fn easing_curves_hit_anchors_and_stay_in_range() {
        for kind in [
            EasingKind::Linear,
            EasingKind::EaseInOutCubic,
            EasingKind::MinJerk,
        ] {
            approx(kind.ease(0.0), 0.0);
            approx(kind.ease(1.0), 1.0);
            let mid = kind.ease(0.5);
            assert!(
                (0.0..=1.0).contains(&mid),
                "{kind:?} at t=0.5 out of range: {mid}"
            );
        }
        // min_jerk's defining property vs. linear/cubic: zero velocity *and*
        // zero acceleration at both ends (10t^3-15t^4+6t^5 has a double root
        // at 0 and 1), which is why it's picked as the smoother default.
        approx(
            EasingKind::MinJerk.ease(0.1),
            0.1_f32.powi(3) * (10.0 - 15.0 * 0.1 + 6.0 * 0.1 * 0.1),
        );
    }

    #[test]
    fn scale_preset_norm_interpolates_towards_neutral_reference() {
        let target = vec![1.0_f32; MOTOR_COUNT];
        let mut neutral = vec![0.0_f32; MOTOR_COUNT];
        neutral[0] = 0.4;

        let at_zero = scale_preset_norm(&target, Some(&neutral), 0.0);
        approx(at_zero[0], 0.4);
        approx(at_zero[1], 0.0);

        let at_one = scale_preset_norm(&target, Some(&neutral), 1.0);
        approx(at_one[0], 1.0);
        approx(at_one[1], 1.0);

        let at_half = scale_preset_norm(&target, Some(&neutral), 0.5);
        approx(at_half[0], 0.7); // 0.4 + 0.5*(1.0-0.4)

        // No `rest` preset -> falls back to a plain 0 neutral per channel.
        let no_rest = scale_preset_norm(&target, None, 0.5);
        approx(no_rest[0], 0.5);
    }

    #[test]
    fn compute_preset_target_applied_pins_disabled_channels_to_neutral() {
        let mut state = plain_state();
        state.channels[9] = MotorChannel {
            enabled: false,
            ..ch_with_id(9, 0.0, 100.0, 40.0)
        };
        let norm = vec![1.0_f32; MOTOR_COUNT]; // would otherwise drive channel 9 to max_applied
        let applied = compute_preset_target_applied(&state.channels, &norm);
        approx(applied[9], 40.0);
    }

    #[test]
    fn advance_active_transition_interpolates_and_clears_on_completion() {
        let mut state = plain_state();
        state.channels[4] = ch_with_id(4, 0.0, 100.0, 0.0);
        state.current_applied[4] = 0.0;

        let mut end = state.current_applied.clone();
        end[4] = 100.0;
        state.active_transition = Some(ActiveTransition {
            start_applied: state.current_applied.clone(),
            end_applied: end,
            // Backdated so elapsed/duration lands mid-way through the
            // transition without a real sleep.
            started_at: Instant::now() - Duration::from_millis(300),
            duration: Duration::from_millis(600),
            easing: EasingKind::Linear,
        });

        advance_active_transition(&mut state);
        approx(state.target_applied[4], 50.0); // linear, t=0.5
        assert!(
            state.active_transition.is_some(),
            "transition should still be running at t=0.5"
        );

        if let Some(transition) = &mut state.active_transition {
            transition.started_at = Instant::now() - Duration::from_millis(1000);
        }
        advance_active_transition(&mut state);
        approx(state.target_applied[4], 100.0);
        assert!(
            state.active_transition.is_none(),
            "transition should clear once it reaches t=1.0"
        );
    }

    #[test]
    fn cancel_automated_motion_clears_transition_and_bumps_generation_only_if_playing() {
        let mut state = plain_state();
        state.active_transition = Some(ActiveTransition {
            start_applied: state.target_applied.clone(),
            end_applied: state.target_applied.clone(),
            started_at: Instant::now(),
            duration: Duration::from_millis(100),
            easing: EasingKind::Linear,
        });

        // No sequence playing: generation is untouched.
        let generation_before = state.playback_generation;
        cancel_automated_motion(&mut state);
        assert!(state.active_transition.is_none());
        assert_eq!(state.playback_generation, generation_before);

        state.active_transition = Some(ActiveTransition {
            start_applied: state.target_applied.clone(),
            end_applied: state.target_applied.clone(),
            started_at: Instant::now(),
            duration: Duration::from_millis(100),
            easing: EasingKind::Linear,
        });
        state.sequence_playback = Some(SequencePlaybackState {
            sequence_id: "demo".to_string(),
            label: "Demo".to_string(),
            step_index: 0,
            total_steps: 3,
        });
        cancel_automated_motion(&mut state);
        assert!(state.active_transition.is_none());
        assert!(state.sequence_playback.is_none());
        assert_eq!(state.playback_generation, generation_before + 1);
    }

    // End-to-end: play a real sequence file (src-tauri/sequences/demo_1.json)
    // against a real ControlService, then verify a manual write preempts
    // playback immediately (per README: dragging a slider mid-sequence stops
    // it and returns to Manual) rather than fighting the player for control.
    #[tokio::test]
    async fn sequence_playback_is_preempted_by_a_manual_write() {
        let log_dir = std::env::temp_dir().join(format!(
            "bionic_face_test_logs_{}_{}",
            std::process::id(),
            "sequence_playback_is_preempted_by_a_manual_write"
        ));
        let app_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let service = ControlService::new(log_dir, app_dir)
            .await
            .expect("failed to init ControlService");

        let sequences = service.list_sequences().await;
        assert!(
            sequences.iter().any(|s| s.id == "demo_1"),
            "expected src-tauri/sequences/demo_1.json to be loaded"
        );

        service
            .play_sequence("demo_1")
            .await
            .expect("play_sequence should start");
        tokio::time::sleep(Duration::from_millis(50)).await;
        let status = service.sequence_playback_status().await;
        assert!(
            status.playing,
            "sequence should be playing shortly after starting"
        );
        assert_eq!(status.sequence_id, Some("demo_1".to_string()));

        service
            .set_motor_target(MotorTargetUpdate {
                motor_id: 0,
                logical_value: 0.0,
            })
            .await
            .expect("manual write should succeed and preempt playback");

        let status_after = service.sequence_playback_status().await;
        assert!(
            !status_after.playing,
            "manual write should stop sequence playback"
        );
    }

    fn idle_ready_state() -> InnerState {
        let mut state = plain_state();
        state.idle_behavior.idle_after_seconds = 0.0;
        state.last_moving_at = Instant::now() - Duration::from_secs(1);
        state
    }

    #[test]
    fn manual_source_enters_idle_once_stationary_past_the_threshold() {
        let mut state = idle_ready_state();
        update_idle_behavior(&mut state, false);
        assert_eq!(state.control_source, ControlSource::Idle);
        assert!(state.idle_base_applied.is_some());
    }

    #[test]
    fn idle_entry_is_blocked_while_moving_or_automated_motion_is_running() {
        let mut state = idle_ready_state();
        update_idle_behavior(&mut state, true); // still moving
        assert_eq!(state.control_source, ControlSource::Manual);

        let mut state = idle_ready_state();
        state.sequence_playback = Some(SequencePlaybackState {
            sequence_id: "demo".to_string(),
            label: "Demo".to_string(),
            step_index: 0,
            total_steps: 1,
        });
        update_idle_behavior(&mut state, false);
        assert_eq!(
            state.control_source,
            ControlSource::Manual,
            "a running sequence should block idle entry"
        );
    }

    #[test]
    fn idle_disabled_never_enters_idle() {
        let mut state = idle_ready_state();
        state.idle_behavior.enabled = false;
        update_idle_behavior(&mut state, false);
        assert_eq!(state.control_source, ControlSource::Manual);
    }

    #[test]
    fn cancel_automated_motion_exits_idle_back_to_manual() {
        let mut state = idle_ready_state();
        update_idle_behavior(&mut state, false);
        assert_eq!(state.control_source, ControlSource::Idle);

        cancel_automated_motion(&mut state);
        assert_eq!(state.control_source, ControlSource::Manual);
        assert!(state.idle_base_applied.is_none());
        assert!(state.noise_states.is_empty());
    }

    #[test]
    fn idle_noise_stays_within_amplitude_around_the_base_pose() {
        let mut state = idle_ready_state();
        state.channels[0] = ch_with_id(0, 0.0, 200.0, 100.0);
        state.target_applied[0] = 100.0;
        state.idle_behavior.noise_channel_ids = vec![0];
        state.idle_behavior.noise_amplitude = 0.03;

        update_idle_behavior(&mut state, false); // enters idle, snapshots base pose
        assert_eq!(state.control_source, ControlSource::Idle);

        let base_norm = state.channels[0].applied_to_norm(100.0);
        for _ in 0..50 {
            update_idle_behavior(&mut state, false);
            let norm = state.channels[0].applied_to_norm(state.target_applied[0]);
            assert!(
                (norm - base_norm).abs() <= state.idle_behavior.noise_amplitude + 1e-3,
                "noise should stay within the configured amplitude of the base pose, got norm={norm}"
            );
        }
    }

    #[test]
    fn blink_progresses_through_phases_and_returns_to_base_pose() {
        let mut state = idle_ready_state();
        // plain_state() pins every test channel's neutral to its min, which
        // makes negative norm degenerate (maps to 0 regardless of sign) --
        // give channel 11 a real span around its neutral so the mirrored
        // "closed" direction is actually observable below.
        state.channels[11] = ch_with_id(11, -1.0, 1.0, 0.0);
        state.target_applied[11] = 0.0;
        state.current_applied[11] = 0.0;
        // Keep noise off the eyelid channels so only the blink moves them.
        state.idle_behavior.noise_channel_ids = vec![];
        update_idle_behavior(&mut state, false);
        assert_eq!(state.control_source, ControlSource::Idle);
        let base = state.idle_base_applied.clone().unwrap();

        // Force the blink to fire immediately.
        state.next_blink_at = Instant::now() - Duration::from_millis(1);
        update_idle_behavior(&mut state, false);
        assert!(matches!(state.blink_phase, Some(BlinkPhase::Closing)));

        // Fast-forward the closing transition to completion (mirrors the
        // real tick loop's advance_active_transition, called here directly
        // since these tests drive the state machine without a real clock).
        if let Some(transition) = &mut state.active_transition {
            transition.started_at = Instant::now() - Duration::from_secs(1);
        }
        advance_active_transition(&mut state);
        for &channel_id in &BLINK_EYELID_CHANNELS {
            let norm = state.channels[channel_id].applied_to_norm(state.target_applied[channel_id]);
            approx(norm, BLINK_CLOSED_NORM * blink_close_direction(channel_id));
        }
        update_idle_behavior(&mut state, false);
        assert!(matches!(
            state.blink_phase,
            Some(BlinkPhase::Holding { .. })
        ));

        // Fast-forward past the hold.
        if let Some(BlinkPhase::Holding { until }) = &mut state.blink_phase {
            *until = Instant::now() - Duration::from_millis(1);
        }
        update_idle_behavior(&mut state, false);
        assert!(matches!(state.blink_phase, Some(BlinkPhase::Opening)));

        // Fast-forward the opening transition to completion.
        if let Some(transition) = &mut state.active_transition {
            transition.started_at = Instant::now() - Duration::from_secs(1);
        }
        advance_active_transition(&mut state);
        update_idle_behavior(&mut state, false);
        assert!(
            state.blink_phase.is_none(),
            "blink should finish and clear its phase"
        );
        for &channel_id in &BLINK_EYELID_CHANNELS {
            approx(state.target_applied[channel_id], base[channel_id]);
        }
    }

    #[test]
    fn idle_noise_is_paused_on_channels_currently_mid_blink() {
        let mut state = idle_ready_state();
        state.idle_behavior.noise_channel_ids = vec![9]; // overlaps a blink channel on purpose
        update_idle_behavior(&mut state, false);
        state.next_blink_at = Instant::now() - Duration::from_millis(1);
        update_idle_behavior(&mut state, false); // starts the blink's closing transition
        assert!(state.blink_phase.is_some());

        let applied_from_blink = state.target_applied[9];
        update_idle_noise(&mut state);
        assert_eq!(
            state.target_applied[9], applied_from_blink,
            "noise must not touch a channel while it's mid-blink"
        );
    }
}
