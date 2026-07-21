use std::collections::BTreeMap;
use std::fs::OpenOptions;
use std::io::{BufWriter, Write};
use std::net::{SocketAddr, UdpSocket};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::{anyhow, bail, Context, Result};
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use tokio::sync::Mutex;
use tracing::{info, warn};

const MOTOR_COUNT: usize = 32;
const HEARTBEAT_HZ: u64 = 100;
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
const NOD_AMPLITUDE_DEG: f32 = 15.0;
const NOD_CYCLES: usize = 2;
const NOD_PHASE_DWELL: Duration = Duration::from_millis(300);

// Wink holds the wink pose briefly, then returns to the center-all neutral.
const WINK_HOLD: Duration = Duration::from_millis(500);
const WINK_PRESET_ID: &str = "wink";

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
pub struct ExpressionPreset {
    pub id: String,
    pub label: String,
    pub angles: Vec<f32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ExpressionPresetSummary {
    pub id: String,
    pub label: String,
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
        }));

        spawn_udp_heartbeat(Arc::clone(&state), Arc::clone(&logger), Arc::clone(&socket));

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
        let mut state = self.state.lock().await;
        state.endpoint = Some(endpoint);
        info!("UDP executor connected to {}", endpoint);
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
        state.expression_presets
            .iter()
            .map(|preset| ExpressionPresetSummary {
                id: preset.id.clone(),
                label: preset.label.clone(),
            })
            .collect()
    }

    pub async fn set_motor_target(&self, update: MotorTargetUpdate) -> Result<RuntimeState> {
        let mut state = self.state.lock().await;
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
        for (motor_id, logical) in logical_values.into_iter().enumerate() {
            apply_motor_target(&mut state, motor_id, logical);
        }
        maybe_apply_jaw_coupling_from_master(&mut state);
        Ok(build_runtime_state(&state))
    }

    pub async fn set_motor_target_norm(&self, motor_id: usize, norm: f32) -> Result<RuntimeState> {
        let mut state = self.state.lock().await;
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
        for (motor_id, norm) in norm_values.into_iter().enumerate() {
            apply_motor_target_norm(&mut state, motor_id, norm);
        }
        maybe_apply_jaw_coupling_from_master(&mut state);
        Ok(build_runtime_state(&state))
    }

    pub async fn center_all(&self) -> RuntimeState {
        let mut state = self.state.lock().await;
        let channels = state.channels.clone();
        for (index, channel) in channels.iter().enumerate() {
            state.target_logical[index] = channel.neutral_logical;
            state.target_applied[index] = channel.neutral_applied;
        }
        build_runtime_state(&state)
    }

    pub async fn apply_expression_preset(&self, preset_id: &str) -> Result<RuntimeState> {
        let mut state = self.state.lock().await;
        let preset = state
            .expression_presets
            .iter()
            .find(|preset| preset.id == preset_id)
            .cloned()
            .ok_or_else(|| anyhow!("expression preset not found: {preset_id}"))?;

        for (motor_id, applied_value) in preset.angles.iter().copied().enumerate() {
            let channel = state.channels[motor_id].clone();
            let applied_value = applied_value.clamp(channel.min_applied, channel.max_applied);
            let logical_value = channel.normalized_logical(applied_value - channel.offset);
            apply_motor_target_with_applied(&mut state, motor_id, logical_value, applied_value);
        }

        Ok(build_runtime_state(&state))
    }

    async fn set_neck_targets(&self, applied_up_motor: f32, applied_mirror_motor: f32) {
        let mut state = self.state.lock().await;
        for (motor_id, applied) in [
            (NECK_UP_MOTOR, applied_up_motor),
            (NECK_MIRROR_MOTOR, applied_mirror_motor),
        ] {
            let channel = state.channels[motor_id].clone();
            let applied = applied.clamp(channel.min_applied, channel.max_applied);
            let logical = channel.normalized_logical(applied - channel.offset);
            apply_motor_target_with_applied(&mut state, motor_id, logical, applied);
        }
    }

    /// Wink: apply the wink pose, hold briefly, then return everything to the
    /// center-all neutral. Pose angles live in the "wink" expression preset.
    pub async fn wink(&self) -> Result<RuntimeState> {
        self.apply_expression_preset(WINK_PRESET_ID).await?;
        tokio::time::sleep(WINK_HOLD).await;
        Ok(self.center_all().await)
    }

    /// Nod the head: oscillate the neck mirror pair up/down for a couple of
    /// cycles, then return to neutral. Targets are set over time and the
    /// heartbeat interpolates + dispatches them.
    pub async fn nod(&self) -> Result<RuntimeState> {
        let (neutral_up, neutral_mirror) = {
            let state = self.state.lock().await;
            (
                state.channels[NECK_UP_MOTOR].neutral_applied,
                state.channels[NECK_MIRROR_MOTOR].neutral_applied,
            )
        };
        // "Up" lifts motor 30 by increasing its angle; motor 31 mirrors it.
        let up = (neutral_up + NOD_AMPLITUDE_DEG, neutral_mirror - NOD_AMPLITUDE_DEG);
        let down = (neutral_up - NOD_AMPLITUDE_DEG, neutral_mirror + NOD_AMPLITUDE_DEG);

        for _ in 0..NOD_CYCLES {
            self.set_neck_targets(up.0, up.1).await;
            tokio::time::sleep(NOD_PHASE_DWELL).await;
            self.set_neck_targets(down.0, down.1).await;
            tokio::time::sleep(NOD_PHASE_DWELL).await;
        }
        self.set_neck_targets(neutral_up, neutral_mirror).await;
        tokio::time::sleep(NOD_PHASE_DWELL).await;

        Ok(self.runtime_state().await)
    }

    pub async fn runtime_state(&self) -> RuntimeState {
        let state = self.state.lock().await;
        build_runtime_state(&state)
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

    pub async fn center_all(&self) -> RuntimeState {
        self.service.center_all().await
    }

    pub async fn apply_expression_preset(&self, preset_id: &str) -> Result<RuntimeState> {
        self.service.apply_expression_preset(preset_id).await
    }

    pub async fn nod(&self) -> Result<RuntimeState> {
        self.service.nod().await
    }

    pub async fn wink(&self) -> Result<RuntimeState> {
        self.service.wink().await
    }

    pub async fn runtime_state(&self) -> RuntimeState {
        self.service.runtime_state().await
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
        if preset.angles.len() != MOTOR_COUNT {
            bail!(
                "expression preset '{}' must contain exactly {} angles",
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
            let maybe_frame = {
                let mut state = state.lock().await;
                let state = &mut *state;
                if let Some(endpoint) = state.endpoint {
                    for index in 0..MOTOR_COUNT {
                        let current = state.current_applied[index];
                        let target = state.target_applied[index];
                        if current != target {
                            moving = true;
                            state.current_applied[index] = step_towards(current, target);
                        }
                    }

                    if moving || tick % KEEPALIVE_TICK_DIVISOR == 0 {
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
                                Some((endpoint, frame))
                            }
                            Err(error) => {
                                warn!("failed to build UDP frame: {error}");
                                None
                            }
                        }
                    } else {
                        None
                    }
                } else {
                    None
                }
            };

            if let Some((endpoint, frame)) = maybe_frame {
                match encode_wire(&frame) {
                    Ok(payload) => {
                        if let Err(error) = socket.send_to(&payload, endpoint) {
                            warn!("failed to send UDP frame to {endpoint}: {error}");
                        }
                    }
                    Err(error) => warn!("failed to encode UDP frame: {error}"),
                }

                // Keepalive frames repeat unchanged angles, so only motion
                // frames go to the log.
                if moving {
                    if let Err(error) = logger.append(&frame).await {
                        warn!("failed to append UDP frame log: {error}");
                    }
                    log_dirty = true;
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
}
