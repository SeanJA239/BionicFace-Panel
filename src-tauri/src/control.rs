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
const MAX_STEP_PER_TICK_DEG: f32 = 2.0;
const CONFIG_PATH: &str = "config/motor_config.json";

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
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
struct ConfigFile {
    transport: TransportConfig,
    channels: Vec<MotorChannel>,
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
pub struct RuntimeState {
    pub endpoint: Option<String>,
    pub heartbeat_hz: u64,
    pub disabled_motor_ids: Vec<usize>,
    pub target_logical: Vec<f32>,
    pub target_applied: Vec<f32>,
    pub current_applied: Vec<f32>,
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
            writer.flush()?;
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
            csv_writer.flush()?;
        }

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
    endpoint: Option<SocketAddr>,
    target_logical: Vec<f32>,
    target_applied: Vec<f32>,
    current_applied: Vec<f32>,
    last_frame: Option<UdpControlFrame>,
}

pub struct ControlService {
    logger: Arc<FrameLogger>,
    state: Arc<Mutex<InnerState>>,
}

impl ControlService {
    pub async fn new(log_dir: PathBuf, app_dir: PathBuf) -> Result<Self> {
        let logger = Arc::new(FrameLogger::new(&log_dir).await?);
        let config = load_config(&app_dir)?;
        let channels = normalize_channels(config.channels)?;

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
            endpoint: None,
            target_logical,
            target_applied,
            current_applied,
            last_frame: None,
        }));

        spawn_udp_heartbeat(Arc::clone(&state), Arc::clone(&logger));

        Ok(Self { logger, state })
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

    pub async fn set_motor_target(&self, update: MotorTargetUpdate) -> Result<RuntimeState> {
        let mut state = self.state.lock().await;
        if update.motor_id >= MOTOR_COUNT {
            bail!("motor_id {} out of range", update.motor_id);
        }
        let channel = state.channels[update.motor_id].clone();
        state.target_logical[update.motor_id] = channel.normalized_logical(update.logical_value);
        state.target_applied[update.motor_id] = channel.logical_to_applied(update.logical_value);
        Ok(build_runtime_state(&state))
    }

    pub async fn set_all_targets(&self, logical_values: Vec<f32>) -> Result<RuntimeState> {
        if logical_values.len() != MOTOR_COUNT {
            bail!("logical_values must contain exactly 32 items");
        }

        let mut state = self.state.lock().await;
        for (motor_id, logical) in logical_values.into_iter().enumerate() {
            let channel = state.channels[motor_id].clone();
            state.target_logical[motor_id] = channel.normalized_logical(logical);
            state.target_applied[motor_id] = channel.logical_to_applied(logical);
        }
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

    pub async fn runtime_state(&self) -> RuntimeState {
        let state = self.state.lock().await;
        build_runtime_state(&state)
    }

    pub async fn last_frame(&self) -> Option<UdpControlFrame> {
        let state = self.state.lock().await;
        state.last_frame.clone()
    }

    pub async fn flush_current_frame(&self) -> Result<Option<UdpControlFrame>> {
        let state = self.state.lock().await;
        let endpoint = match state.endpoint {
            Some(endpoint) => endpoint,
            None => return Ok(None),
        };
        let frame = build_frame(
            state.frame_seq + 1,
            "manual-flush".to_string(),
            state.current_applied.clone(),
        )?;
        drop(state);

        send_frame(endpoint, &frame)?;
        self.logger.append(&frame).await?;

        let mut state = self.state.lock().await;
        state.frame_seq = frame.frame_id;
        state.last_frame = Some(frame.clone());
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

    pub async fn set_motor_target(&self, update: MotorTargetUpdate) -> Result<RuntimeState> {
        self.service.set_motor_target(update).await
    }

    pub async fn set_all_targets(&self, logical_values: Vec<f32>) -> Result<RuntimeState> {
        self.service.set_all_targets(logical_values).await
    }

    pub async fn center_all(&self) -> RuntimeState {
        self.service.center_all().await
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
    }
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

fn spawn_udp_heartbeat(state: Arc<Mutex<InnerState>>, logger: Arc<FrameLogger>) {
    tauri::async_runtime::spawn(async move {
        let socket = match UdpSocket::bind("0.0.0.0:0") {
            Ok(socket) => socket,
            Err(error) => {
                warn!("failed to bind UDP socket for heartbeat: {error}");
                return;
            }
        };

        let mut ticker = tokio::time::interval(Duration::from_millis(1000 / HEARTBEAT_HZ));
        loop {
            ticker.tick().await;

            let maybe_frame = {
                let mut state = state.lock().await;
                if let Some(endpoint) = state.endpoint {
                    for index in 0..MOTOR_COUNT {
                        state.current_applied[index] =
                            step_towards(state.current_applied[index], state.target_applied[index]);
                    }

                    state.frame_seq += 1;
                    let frame = match build_frame(
                        state.frame_seq,
                        "udp-heartbeat".to_string(),
                        state.current_applied.clone(),
                    ) {
                        Ok(frame) => frame,
                        Err(error) => {
                            warn!("failed to build UDP frame: {error}");
                            continue;
                        }
                    };
                    state.last_frame = Some(frame.clone());
                    Some((endpoint, frame))
                } else {
                    None
                }
            };

            let Some((endpoint, frame)) = maybe_frame else {
                continue;
            };

            let payload = match serde_json::to_vec(&frame) {
                Ok(payload) => payload,
                Err(error) => {
                    warn!("failed to encode UDP frame: {error}");
                    continue;
                }
            };

            if let Err(error) = socket.send_to(&payload, endpoint) {
                warn!("failed to send UDP frame to {endpoint}: {error}");
                continue;
            }

            if let Err(error) = logger.append(&frame).await {
                warn!("failed to append UDP frame log: {error}");
            }
        }
    });
}

fn send_frame(endpoint: SocketAddr, frame: &UdpControlFrame) -> Result<()> {
    let socket = UdpSocket::bind("0.0.0.0:0")?;
    let payload = serde_json::to_vec(frame)?;
    socket.send_to(&payload, endpoint)?;
    Ok(())
}
