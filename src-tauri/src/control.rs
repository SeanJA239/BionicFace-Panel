use std::fs::OpenOptions;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{anyhow, bail, Context, Result};
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use tokio::sync::Mutex;
use tracing::info;

const MOTOR_COUNT: usize = 32;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorConfig {
    pub id: usize,
    pub name: String,
    pub min_angle: f32,
    pub max_angle: f32,
    pub zero_offset: f32,
    pub home_logical: f32,
}

impl MotorConfig {
    fn apply(&self, logical: f32) -> f32 {
        (logical + self.zero_offset).clamp(self.min_angle, self.max_angle)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BlendshapeFrame {
    pub blendshape_names: Vec<String>,
    pub coefficients: Vec<f32>,
    pub mapping: Vec<Vec<f32>>,
    pub source: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorCommandFrame {
    pub frame_id: u64,
    pub timestamp_ns: u128,
    pub timestamp_rfc3339: String,
    pub source: String,
    pub logical_angles: Vec<f32>,
    pub applied_angles: Vec<f32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorFrameAck {
    pub ok: bool,
    pub frame_id: u64,
    pub timestamp_ns: u128,
    pub endpoint: String,
    pub reply: serde_json::Value,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorAdjustRequest {
    pub motor_id: usize,
    pub logical_angle: f32,
    pub source: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConnectionStatus {
    pub connected: bool,
    pub endpoint: Option<String>,
}

#[derive(Default)]
struct ZmqClient {
    context: Option<zmq::Context>,
    endpoint: Option<String>,
    socket: Option<zmq::Socket>,
}

impl ZmqClient {
    fn connect(&mut self, endpoint: &str) -> Result<ConnectionStatus> {
        if self.socket.is_some() {
            self.disconnect()?;
        }

        let context = zmq::Context::new();
        let socket = context.socket(zmq::REQ)?;
        socket.set_linger(0)?;
        socket.set_rcvtimeo(100)?;
        socket.set_sndtimeo(100)?;
        socket.connect(endpoint)?;

        self.context = Some(context);
        self.endpoint = Some(endpoint.to_string());
        self.socket = Some(socket);

        Ok(ConnectionStatus {
            connected: true,
            endpoint: self.endpoint.clone(),
        })
    }

    fn disconnect(&mut self) -> Result<()> {
        if let Some(socket) = self.socket.take() {
            socket.disconnect(self.endpoint.as_deref().unwrap_or_default())?;
        }
        self.endpoint = None;
        self.context = None;
        Ok(())
    }

    fn send_json(&self, value: &serde_json::Value) -> Result<serde_json::Value> {
        let socket = self
            .socket
            .as_ref()
            .ok_or_else(|| anyhow!("ZMQ endpoint is not connected"))?;

        socket.send(serde_json::to_vec(value)?, 0)?;
        let message = socket.recv_msg(0)?;
        Ok(serde_json::from_slice(message.as_ref())?)
    }

    fn endpoint(&self) -> Result<String> {
        self.endpoint
            .clone()
            .ok_or_else(|| anyhow!("ZMQ endpoint is not connected"))
    }
}

struct FrameLogger {
    jsonl: Mutex<BufWriter<std::fs::File>>,
    csv: Mutex<csv::Writer<std::fs::File>>,
}

impl FrameLogger {
    async fn new(dir: &Path) -> Result<Self> {
        tokio::fs::create_dir_all(dir).await?;

        let jsonl_path = dir.join("motor_frames.jsonl");
        let csv_path = dir.join("motor_frames.csv");

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
        write_csv_header_if_empty(dir.join("motor_frames.csv"), &mut csv_writer)?;

        Ok(Self {
            jsonl: Mutex::new(BufWriter::new(jsonl_file)),
            csv: Mutex::new(csv_writer),
        })
    }

    async fn append(&self, frame: &MotorCommandFrame) -> Result<()> {
        let encoded = serde_json::to_vec(frame)?;
        {
            let mut writer = self.jsonl.lock().await;
            writer.write_all(&encoded)?;
            writer.write_all(b"\n")?;
            writer.flush()?;
        }

        {
            let mut csv_writer = self.csv.lock().await;
            let mut row = Vec::with_capacity(4 + frame.applied_angles.len());
            row.push(frame.frame_id.to_string());
            row.push(frame.timestamp_ns.to_string());
            row.push(frame.timestamp_rfc3339.clone());
            row.push(frame.source.clone());
            row.extend(
                frame
                    .applied_angles
                    .iter()
                    .map(|value| format!("{value:.4}")),
            );
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
    motor_config: Vec<MotorConfig>,
    last_frame: Option<MotorCommandFrame>,
    zmq: ZmqClient,
}

pub struct ControlService {
    logger: Arc<FrameLogger>,
    state: Mutex<InnerState>,
}

impl ControlService {
    pub async fn new(log_dir: PathBuf) -> Result<Self> {
        let logger = Arc::new(FrameLogger::new(&log_dir).await?);

        Ok(Self {
            logger,
            state: Mutex::new(InnerState {
                frame_seq: 0,
                motor_config: default_motor_config(),
                last_frame: None,
                zmq: ZmqClient::default(),
            }),
        })
    }

    pub async fn connect(&self, endpoint: String) -> Result<ConnectionStatus> {
        let mut state = self.state.lock().await;
        let status = state.zmq.connect(&endpoint)?;
        info!("Connected to Pi endpoint {}", endpoint);
        Ok(status)
    }

    pub async fn disconnect(&self) -> Result<()> {
        let mut state = self.state.lock().await;
        state.zmq.disconnect()?;
        info!("Disconnected from Pi endpoint");
        Ok(())
    }

    pub async fn replace_motor_config(&self, motors: Vec<MotorConfig>) -> Result<()> {
        let mut state = self.state.lock().await;
        state.motor_config = normalize_motor_config(motors)?;
        Ok(())
    }

    pub async fn ping(&self) -> Result<serde_json::Value> {
        let state = self.state.lock().await;
        state
            .zmq
            .send_json(&serde_json::json!({ "command": "ping" }))
    }

    pub async fn send_blendshape_frame(&self, frame: BlendshapeFrame) -> Result<MotorFrameAck> {
        if frame.coefficients.len() != frame.blendshape_names.len() {
            bail!("blendshape_names and coefficients length mismatch");
        }
        if frame.mapping.len() != MOTOR_COUNT {
            bail!("mapping must contain exactly 32 motor rows");
        }
        if frame
            .mapping
            .iter()
            .any(|row| row.len() != frame.coefficients.len())
        {
            bail!("each mapping row length must equal blendshape count");
        }

        let logical_angles = frame
            .mapping
            .iter()
            .map(|row| {
                row.iter()
                    .zip(frame.coefficients.iter())
                    .fold(0.0_f32, |sum, (weight, coeff)| sum + weight * coeff)
            })
            .collect::<Vec<_>>();

        self.send_motor_frame(logical_angles, frame.source).await
    }

    pub async fn send_motor_frame(
        &self,
        logical_angles: Vec<f32>,
        source: Option<String>,
    ) -> Result<MotorFrameAck> {
        if logical_angles.len() != MOTOR_COUNT {
            bail!("logical_angles must contain exactly 32 values");
        }

        let frame = {
            let mut state = self.state.lock().await;
            let next_frame_seq = state.frame_seq + 1;
            let motor_config = state.motor_config.clone();
            state.frame_seq = next_frame_seq;
            build_frame(
                next_frame_seq,
                &motor_config,
                logical_angles,
                source.unwrap_or_else(|| "ui".to_string()),
            )?
        };

        self.logger.append(&frame).await?;

        let (endpoint, reply) = {
            let mut state = self.state.lock().await;
            let endpoint = state.zmq.endpoint()?;
            let payload = serde_json::json!({
                "command": "set_all",
                "frame_id": frame.frame_id,
                "timestamp_ns": frame.timestamp_ns,
                "logical_angles": frame.logical_angles,
                "applied_angles": frame.applied_angles,
            });
            let reply = state.zmq.send_json(&payload)?;
            state.last_frame = Some(frame.clone());
            (endpoint, reply)
        };

        Ok(MotorFrameAck {
            ok: true,
            frame_id: frame.frame_id,
            timestamp_ns: frame.timestamp_ns,
            endpoint,
            reply,
        })
    }

    pub async fn set_single_motor(&self, request: MotorAdjustRequest) -> Result<MotorFrameAck> {
        if request.motor_id >= MOTOR_COUNT {
            bail!("motor_id {} out of range", request.motor_id);
        }

        let logical_angles = {
            let state = self.state.lock().await;
            let mut logical = state
                .last_frame
                .as_ref()
                .map(|frame| frame.logical_angles.clone())
                .unwrap_or_else(|| vec![0.0; MOTOR_COUNT]);
            logical[request.motor_id] = request.logical_angle;
            logical
        };

        self.send_motor_frame(logical_angles, request.source).await
    }

    pub async fn last_frame(&self) -> Option<MotorCommandFrame> {
        let state = self.state.lock().await;
        state.last_frame.clone()
    }
}

pub struct AppState {
    service: ControlService,
}

impl AppState {
    pub fn new(service: ControlService) -> Self {
        Self { service }
    }

    pub async fn connect(&self, endpoint: String) -> Result<ConnectionStatus> {
        self.service.connect(endpoint).await
    }

    pub async fn disconnect(&self) -> Result<()> {
        self.service.disconnect().await
    }

    pub async fn replace_motor_config(&self, motors: Vec<MotorConfig>) -> Result<()> {
        self.service.replace_motor_config(motors).await
    }

    pub async fn send_motor_frame(
        &self,
        logical_angles: Vec<f32>,
        source: Option<String>,
    ) -> Result<MotorFrameAck> {
        self.service.send_motor_frame(logical_angles, source).await
    }

    pub async fn set_single_motor(&self, request: MotorAdjustRequest) -> Result<MotorFrameAck> {
        self.service.set_single_motor(request).await
    }

    pub async fn send_blendshape_frame(&self, frame: BlendshapeFrame) -> Result<MotorFrameAck> {
        self.service.send_blendshape_frame(frame).await
    }

    pub async fn ping(&self) -> Result<serde_json::Value> {
        self.service.ping().await
    }

    pub async fn last_frame(&self) -> Option<MotorCommandFrame> {
        self.service.last_frame().await
    }
}

fn build_frame(
    frame_seq: u64,
    motor_config: &[MotorConfig],
    logical_angles: Vec<f32>,
    source: String,
) -> Result<MotorCommandFrame> {
    if motor_config.len() != MOTOR_COUNT {
        bail!("motor_config must contain exactly 32 entries");
    }

    let timestamp_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .context("system clock before UNIX_EPOCH")?
        .as_nanos();
    let timestamp_rfc3339 = DateTime::<Utc>::from(SystemTime::now())
        .to_rfc3339_opts(chrono::SecondsFormat::Micros, true);
    let applied_angles = logical_angles
        .iter()
        .zip(motor_config.iter())
        .map(|(angle, config)| config.apply(*angle))
        .collect::<Vec<_>>();

    Ok(MotorCommandFrame {
        frame_id: frame_seq,
        timestamp_ns,
        timestamp_rfc3339,
        source,
        logical_angles,
        applied_angles,
    })
}

fn default_motor_config() -> Vec<MotorConfig> {
    (0..MOTOR_COUNT)
        .map(|id| MotorConfig {
            id,
            name: format!("motor_{id:02}"),
            min_angle: 0.0,
            max_angle: 180.0,
            zero_offset: 0.0,
            home_logical: 90.0,
        })
        .collect()
}

fn normalize_motor_config(motors: Vec<MotorConfig>) -> Result<Vec<MotorConfig>> {
    if motors.len() != MOTOR_COUNT {
        bail!("motor config must contain exactly 32 entries");
    }

    let mut slots = vec![None; MOTOR_COUNT];
    for motor in motors {
        let motor_id = motor.id;
        if motor_id >= MOTOR_COUNT {
            bail!("motor id {} out of range", motor_id);
        }
        if motor.min_angle > motor.max_angle {
            bail!("motor id {} has inverted min/max", motor_id);
        }
        if slots[motor_id].is_some() {
            bail!("duplicate motor id {}", motor_id);
        }
        slots[motor_id] = Some(motor);
    }

    slots
        .into_iter()
        .collect::<Option<Vec<_>>>()
        .ok_or_else(|| anyhow!("motor config missing required ids"))
}
