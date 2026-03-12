mod control;

use std::path::PathBuf;
use std::sync::Arc;

use control::{
    AppState, BlendshapeFrame, ConnectionStatus, ControlService, MotorAdjustRequest,
    MotorCommandFrame, MotorConfig, MotorFrameAck,
};
use tauri::State;

#[tauri::command]
async fn connect_pi(
    state: State<'_, Arc<AppState>>,
    endpoint: String,
) -> Result<ConnectionStatus, String> {
    state.connect(endpoint).await.map_err(|err| err.to_string())
}

#[tauri::command]
async fn disconnect_pi(state: State<'_, Arc<AppState>>) -> Result<(), String> {
    state.disconnect().await.map_err(|err| err.to_string())
}

#[tauri::command]
async fn update_motor_config(
    state: State<'_, Arc<AppState>>,
    motors: Vec<MotorConfig>,
) -> Result<(), String> {
    state
        .replace_motor_config(motors)
        .await
        .map_err(|err| err.to_string())
}

#[tauri::command]
async fn send_motor_frame(
    state: State<'_, Arc<AppState>>,
    logical_angles: Vec<f32>,
    source: Option<String>,
) -> Result<MotorFrameAck, String> {
    state
        .send_motor_frame(logical_angles, source)
        .await
        .map_err(|err| err.to_string())
}

#[tauri::command]
async fn send_blendshape_frame(
    state: State<'_, Arc<AppState>>,
    blendshape_names: Vec<String>,
    coefficients: Vec<f32>,
    mapping: Vec<Vec<f32>>,
    source: Option<String>,
) -> Result<MotorFrameAck, String> {
    let frame = BlendshapeFrame {
        blendshape_names,
        coefficients,
        mapping,
        source,
    };

    state
        .send_blendshape_frame(frame)
        .await
        .map_err(|err| err.to_string())
}

#[tauri::command]
async fn set_single_motor(
    state: State<'_, Arc<AppState>>,
    motor_id: usize,
    logical_angle: f32,
    source: Option<String>,
) -> Result<MotorFrameAck, String> {
    state
        .set_single_motor(MotorAdjustRequest {
            motor_id,
            logical_angle,
            source,
        })
        .await
        .map_err(|err| err.to_string())
}

#[tauri::command]
async fn ping_pi(state: State<'_, Arc<AppState>>) -> Result<serde_json::Value, String> {
    state.ping().await.map_err(|err| err.to_string())
}

#[tauri::command]
async fn get_last_frame(
    state: State<'_, Arc<AppState>>,
) -> Result<Option<MotorCommandFrame>, String> {
    Ok(state.last_frame().await)
}

fn init_tracing() {
    use tracing_subscriber::{fmt, EnvFilter};

    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    fmt().with_env_filter(filter).compact().init();
}

fn default_log_dir(app_handle: &tauri::AppHandle) -> PathBuf {
    app_handle
        .path()
        .app_local_data_dir()
        .unwrap_or_else(|_| PathBuf::from("./logs"))
        .join("logs")
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    init_tracing();

    tauri::Builder::default()
        .setup(|app| {
            let log_dir = default_log_dir(app.handle());
            let service = tauri::async_runtime::block_on(ControlService::new(log_dir))?;
            app.manage(Arc::new(AppState::new(service)));
            Ok(())
        })
        .invoke_handler(tauri::generate_handler![
            connect_pi,
            disconnect_pi,
            update_motor_config,
            send_motor_frame,
            send_blendshape_frame,
            set_single_motor,
            ping_pi,
            get_last_frame,
        ])
        .run(tauri::generate_context!())
        .expect("failed to run tauri application");
}

fn main() {
    run();
}
