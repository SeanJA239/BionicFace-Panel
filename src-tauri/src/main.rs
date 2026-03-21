mod control;

use std::path::PathBuf;
use std::sync::Arc;

use control::{
    AppState, MotorChannel, MotorTargetUpdate, RuntimeState, TransportStatus, UdpControlFrame,
};
use tauri::{Manager, State};

#[tauri::command]
async fn connect_pi(
    state: State<'_, Arc<AppState>>,
    endpoint: String,
) -> Result<TransportStatus, String> {
    state.connect(endpoint).await.map_err(|err| err.to_string())
}

#[tauri::command]
async fn disconnect_pi(state: State<'_, Arc<AppState>>) -> Result<(), String> {
    state.disconnect().await.map_err(|err| err.to_string())
}

#[tauri::command]
async fn get_transport_status(state: State<'_, Arc<AppState>>) -> Result<TransportStatus, String> {
    Ok(state.transport_status().await)
}

#[tauri::command]
async fn get_motor_channels(state: State<'_, Arc<AppState>>) -> Result<Vec<MotorChannel>, String> {
    Ok(state.channels().await)
}

#[tauri::command]
async fn set_motor_target(
    state: State<'_, Arc<AppState>>,
    motor_id: usize,
    logical_value: f32,
) -> Result<RuntimeState, String> {
    state
        .set_motor_target(MotorTargetUpdate {
            motor_id,
            logical_value,
        })
        .await
        .map_err(|err| err.to_string())
}

#[tauri::command]
async fn set_all_targets(
    state: State<'_, Arc<AppState>>,
    logical_values: Vec<f32>,
) -> Result<RuntimeState, String> {
    state
        .set_all_targets(logical_values)
        .await
        .map_err(|err| err.to_string())
}

#[tauri::command]
async fn center_all(state: State<'_, Arc<AppState>>) -> Result<RuntimeState, String> {
    Ok(state.center_all().await)
}

#[tauri::command]
async fn get_runtime_state(state: State<'_, Arc<AppState>>) -> Result<RuntimeState, String> {
    Ok(state.runtime_state().await)
}

#[tauri::command]
async fn get_last_frame(
    state: State<'_, Arc<AppState>>,
) -> Result<Option<UdpControlFrame>, String> {
    Ok(state.last_frame().await)
}

#[tauri::command]
async fn flush_current_frame(
    state: State<'_, Arc<AppState>>,
) -> Result<Option<UdpControlFrame>, String> {
    state
        .flush_current_frame()
        .await
        .map_err(|err| err.to_string())
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

fn app_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    init_tracing();

    tauri::Builder::default()
        .setup(|app| {
            let log_dir = default_log_dir(app.handle());
            let service =
                tauri::async_runtime::block_on(control::ControlService::new(log_dir, app_dir()))?;
            app.manage(Arc::new(AppState::new(service)));
            Ok(())
        })
        .invoke_handler(tauri::generate_handler![
            connect_pi,
            disconnect_pi,
            get_transport_status,
            get_motor_channels,
            set_motor_target,
            set_all_targets,
            center_all,
            get_runtime_state,
            get_last_frame,
            flush_current_frame,
        ])
        .run(tauri::generate_context!())
        .expect("failed to run tauri application");
}

fn main() {
    run();
}
