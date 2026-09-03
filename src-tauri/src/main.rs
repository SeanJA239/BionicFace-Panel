mod control;

use std::path::PathBuf;
use std::sync::Arc;

use control::{
    AppState, ExpressionPresetSummary, ExternalInputStatus, MotorChannel, MotorTargetUpdate,
    RuntimeState, SequencePlaybackStatus, SequenceSummary, TransportStatus, UdpControlFrame,
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
async fn list_expression_presets(
    state: State<'_, Arc<AppState>>,
) -> Result<Vec<ExpressionPresetSummary>, String> {
    Ok(state.expression_presets().await)
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
async fn set_motor_target_norm(
    state: State<'_, Arc<AppState>>,
    motor_id: usize,
    norm: f32,
) -> Result<RuntimeState, String> {
    state
        .set_motor_target_norm(motor_id, norm)
        .await
        .map_err(|err| err.to_string())
}

#[tauri::command]
async fn set_all_targets_norm(
    state: State<'_, Arc<AppState>>,
    norm_values: Vec<f32>,
) -> Result<RuntimeState, String> {
    state
        .set_all_targets_norm(norm_values)
        .await
        .map_err(|err| err.to_string())
}

#[tauri::command]
async fn center_all(state: State<'_, Arc<AppState>>) -> Result<RuntimeState, String> {
    state.center_all().await.map_err(|err| err.to_string())
}

#[tauri::command]
async fn apply_expression_preset(
    state: State<'_, Arc<AppState>>,
    preset_id: String,
) -> Result<RuntimeState, String> {
    state
        .apply_expression_preset(&preset_id)
        .await
        .map_err(|err| err.to_string())
}

#[tauri::command]
async fn apply_expression_preset_scaled(
    state: State<'_, Arc<AppState>>,
    preset_id: String,
    intensity: f32,
) -> Result<RuntimeState, String> {
    state
        .apply_expression_preset_scaled(&preset_id, intensity)
        .await
        .map_err(|err| err.to_string())
}

#[tauri::command]
async fn nod(state: State<'_, Arc<AppState>>) -> Result<RuntimeState, String> {
    state.nod().await.map_err(|err| err.to_string())
}

#[tauri::command]
async fn wink(state: State<'_, Arc<AppState>>) -> Result<RuntimeState, String> {
    state.wink().await.map_err(|err| err.to_string())
}

#[tauri::command]
async fn laugh(state: State<'_, Arc<AppState>>) -> Result<RuntimeState, String> {
    state.laugh().await.map_err(|err| err.to_string())
}

#[tauri::command]
async fn get_runtime_state(state: State<'_, Arc<AppState>>) -> Result<RuntimeState, String> {
    Ok(state.runtime_state().await)
}

#[tauri::command]
async fn get_external_input_status(
    state: State<'_, Arc<AppState>>,
) -> Result<ExternalInputStatus, String> {
    Ok(state.external_input_status().await)
}

#[tauri::command]
async fn force_manual_control(state: State<'_, Arc<AppState>>) -> Result<RuntimeState, String> {
    Ok(state.force_manual_control().await)
}

#[tauri::command]
async fn set_idle_behavior_enabled(
    state: State<'_, Arc<AppState>>,
    enabled: bool,
) -> Result<RuntimeState, String> {
    Ok(state.set_idle_behavior_enabled(enabled).await)
}

#[tauri::command]
async fn list_sequences(state: State<'_, Arc<AppState>>) -> Result<Vec<SequenceSummary>, String> {
    Ok(state.list_sequences().await)
}

#[tauri::command]
async fn get_sequence_playback_status(
    state: State<'_, Arc<AppState>>,
) -> Result<SequencePlaybackStatus, String> {
    Ok(state.sequence_playback_status().await)
}

#[tauri::command]
async fn play_sequence(state: State<'_, Arc<AppState>>, sequence_id: String) -> Result<(), String> {
    state
        .play_sequence(&sequence_id)
        .await
        .map_err(|err| err.to_string())
}

#[tauri::command]
async fn stop_sequence(state: State<'_, Arc<AppState>>) -> Result<RuntimeState, String> {
    Ok(state.stop_sequence().await)
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

fn default_log_dir(_app_handle: &tauri::AppHandle) -> PathBuf {
    std::env::current_dir()
        .unwrap_or_else(|_| PathBuf::from("."))
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
            list_expression_presets,
            set_motor_target,
            set_all_targets,
            set_motor_target_norm,
            set_all_targets_norm,
            center_all,
            apply_expression_preset,
            apply_expression_preset_scaled,
            nod,
            laugh,
            wink,
            list_sequences,
            get_sequence_playback_status,
            play_sequence,
            stop_sequence,
            get_runtime_state,
            get_external_input_status,
            force_manual_control,
            set_idle_behavior_enabled,
            get_last_frame,
            flush_current_frame,
        ])
        .run(tauri::generate_context!())
        .expect("failed to run tauri application");
}

fn main() {
    run();
}
