import { invoke } from "@tauri-apps/api/core";

export type MotorChannel = {
  id: number;
  name: string;
  board: number;
  channel: number;
  boardAddress: number;
  minApplied: number;
  maxApplied: number;
  offset: number;
  minLogical: number;
  maxLogical: number;
  neutralApplied: number;
  neutralLogical: number;
  enabled: boolean;
};

export type TransportStatus = {
  connected: boolean;
  endpoint: string | null;
  heartbeatHz: number;
};

export type ControlSource = "manual" | "external";

export type RuntimeState = {
  endpoint: string | null;
  heartbeatHz: number;
  disabledMotorIds: number[];
  targetLogical: number[];
  targetApplied: number[];
  currentApplied: number[];
  // Bipolar normalized (-1..1) views, derived per channel (phase 1).
  targetNorm: number[];
  currentNorm: number[];
  controlSource: ControlSource;
};

export type ExternalInputStatus = {
  port: number;
  active: boolean;
  lastSeq: number | null;
  fps: number;
  timeoutMs: number;
  controlSource: ControlSource;
};

export type SequenceSummary = {
  id: string;
  label: string;
  stepCount: number;
  loopPlayback: boolean;
};

export type SequencePlaybackStatus = {
  playing: boolean;
  sequenceId: string | null;
  label: string | null;
  stepIndex: number | null;
  totalSteps: number | null;
};

export type UdpControlFrame = {
  frameId: number;
  timestampNs: number;
  timestampRfc3339: string;
  source: string;
  angles: number[];
};

export type ExpressionPresetSummary = {
  id: string;
  label: string;
};

const inTauri = typeof window !== "undefined" && "__TAURI_INTERNALS__" in window;

async function safeInvoke<T>(command: string, args: Record<string, unknown> = {}): Promise<T> {
  if (!inTauri) {
    throw new Error(`Tauri runtime not detected for command: ${command}`);
  }
  return invoke<T>(command, args);
}

export async function connectPi(endpoint: string): Promise<TransportStatus> {
  return safeInvoke("connect_pi", { endpoint });
}

export async function disconnectPi(): Promise<void> {
  return safeInvoke("disconnect_pi");
}

export async function getTransportStatus(): Promise<TransportStatus> {
  return safeInvoke("get_transport_status");
}

export async function getMotorChannels(): Promise<MotorChannel[]> {
  return safeInvoke("get_motor_channels");
}

export async function listExpressionPresets(): Promise<ExpressionPresetSummary[]> {
  return safeInvoke("list_expression_presets");
}

export async function setMotorTarget(motorId: number, logicalValue: number): Promise<RuntimeState> {
  return safeInvoke("set_motor_target", { motorId, logicalValue });
}

export async function setAllTargets(logicalValues: number[]): Promise<RuntimeState> {
  return safeInvoke("set_all_targets", { logicalValues });
}

export async function setMotorTargetNorm(motorId: number, norm: number): Promise<RuntimeState> {
  return safeInvoke("set_motor_target_norm", { motorId, norm });
}

export async function setAllTargetsNorm(normValues: number[]): Promise<RuntimeState> {
  return safeInvoke("set_all_targets_norm", { normValues });
}

export async function centerAll(): Promise<RuntimeState> {
  return safeInvoke("center_all");
}

export async function applyExpressionPreset(presetId: string): Promise<RuntimeState> {
  return safeInvoke("apply_expression_preset", { presetId });
}

export async function applyExpressionPresetScaled(
  presetId: string,
  intensity: number,
): Promise<RuntimeState> {
  return safeInvoke("apply_expression_preset_scaled", { presetId, intensity });
}

export async function nod(): Promise<RuntimeState> {
  return safeInvoke("nod");
}

export async function listSequences(): Promise<SequenceSummary[]> {
  return safeInvoke("list_sequences");
}

export async function getSequencePlaybackStatus(): Promise<SequencePlaybackStatus> {
  return safeInvoke("get_sequence_playback_status");
}

export async function playSequence(sequenceId: string): Promise<void> {
  return safeInvoke("play_sequence", { sequenceId });
}

export async function stopSequence(): Promise<RuntimeState> {
  return safeInvoke("stop_sequence");
}

export async function wink(): Promise<RuntimeState> {
  return safeInvoke("wink");
}

export async function getRuntimeState(): Promise<RuntimeState> {
  return safeInvoke("get_runtime_state");
}

export async function getExternalInputStatus(): Promise<ExternalInputStatus> {
  return safeInvoke("get_external_input_status");
}

export async function forceManualControl(): Promise<RuntimeState> {
  return safeInvoke("force_manual_control");
}

export async function getLastFrame(): Promise<UdpControlFrame | null> {
  return safeInvoke("get_last_frame");
}

export async function flushCurrentFrame(): Promise<UdpControlFrame | null> {
  return safeInvoke("flush_current_frame");
}
