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

export type RuntimeState = {
  endpoint: string | null;
  heartbeatHz: number;
  disabledMotorIds: number[];
  targetLogical: number[];
  targetApplied: number[];
  currentApplied: number[];
};

export type UdpControlFrame = {
  frameId: number;
  timestampNs: number;
  timestampRfc3339: string;
  source: string;
  angles: number[];
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

export async function setMotorTarget(motorId: number, logicalValue: number): Promise<RuntimeState> {
  return safeInvoke("set_motor_target", { motorId, logicalValue });
}

export async function setAllTargets(logicalValues: number[]): Promise<RuntimeState> {
  return safeInvoke("set_all_targets", { logicalValues });
}

export async function centerAll(): Promise<RuntimeState> {
  return safeInvoke("center_all");
}

export async function getRuntimeState(): Promise<RuntimeState> {
  return safeInvoke("get_runtime_state");
}

export async function getLastFrame(): Promise<UdpControlFrame | null> {
  return safeInvoke("get_last_frame");
}

export async function flushCurrentFrame(): Promise<UdpControlFrame | null> {
  return safeInvoke("flush_current_frame");
}
