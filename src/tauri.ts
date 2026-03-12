import { invoke } from "@tauri-apps/api/core";

export type MotorConfig = {
  id: number;
  name: string;
  min_angle: number;
  max_angle: number;
  zero_offset: number;
  home_logical: number;
};

export type MotorFrameAck = {
  ok: boolean;
  frame_id: number;
  timestamp_ns: number;
  endpoint: string;
  reply: unknown;
};

export type ConnectionStatus = {
  connected: boolean;
  endpoint: string | null;
};

const inTauri = typeof window !== "undefined" && "__TAURI_INTERNALS__" in window;

async function safeInvoke<T>(command: string, args: Record<string, unknown>): Promise<T> {
  if (!inTauri) {
    throw new Error(`Tauri runtime not detected for command: ${command}`);
  }
  return invoke<T>(command, args);
}

export async function connectPi(endpoint: string): Promise<ConnectionStatus> {
  return safeInvoke("connect_pi", { endpoint });
}

export async function pingPi(): Promise<unknown> {
  return safeInvoke("ping_pi", {});
}

export async function updateMotorConfig(motors: MotorConfig[]): Promise<void> {
  return safeInvoke("update_motor_config", { motors });
}

export async function setSingleMotor(
  motorId: number,
  logicalAngle: number,
  source = "topology-drag",
): Promise<MotorFrameAck> {
  return safeInvoke("set_single_motor", {
    motorId,
    logicalAngle,
    source,
  });
}

export async function sendBlendshapeFrame(
  blendshapeNames: string[],
  coefficients: number[],
  mapping: number[][],
  source = "blendshape-panel",
): Promise<MotorFrameAck> {
  return safeInvoke("send_blendshape_frame", {
    blendshapeNames,
    coefficients,
    mapping,
    source,
  });
}
