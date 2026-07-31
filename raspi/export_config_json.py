from __future__ import annotations

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "raspi" / "config.py"
OUTPUT_PATH = ROOT / "src-tauri" / "config" / "motor_config.json"
PRESETS_PATH = ROOT / "presets.json"


def load_module(path: Path):
    spec = importlib.util.spec_from_file_location("raspi_config", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load config from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def build_channel(module, motor_id: int) -> dict:
    motor_map = module.MOTOR_MAP
    limits = getattr(module, "MOTOR_LIMITS", {})
    offsets = getattr(module, "MOTOR_OFFSET", {})
    initial_applied = getattr(module, "MOTOR_INITIAL_APPLIED", {})
    names = getattr(module, "MOTOR_NAMES", {})
    disabled_motors = set(getattr(module, "DISABLED_MOTORS", []))
    board_addresses = list(getattr(module, "BOARD_ADDRESSES", [0x40, 0x41]))

    board, channel = motor_map[motor_id]
    min_applied, max_applied = limits.get(motor_id, (0, 180))
    offset = float(offsets.get(motor_id, 0))
    neutral_applied = float(
        initial_applied.get(motor_id, (float(min_applied) + float(max_applied)) / 2.0)
    )
    neutral_applied = clamp(neutral_applied, float(min_applied), float(max_applied))
    neutral_logical = neutral_applied - offset
    enabled = motor_id not in disabled_motors

    return {
        "id": motor_id,
        "name": names.get(motor_id, f"motor_{motor_id:02d}"),
        "board": int(board),
        "channel": int(channel),
        "boardAddress": board_addresses[int(board)],
        "minApplied": float(min_applied),
        "maxApplied": float(max_applied),
        "offset": offset,
        "minLogical": float(min_applied) - offset,
        "maxLogical": float(max_applied) - offset,
        "neutralApplied": neutral_applied,
        "neutralLogical": neutral_logical,
        "enabled": enabled,
    }


def build_jaw_coupling(module) -> dict | None:
    """control.rs's JawCouplingConfig (norm-space jaw coupling) takes one
    signed ratio per slave: slave_norm = master_norm * slave_ratios[slave].
    config.py's JAW_COUPLING still separates that into a shared `ratio`
    magnitude plus a per-slave `directions` sign, so combine them here.
    """
    coupling = getattr(module, "JAW_COUPLING", None)
    if not coupling:
        return None

    master_motor_id = int(coupling["master_motor_id"])
    slave_motor_ids = [int(motor_id) for motor_id in coupling["slave_motor_ids"]]
    ratio = float(coupling.get("ratio", 1.0))
    raw_directions = coupling.get("directions", {})

    slave_ratios = {}
    for motor_id in slave_motor_ids:
        direction = float(raw_directions.get(motor_id, 1.0))
        slave_ratios[str(motor_id)] = ratio * direction

    return {
        "masterMotorId": master_motor_id,
        "slaveRatios": slave_ratios,
    }


def load_expression_presets(path: Path) -> list[dict]:
    """Load presets.json: each preset stores a bipolar normalized (-1..1)
    coefficient per channel (see raspi/migrate_presets_to_normalized.py),
    calibration-independent by construction, so no MOTOR_LIMITS/offset
    lookup is needed here.
    """
    if not path.exists():
        return []

    raw = json.loads(path.read_text(encoding="utf-8"))
    presets: list[dict] = []
    for entry in raw.get("presets", []):
        preset_id = str(entry["id"])
        label = str(entry["label"])
        norm = [float(value) for value in entry["norm"]]
        if len(norm) != 32:
            raise RuntimeError(
                f"Expression preset '{preset_id}' must contain exactly 32 norm values, got {len(norm)}"
            )
        clamped = [clamp(value, -1.0, 1.0) for value in norm]
        if clamped != norm:
            print(
                f"WARNING: preset '{preset_id}' had out-of-range norm values, clamped to [-1, 1]"
            )
        presets.append({"id": preset_id, "label": label, "norm": clamped})

    return presets


def build_idle_behavior(module) -> dict:
    """Maps config.py's IDLE_BEHAVIOR (snake_case, matching control.rs's
    IdleBehaviorConfig field names 1:1) to the camelCase keys Rust's serde
    deserializer expects on the wire. Every key is optional on the Rust side
    (each has its own serde default), so a missing IDLE_BEHAVIOR dict, or a
    partially-filled one, still loads.
    """
    idle_behavior = getattr(module, "IDLE_BEHAVIOR", {})
    key_map = {
        "enabled": "enabled",
        "idle_after_seconds": "idleAfterSeconds",
        "noise_channel_ids": "noiseChannelIds",
        "noise_amplitude": "noiseAmplitude",
        "noise_freq_min_hz": "noiseFreqMinHz",
        "noise_freq_max_hz": "noiseFreqMaxHz",
        "blink_min_interval_seconds": "blinkMinIntervalSeconds",
        "blink_max_interval_seconds": "blinkMaxIntervalSeconds",
    }
    return {
        camel_key: idle_behavior[snake_key]
        for snake_key, camel_key in key_map.items()
        if snake_key in idle_behavior
    }


def main() -> None:
    module = load_module(CONFIG_PATH)
    if len(module.MOTOR_MAP) != 32:
        raise RuntimeError(f"Expected 32 motors, got {len(module.MOTOR_MAP)}")
    channels = [build_channel(module, motor_id) for motor_id in range(32)]

    payload = {
        "transport": {
            "host": "192.168.1.101",
            "port": int(getattr(module, "UDP_PORT", 6000)),
            "boardAddresses": list(getattr(module, "BOARD_ADDRESSES", [0x40, 0x41])),
        },
        "channels": channels,
        "jawCoupling": build_jaw_coupling(module),
        "expressionPresets": load_expression_presets(PRESETS_PATH),
        "externalInput": {
            "port": int(getattr(module, "EXTERNAL_INPUT_PORT", 6100)),
            "timeoutMs": int(getattr(module, "EXTERNAL_INPUT_TIMEOUT_MS", 500)),
        },
        "idleBehavior": build_idle_behavior(module),
    }

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"Wrote {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
