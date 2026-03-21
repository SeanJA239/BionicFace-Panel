from __future__ import annotations

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "raspi" / "config.py"
OUTPUT_PATH = ROOT / "src-tauri" / "config" / "motor_config.json"

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
    names = getattr(module, "MOTOR_NAMES", {})
    disabled_motors = set(getattr(module, "DISABLED_MOTORS", [30, 31]))
    board_addresses = list(getattr(module, "BOARD_ADDRESSES", [0x40, 0x41]))

    board, channel = motor_map[motor_id]
    min_applied, max_applied = limits.get(motor_id, (0, 180))
    offset = float(offsets.get(motor_id, 0))
    neutral_applied = (float(min_applied) + float(max_applied)) / 2.0
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


def main() -> None:
    module = load_module(CONFIG_PATH)
    if len(module.MOTOR_MAP) != 32:
        raise RuntimeError(f"Expected 32 motors, got {len(module.MOTOR_MAP)}")
    channels = [build_channel(module, motor_id) for motor_id in range(32)]

    payload = {
        "transport": {
            "host": "0.0.0.0",
            "port": int(getattr(module, "UDP_PORT", 6000)),
            "boardAddresses": list(getattr(module, "BOARD_ADDRESSES", [0x40, 0x41])),
        },
        "channels": channels,
    }

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"Wrote {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
