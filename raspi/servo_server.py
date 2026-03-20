from __future__ import annotations

import importlib.util
import logging
import os
from pathlib import Path
import signal
import time
from dataclasses import dataclass
from typing import Any

import zmq
from adafruit_servokit import ServoKit

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
LOGGER = logging.getLogger("servo-server")

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[2] / "xjtlu_BionicFace_hardware" / "config.py"


@dataclass(frozen=True)
class Motor:
    id: int
    name: str
    board: int
    channel: int
    min_angle: float
    max_angle: float
    zero_offset: float
    home_logical: float
    actuation_range: float


@dataclass(frozen=True)
class RuntimeConfig:
    bind: str
    pwm_frequency_hz: int
    home_step_delay_sec: float
    servo_release_on_exit: bool
    board_addresses: list[int]
    motors: dict[int, Motor]


def load_runtime_config() -> RuntimeConfig:
    config_path = Path(os.environ.get("BIONIC_FACE_CONFIG", DEFAULT_CONFIG_PATH)).resolve()
    LOGGER.info("Loading config from %s", config_path)
    if not config_path.exists():
        raise FileNotFoundError(f"Config not found: {config_path}")

    spec = importlib.util.spec_from_file_location("bionic_face_config", config_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Unable to load config module from {config_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    board_addresses = list(getattr(module, "PCA9685_ADDRESSES", [0x40, 0x41, 0x42]))
    bind = str(getattr(module, "ZMQ_BIND", "tcp://0.0.0.0:5555"))
    pwm_frequency_hz = int(getattr(module, "PWM_FREQUENCY_HZ", 50))
    home_step_delay_sec = float(getattr(module, "HOME_STEP_DELAY_SEC", 0.03))
    servo_release_on_exit = bool(getattr(module, "SERVO_RELEASE_ON_EXIT", False))

    if hasattr(module, "MOTORS"):
        motors = _normalize_rich_motor_config(module.MOTORS, board_addresses)
    elif hasattr(module, "MOTOR_MAP"):
        motor_safety = getattr(module, "MOTOR_SAFETY", {})
        motors = _normalize_legacy_motor_map(module.MOTOR_MAP, motor_safety, board_addresses)
    else:
        raise ValueError("Config must define MOTORS or MOTOR_MAP")

    return RuntimeConfig(
        bind=bind,
        pwm_frequency_hz=pwm_frequency_hz,
        home_step_delay_sec=home_step_delay_sec,
        servo_release_on_exit=servo_release_on_exit,
        board_addresses=board_addresses,
        motors=motors,
    )


def _normalize_rich_motor_config(motor_items: list[dict[str, Any]], board_addresses: list[int]) -> dict[int, Motor]:
    motors: dict[int, Motor] = {}
    used_ids: set[int] = set()
    used_slots: set[tuple[int, int]] = set()

    for item in motor_items:
        motor = Motor(**item)
        _validate_motor(motor, used_ids, used_slots, board_addresses)
        motors[motor.id] = motor

    if len(motors) != 32:
        raise ValueError(f"Expected 32 motors, got {len(motors)}")
    return motors


def _normalize_legacy_motor_map(
    motor_map: dict[int, tuple[int, int]],
    motor_safety: dict[int, dict[str, Any]],
    board_addresses: list[int],
) -> dict[int, Motor]:
    motors: dict[int, Motor] = {}
    used_ids: set[int] = set()
    used_slots: set[tuple[int, int]] = set()

    for motor_id, slot in motor_map.items():
        board, channel = slot
        safety = motor_safety.get(motor_id, {})
        motor = Motor(
            id=int(motor_id),
            name=str(safety.get("name", f"motor_{motor_id:02d}")),
            board=int(board),
            channel=int(channel),
            min_angle=float(safety.get("min_angle", 0.0)),
            max_angle=float(safety.get("max_angle", 180.0)),
            zero_offset=float(safety.get("zero_offset", 0.0)),
            home_logical=float(safety.get("home_logical", 90.0)),
            actuation_range=float(safety.get("actuation_range", 180.0)),
        )
        _validate_motor(motor, used_ids, used_slots, board_addresses)
        motors[motor.id] = motor

    if len(motors) != 32:
        raise ValueError(f"Expected 32 motors in MOTOR_MAP, got {len(motors)}")
    return motors


def _validate_motor(
    motor: Motor,
    used_ids: set[int],
    used_slots: set[tuple[int, int]],
    board_addresses: list[int],
) -> None:
    if motor.id < 0:
        raise ValueError(f"Invalid motor id: {motor.id}")
    if motor.id in used_ids:
        raise ValueError(f"Duplicate motor id: {motor.id}")
    slot = (motor.board, motor.channel)
    if slot in used_slots:
        raise ValueError(f"Duplicate board/channel slot: {slot}")
    if motor.board >= len(board_addresses):
        raise ValueError(f"Motor {motor.id} points to unknown board index {motor.board}")
    if not (0 <= motor.channel <= 15):
        raise ValueError(f"Motor {motor.id} has invalid channel {motor.channel}")
    if motor.min_angle > motor.max_angle:
        raise ValueError(f"Motor {motor.id} has inverted min/max")
    used_ids.add(motor.id)
    used_slots.add(slot)


class ServoRuntime:
    def __init__(self, runtime_config: RuntimeConfig) -> None:
        self.runtime_config = runtime_config
        self.motors = runtime_config.motors
        self.kits = self._build_kits()
        self.current_raw_angles: dict[int, float] = {}
        self._configure_servos()

    def _build_kits(self) -> dict[int, ServoKit]:
        kits: dict[int, ServoKit] = {}
        for board_index, address in enumerate(self.runtime_config.board_addresses):
            LOGGER.info("Initializing PCA9685 board=%s address=0x%02X", board_index, address)
            kits[board_index] = ServoKit(
                channels=16,
                address=address,
                frequency=self.runtime_config.pwm_frequency_hz,
            )
        return kits

    def _configure_servos(self) -> None:
        for motor in self.motors.values():
            self.kits[motor.board].servo[motor.channel].actuation_range = motor.actuation_range

    def _logical_to_raw(self, motor: Motor, logical_angle: float) -> tuple[float, bool]:
        raw = logical_angle + motor.zero_offset
        clamped = min(max(raw, motor.min_angle), motor.max_angle)
        return clamped, clamped != raw

    def write_motor(self, motor_id: int, logical_angle: float) -> dict[str, Any]:
        motor = self.motors[motor_id]
        applied_raw, was_clamped = self._logical_to_raw(motor, logical_angle)
        self.kits[motor.board].servo[motor.channel].angle = applied_raw
        self.current_raw_angles[motor_id] = applied_raw
        return {
            "motor_id": motor_id,
            "name": motor.name,
            "logical_angle": float(logical_angle),
            "applied_raw_angle": applied_raw,
            "clamped": was_clamped,
        }

    def move_home(self) -> list[dict[str, Any]]:
        LOGGER.info("Executing startup homing for %s motors", len(self.motors))
        results: list[dict[str, Any]] = []
        for motor_id in sorted(self.motors):
            motor = self.motors[motor_id]
            result = self.write_motor(motor_id, motor.home_logical)
            results.append(result)
            time.sleep(self.runtime_config.home_step_delay_sec)
        return results

    def release_all(self) -> None:
        if not self.runtime_config.servo_release_on_exit:
            return
        for motor in self.motors.values():
            self.kits[motor.board].servo[motor.channel].angle = None


def handle_message(runtime: ServoRuntime, payload: dict[str, Any]) -> dict[str, Any]:
    command = payload.get("command")

    if command == "ping":
        return {"ok": True, "command": "ping", "server_time_ns": time.time_ns()}

    if command == "home":
        results = runtime.move_home()
        return {
            "ok": True,
            "command": "home",
            "server_time_ns": time.time_ns(),
            "results": results,
        }

    if command == "set_one":
        motor_id = int(payload["motor_id"])
        logical_angle = float(payload["logical_angle"])
        if motor_id not in runtime.motors:
            raise ValueError(f"Unknown motor_id: {motor_id}")
        result = runtime.write_motor(motor_id, logical_angle)
        return {
            "ok": True,
            "command": "set_one",
            "server_time_ns": time.time_ns(),
            "result": result,
        }

    if command == "set_all":
        logical_angles = payload["logical_angles"]
        if not isinstance(logical_angles, list):
            raise ValueError("logical_angles must be a list")
        if len(logical_angles) != len(runtime.motors):
            raise ValueError(f"logical_angles must contain {len(runtime.motors)} values")

        results: list[dict[str, Any]] = []
        for motor_id, logical_angle in enumerate(logical_angles):
            results.append(runtime.write_motor(motor_id, float(logical_angle)))
        return {
            "ok": True,
            "command": "set_all",
            "server_time_ns": time.time_ns(),
            "results": results,
        }

    raise ValueError(f"Unsupported command: {command}")


def serve() -> None:
    runtime_config = load_runtime_config()
    runtime = ServoRuntime(runtime_config)
    runtime.move_home()

    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.setsockopt(zmq.LINGER, 0)
    socket.bind(runtime_config.bind)
    LOGGER.info("ZMQ REP server listening on %s", runtime_config.bind)

    stop = False

    def _request_stop(signum: int, _frame: Any) -> None:
        nonlocal stop
        LOGGER.info("Received signal %s, shutting down", signum)
        stop = True

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    try:
        while not stop:
            try:
                payload = socket.recv_json(flags=zmq.NOBLOCK)
            except zmq.Again:
                time.sleep(0.001)
                continue

            LOGGER.debug("RX %s", payload)
            try:
                reply = handle_message(runtime, payload)
            except Exception as exc:  # noqa: BLE001
                LOGGER.exception("Command failed")
                reply = {
                    "ok": False,
                    "error": str(exc),
                    "server_time_ns": time.time_ns(),
                }
            socket.send_json(reply)
    finally:
        runtime.release_all()
        socket.close()
        context.term()


if __name__ == "__main__":
    serve()
