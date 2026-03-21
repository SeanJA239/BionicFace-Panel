from __future__ import annotations

import importlib.util
import json
import logging
import os
import signal
import socket
from pathlib import Path
from typing import Any

from adafruit_servokit import ServoKit


logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
LOGGER = logging.getLogger("udp-servo-executor")

DEFAULT_CONFIG_PATH = Path(__file__).resolve().with_name("config.py")
DEFAULT_BOARD_ADDRESSES = [0x40, 0x41]
DEFAULT_UDP_PORT = 6000


def load_module(path: Path):
    spec = importlib.util.spec_from_file_location("raspi_config", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load config from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_kits(module) -> dict[int, ServoKit]:
    board_addresses = list(getattr(module, "BOARD_ADDRESSES", DEFAULT_BOARD_ADDRESSES))
    motor_limits = getattr(module, "MOTOR_LIMITS", {})
    motor_map = module.MOTOR_MAP

    kits = {
        index: ServoKit(channels=16, address=address, frequency=50)
        for index, address in enumerate(board_addresses)
    }

    for motor_id, (board, channel) in motor_map.items():
        max_limit = int(motor_limits.get(motor_id, (0, 180))[1])
        kits[board].servo[channel].actuation_range = max(180, max_limit)

    return kits


def apply_angles(kits: dict[int, ServoKit], motor_map: dict[int, tuple[int, int]], angles: list[float]) -> None:
    for motor_id, angle in enumerate(angles):
        board, channel = motor_map[motor_id]
        kits[board].servo[channel].angle = angle


def main() -> None:
    config_path = Path(os.environ.get("BIONIC_FACE_CONFIG", DEFAULT_CONFIG_PATH)).resolve()
    module = load_module(config_path)
    kits = build_kits(module)
    motor_map = module.MOTOR_MAP
    udp_port = int(getattr(module, "UDP_PORT", DEFAULT_UDP_PORT))

    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(("0.0.0.0", udp_port))
    server.settimeout(0.2)
    LOGGER.info("UDP executor listening on 0.0.0.0:%s", udp_port)

    stop = False

    def request_stop(signum: int, _frame: Any) -> None:
        nonlocal stop
        LOGGER.info("Received signal %s, shutting down", signum)
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    while not stop:
        try:
            packet, _addr = server.recvfrom(65535)
        except socket.timeout:
            continue

        try:
            payload = json.loads(packet.decode("utf-8"))
            angles = payload["angles"]
            if len(angles) != 32:
                continue
            apply_angles(kits, motor_map, angles)
        except Exception as exc:  # noqa: BLE001
            LOGGER.warning("Skipping invalid packet: %s", exc)

    server.close()


if __name__ == "__main__":
    main()
