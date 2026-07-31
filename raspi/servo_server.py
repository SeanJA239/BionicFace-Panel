from __future__ import annotations

import argparse
import importlib.util
import json
import logging
import os
import signal
import socket
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from adafruit_servokit import ServoKit

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s"
)
LOGGER = logging.getLogger("udp-servo-executor")

DEFAULT_CONFIG_PATH = Path(__file__).resolve().with_name("config.py")
DEFAULT_BOARD_ADDRESSES = [0x40, 0x41]
DEFAULT_UDP_PORT = 6000
DRY_RUN_REPORT_INTERVAL_S = 1.0


def load_module(path: Path):
    spec = importlib.util.spec_from_file_location("raspi_config", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load config from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_kits(module) -> dict[int, ServoKit]:
    # Imported here, not at module scope, so --dry-run can run on any PC
    # without adafruit-servokit (and its Blinka/I2C backend) installed.
    from adafruit_servokit import ServoKit

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


class DryRunReporter:
    """Tracks frame throughput and the latest frame without touching I2C.

    Prints at most once per DRY_RUN_REPORT_INTERVAL_S regardless of incoming
    frame rate, so a 100Hz heartbeat doesn't flood the terminal.
    """

    def __init__(self, interval: float = DRY_RUN_REPORT_INTERVAL_S) -> None:
        self.interval = interval
        self.frame_count = 0
        self.last_report = time.monotonic()

    def record(self, payload: dict[str, Any]) -> None:
        self.frame_count += 1
        now = time.monotonic()
        elapsed = now - self.last_report
        if elapsed < self.interval:
            return

        fps = self.frame_count / elapsed
        angles = payload.get("angles", [])
        angles_summary = ", ".join(f"{angle:.1f}" for angle in angles)
        LOGGER.info(
            "[dry-run] %.1f fps | frameId=%s source=%s | angles=[%s]",
            fps,
            payload.get("frameId"),
            payload.get("source"),
            angles_summary,
        )
        self.frame_count = 0
        self.last_report = now


def apply_angles(
    kits: dict[int, ServoKit],
    motor_map: dict[int, tuple[int, int]],
    angles: list[float],
    last_angles: list[float | None],
) -> None:
    # Each .angle assignment is several I2C register writes; skipping
    # unchanged channels keeps the bus from saturating at high frame rates.
    for motor_id, angle in enumerate(angles):
        if last_angles[motor_id] == angle:
            continue
        board, channel = motor_map[motor_id]
        kits[board].servo[channel].angle = angle
        last_angles[motor_id] = angle


def drain_to_latest(server: socket.socket, packet: bytes) -> bytes:
    # I2C writes are slower than the incoming frame rate can be; only the
    # newest queued frame matters, older ones are stale targets.
    server.setblocking(False)
    try:
        while True:
            packet, _addr = server.recvfrom(65535)
    except BlockingIOError:
        pass
    finally:
        server.settimeout(0.2)
    return packet


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="BionicFace UDP servo executor")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help=(
            "Skip adafruit-servokit and all I2C writes; just log received "
            "frame rate/summary once a second. Runs on any PC as a mock "
            "executor for tools/face_visualizer.py-style development."
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config_path = Path(
        os.environ.get("BIONIC_FACE_CONFIG", DEFAULT_CONFIG_PATH)
    ).resolve()
    module = load_module(config_path)
    motor_map = module.MOTOR_MAP
    udp_port = int(getattr(module, "UDP_PORT", DEFAULT_UDP_PORT))

    kits = None if args.dry_run else build_kits(module)
    reporter = DryRunReporter() if args.dry_run else None

    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(("0.0.0.0", udp_port))
    server.settimeout(0.2)
    LOGGER.info(
        "UDP executor listening on 0.0.0.0:%s%s",
        udp_port,
        " (dry-run, no I2C)" if args.dry_run else "",
    )

    stop = False

    def request_stop(signum: int, _frame: Any) -> None:
        nonlocal stop
        LOGGER.info("Received signal %s, shutting down", signum)
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    last_angles: list[float | None] = [None] * 32

    while not stop:
        try:
            packet, _addr = server.recvfrom(65535)
        except TimeoutError:
            continue

        packet = drain_to_latest(server, packet)

        try:
            payload = json.loads(packet.decode("utf-8"))
            angles = payload["angles"]
            if len(angles) != 32:
                continue
            if reporter is not None:
                reporter.record(payload)
            else:
                apply_angles(kits, motor_map, angles, last_angles)
        except Exception as exc:  # noqa: BLE001
            LOGGER.warning("Skipping invalid packet: %s", exc)

    server.close()


if __name__ == "__main__":
    main()
