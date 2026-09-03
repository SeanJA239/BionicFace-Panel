"""Self-modelling dataset collection: random coefficients in, landmarks out.

Sends random coefficient frames to ControlService's external input port, waits
for the pose to actually settle, then photographs the result and stores the
landmarks alongside the command that produced them.

Two things this deliberately does not do. It never talks to the Pi or to any
servo directly -- everything goes through the external input port, so Rust's
clamp/coupling/rate-limit path stays the single authority. And it does not
sleep a fixed time per sample: it watches the frame log until motion stops, so
a big pose change gets the time it needs and a small one does not waste any.

Idle noise and blink do not need to be switched off. Holding the external
source suppresses them for as long as frames keep arriving, which is exactly
the collection window (see control.rs's update_idle_behavior).

**During the no-hardware phase the two halves are decoupled**: commands go to
the digital twin while the images are of whatever the camera sees, typically a
human stand-in that does not respond to them. Such samples are pipeline
validation, not trainable data, so every sample records `subject`; filter on it
(or on `session`) before training on anything.

Usage:
    python tools/collect_dataset.py --samples 50 --subject human-standin
    python tools/collect_dataset.py --samples 1000 --subject robot
"""

from __future__ import annotations

import argparse
import json
import random
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2

from camera_capture import Camera, CameraError, CaptureConfig
from facemesh import DEFAULT_MODEL, LandmarkExtractor
from mediapipe_driver import MOTOR_COUNT, build_frame

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MOTOR_CONFIG = REPO_ROOT / "src-tauri" / "config" / "motor_config.json"
DEFAULT_FRAME_LOG = REPO_ROOT / "logs" / "udp_frames.jsonl"
DEFAULT_OUT = REPO_ROOT / "dataset"
HOLD_HZ = 25.0


def participating_channels(motor_config: dict[str, Any]) -> list[int]:
    """The channels worth sampling: independent, enabled, not a follower.

    Excluded by default:
      * disabled channels (the tongue pair, per the config's own `enabled`)
      * the neck, which is frozen for capture -- head pose is not what is being
        modelled and moving it would change the camera geometry mid-dataset
      * the jaw coupling's slaves. Note the coupling's master is channel 26 with
        27 following it, so 26 *is* sampled and 27 is not -- the plan's "26/27
        are both followers" does not match jawCoupling in motor_config.json.
    """
    coupling = motor_config.get("jawCoupling") or {}
    followers = {int(k) for k in (coupling.get("slaveRatios") or {})}
    neck = {
        channel["id"]
        for channel in motor_config["channels"]
        if channel.get("name", "").startswith("neck")
    }
    return [
        channel["id"]
        for channel in motor_config["channels"]
        if channel.get("enabled", True) and channel["id"] not in followers | neck
    ]


class TargetHold:
    """Re-sends the current target so the external source stays claimed.

    One frame per sample would work for the pose itself, but the source falls
    back to Manual after its timeout, which would let idle noise back in
    mid-capture. Holding at a steady rate keeps External asserted throughout.
    """

    def __init__(self, host: str, port: int) -> None:
        self._addr = (host, port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._target: list[float | None] = [None] * MOTOR_COUNT
        self._seq = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def set(self, coefficients: list[float | None]) -> None:
        with self._lock:
            self._target = list(coefficients)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._socket.close()

    def _run(self) -> None:
        interval = 1.0 / HOLD_HZ
        while not self._stop.is_set():
            with self._lock:
                self._seq += 1
                payload = build_frame(self._seq, self._target)
            self._socket.sendto(json.dumps(payload).encode("utf-8"), self._addr)
            time.sleep(interval)


class FrameLogTail:
    """Follows ControlService's UDP frame log to see what the pose actually did.

    The log is the only place an outside process can read the applied angles --
    the runtime state is behind Tauri IPC. Only frames where something moved are
    written, so "no new line for a while" means the interpolation has settled,
    and the last line carries the angles it settled on.
    """

    def __init__(self, path: Path) -> None:
        self._path = path
        self._handle = path.open("r", encoding="utf-8")
        self._handle.seek(0, 2)  # a long-running app leaves a huge log; skip it
        self._last: dict[str, Any] | None = None

    def close(self) -> None:
        self._handle.close()

    def drain(self) -> int:
        """Reads whatever is new; returns how many lines that was."""
        count = 0
        for line in self._handle:
            line = line.strip()
            if not line:
                continue
            try:
                self._last = json.loads(line)
            except json.JSONDecodeError:
                continue  # a partially flushed line; it will be re-read whole
            count += 1
        return count

    @property
    def angles(self) -> list[float] | None:
        return None if self._last is None else self._last.get("angles")

    def wait_until_reached(
        self, expected: dict[int, float], tolerance: float, timeout: float
    ) -> bool:
        """Waits until the logged angles match `expected` on every given channel.

        Waiting for the log to merely go quiet does not work: ControlService
        buffers the frame log and flushes it when motion settles *or* on a
        periodic timer, so mid-move the file can sit unchanged long enough to
        look settled, and a periodic flush mid-move looks like fresh activity.
        Comparing against the commanded angles sidesteps the buffering entirely
        -- convergence is the thing actually being waited for.

        Returns False on timeout, in which case the caller should record the
        sample as unsettled rather than trusting the pose.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.drain()
            angles = self.angles
            if angles is not None and all(
                abs(angles[channel_id] - value) <= tolerance
                for channel_id, value in expected.items()
            ):
                return True
            time.sleep(0.005)
        return False


@dataclass
class Session:
    """One collection run's identity and provenance."""

    session_id: str
    subject: str
    channels: list[int]
    camera_fingerprint: dict[str, int] = field(default_factory=dict)
    negotiated: dict[str, Any] = field(default_factory=dict)

    def manifest(self) -> dict[str, Any]:
        return {
            "session": self.session_id,
            "subject": self.subject,
            "sampled_channels": self.channels,
            "camera_fingerprint": self.camera_fingerprint,
            "camera_format": self.negotiated,
            "motor_count": MOTOR_COUNT,
        }


def expected_angles(
    coefficients: list[float | None], motor_config: dict[str, Any]
) -> dict[int, float]:
    """Where each commanded channel should end up, in applied degrees.

    Mirrors the external input port's conversion in control.rs: the coefficient
    is flat unipolar across the channel's whole applied range. Used only to
    decide when the pose has converged, and a mismatch surfaces as an unsettled
    sample rather than being silently accepted.
    """
    channels = {channel["id"]: channel for channel in motor_config["channels"]}
    expected = {}
    for channel_id, coefficient in enumerate(coefficients):
        if coefficient is None:
            continue
        channel = channels[channel_id]
        low, high = channel["minApplied"], channel["maxApplied"]
        expected[channel_id] = low + coefficient * (high - low)
    return expected


def random_coefficients(channels: list[int], rng: random.Random) -> list[float | None]:
    coefficients: list[float | None] = [None] * MOTOR_COUNT
    for channel_id in channels:
        coefficients[channel_id] = round(rng.random(), 4)
    return coefficients


def collect(args: argparse.Namespace) -> int:
    motor_config = json.loads(args.motor_config.read_text(encoding="utf-8"))
    channels = (
        [int(c) for c in args.channels.split(",")]
        if args.channels
        else participating_channels(motor_config)
    )
    config = CaptureConfig.load(args.config)
    rng = random.Random(args.seed)

    session_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = args.out / session_id
    (out_dir / "images").mkdir(parents=True, exist_ok=True)

    if not args.frame_log.exists():
        raise CameraError(
            f"no frame log at {args.frame_log}. ControlService writes it while "
            f"anything is moving, so start the panel app first."
        )

    hold = TargetHold(args.host, args.port)
    tail = FrameLogTail(args.frame_log)
    written = 0
    undetected = 0
    try:
        with Camera(config) as camera:
            locked = camera.lock_params()
            session = Session(
                session_id=session_id,
                subject=args.subject,
                channels=channels,
                camera_fingerprint=locked,
                negotiated=camera.negotiated_format(),
            )
            (out_dir / "manifest.json").write_text(
                json.dumps(session.manifest(), indent=2) + "\n", encoding="utf-8"
            )
            print(f"session {session_id} -> {out_dir}")
            print(f"sampling {len(channels)} channels: {channels}")
            print(f"subject: {args.subject}\n")

            hold.start()
            with LandmarkExtractor(args.model) as extractor:
                meta_path = out_dir / "samples.jsonl"
                with meta_path.open("w", encoding="utf-8") as meta:
                    for index in range(args.samples):
                        target = random_coefficients(channels, rng)
                        tail.drain()
                        hold.set(target)
                        expected = expected_angles(target, motor_config)
                        settled = tail.wait_until_reached(
                            expected,
                            args.settle_tolerance_deg,
                            args.settle_timeout_ms / 1000.0,
                        )
                        if tail.angles is None:
                            # A fresh random target across this many channels
                            # always moves something, so an untouched log means
                            # nobody is consuming the frames -- the panel app is
                            # not running, or it is bound elsewhere. Failing here
                            # beats writing a session full of null angles.
                            raise CameraError(
                                f"no motion appeared in {args.frame_log} after "
                                f"commanding sample {index}. Is the panel app "
                                f"running, and is its external input port "
                                f"{args.port}?"
                            )
                        time.sleep(args.extra_ms / 1000.0)

                        frame = camera.grab()
                        landmarks = extractor.detect(frame.image)
                        if landmarks is None:
                            undetected += 1

                        name = f"{index:05d}.jpg"
                        cv2.imwrite(
                            str(out_dir / "images" / name),
                            frame.image,
                            [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality],
                        )
                        meta.write(
                            json.dumps(
                                {
                                    "id": index,
                                    "session": session_id,
                                    "subject": args.subject,
                                    "timestamp_ns": time.time_ns(),
                                    "grab_monotonic": frame.timestamp,
                                    "target_coefficients": target,
                                    "applied_angles": tail.angles,
                                    "settled": settled,
                                    "landmark_count": 0
                                    if landmarks is None
                                    else len(landmarks),
                                    "landmarks": None
                                    if landmarks is None
                                    else [
                                        [round(float(x), 3), round(float(y), 3)]
                                        for x, y in landmarks
                                    ],
                                    "image": f"images/{name}",
                                }
                            )
                            + "\n"
                        )
                        written += 1
                        if not settled:
                            print(
                                f"  sample {index}: pose never settled, recorded anyway"
                            )
                        if (index + 1) % 10 == 0:
                            print(
                                f"  {index + 1}/{args.samples} ({undetected} without a face)"
                            )
    finally:
        hold.close()
        tail.close()

    print(f"\nwrote {written} samples to {out_dir}")
    if undetected:
        print(f"{undetected} had no detectable face -- landmarks are null on those")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--samples", type=int, default=50)
    parser.add_argument(
        "--subject",
        required=True,
        help=(
            "What the camera is actually looking at, e.g. 'robot' or "
            "'human-standin'. Recorded per sample: stand-in images do not respond "
            "to the commands, so those sessions are not trainable data."
        ),
    )
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument(
        "--config", type=Path, default=REPO_ROOT / "tools" / "camera_params.json"
    )
    parser.add_argument("--motor-config", type=Path, default=DEFAULT_MOTOR_CONFIG)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--frame-log", type=Path, default=DEFAULT_FRAME_LOG)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=6100)
    parser.add_argument("--channels", default=None, help="comma-separated override")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument(
        "--settle-tolerance-deg",
        type=float,
        default=0.5,
        help="how close the applied angles must get to the command to count as settled",
    )
    parser.add_argument("--settle-timeout-ms", type=float, default=4000.0)
    parser.add_argument("--extra-ms", type=float, default=300.0)
    parser.add_argument("--jpeg-quality", type=int, default=92)
    args = parser.parse_args(argv)

    try:
        return collect(args)
    except (CameraError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
