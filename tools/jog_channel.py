"""Single-channel jog: one channel moves, every other one holds neutral.

The safety front-end for first power-on, and the instrument for two
measurements that only the bare mechanism can give up: which anatomical feature
each channel actually drives (and in which direction), and each channel's
linkage ratio -- degrees commanded against millimetres the linkage endpoint
travels. Once the skin is on, neither is observable.

ControlService sends all 32 channels every frame; there is no single-channel
transport and this does not add one. "Jogging one channel" here means sending a
full frame in which every other channel sits at its neutral coefficient, so the
mechanism only ever holds a pose that was asked for explicitly.

Everything goes over the external input port, so Rust stays the only authority
on the coefficient->angle conversion, limit clamping, jaw coupling and rate
limiting. Nothing here recomputes any of that: the angle this tool displays is
read back out of ControlService's frame log, not derived locally. The one piece
of arithmetic it does own is the --step-deg conversion, and only because the
external port's mapping is flat unipolar across the whole applied range, which
makes `d_coefficient = d_degrees / (max - min)` exact rather than an
approximation. The bipolar preset math, clamping, coupling and rate limiting
all stay where they are.

Usage:
    python tools/jog_channel.py
    python tools/jog_channel.py --step-deg 2.0        # jog in degrees
    python tools/jog_channel.py --channel 11          # start on a channel
"""

from __future__ import annotations

import argparse
import json
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from collect_dataset import (
    DEFAULT_FRAME_LOG,
    DEFAULT_MOTOR_CONFIG,
    MOTOR_COUNT,
    FrameLogTail,
    TargetHold,
)

# How close a channel has to sit to neutral to count as parked. The rate limiter
# walks channels in at 2 deg/tick, so this is a settled-state tolerance.
NEUTRAL_TOLERANCE_DEG = 0.5
STARTUP_TIMEOUT_S = 4.0
DEFAULT_STEP_COEFF = 0.01
# A limit jump has to be confirmed by a second press inside this window.
CONFIRM_WINDOW_S = 3.0


@dataclass(frozen=True)
class Channel:
    """One servo channel's identity and calibrated range."""

    id: int
    name: str
    min_applied: float
    neutral_applied: float
    max_applied: float
    enabled: bool

    @property
    def span(self) -> float:
        return self.max_applied - self.min_applied

    @property
    def neutral_coefficient(self) -> float:
        """Where neutral sits on the external port's flat unipolar scale."""
        if self.span <= 1e-6:
            return 0.5
        return (self.neutral_applied - self.min_applied) / self.span


@dataclass(frozen=True)
class Coupling:
    """Which channel drives which, so followers are neither commanded nor
    reported as strays when their master is the one being jogged."""

    master: int | None
    followers: frozenset[int]


def load_config(path: Path) -> tuple[list[Channel], Coupling]:
    config: dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))
    channels = [
        Channel(
            id=entry["id"],
            name=entry.get("name") or f"channel {entry['id']}",
            min_applied=float(entry["minApplied"]),
            neutral_applied=float(entry["neutralApplied"]),
            max_applied=float(entry["maxApplied"]),
            enabled=bool(entry.get("enabled", True)),
        )
        for entry in config["channels"]
    ]
    if len(channels) != MOTOR_COUNT:
        raise SystemExit(f"expected {MOTOR_COUNT} channels, {path} has {len(channels)}")
    jaw = config.get("jawCoupling", {})
    master = jaw.get("masterMotorId")
    coupling = Coupling(
        master=None if master is None else int(master),
        followers=frozenset(int(k) for k in jaw.get("slaveRatios", {})),
    )
    return channels, coupling


class RawTerminal:
    """cbreak mode for single-keypress reads.

    cbreak rather than raw, so Ctrl-C keeps working -- this drives servos.
    """

    def __init__(self) -> None:
        self._fd = sys.stdin.fileno()
        self._saved: Any = None

    def __enter__(self) -> None:
        self._saved = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)

    def __exit__(self, *_exc: object) -> None:
        if self._saved is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)


ARROWS = {"[A": "UP", "[B": "DOWN", "[C": "RIGHT", "[D": "LEFT"}


def read_key(timeout: float) -> str | None:
    """One keypress, or None if nothing arrived inside `timeout`."""
    if not select.select([sys.stdin], [], [], timeout)[0]:
        return None
    char = sys.stdin.read(1)
    if char != "\x1b":
        return char
    if not select.select([sys.stdin], [], [], 0.05)[0]:
        return "ESC"
    return ARROWS.get(sys.stdin.read(2), "")


class Jogger:
    """Holds one channel off neutral and reports what the mechanism did."""

    def __init__(
        self,
        channels: list[Channel],
        coupling: Coupling,
        hold: TargetHold,
        step_deg: float | None,
        step_coeff: float,
    ) -> None:
        self.channels = channels
        self.coupling = coupling
        self.hold = hold
        self.step_deg = step_deg
        self.step_coeff = step_coeff
        self.selectable = [c.id for c in channels if c.id not in coupling.followers]
        self.coefficients: list[float | None] = [
            None if c.id in coupling.followers else c.neutral_coefficient
            for c in channels
        ]
        self.selected = self.selectable[0]
        self.pending_limit: tuple[str, float] | None = None
        self.digits = ""
        self.message = ""

    @property
    def channel(self) -> Channel:
        return self.channels[self.selected]

    def push(self) -> None:
        self.hold.set(self.coefficients)

    def step_for(self, channel: Channel) -> float:
        """Jog step in coefficient units, recomputed per channel in degree mode."""
        if self.step_deg is None:
            return self.step_coeff
        return 0.0 if channel.span <= 1e-6 else self.step_deg / channel.span

    def jog(self, direction: int) -> None:
        current = self.coefficients[self.selected]
        if current is None:
            return
        step = self.step_for(self.channel)
        self.coefficients[self.selected] = min(
            1.0, max(0.0, current + direction * step)
        )
        self.pending_limit = None
        self.push()

    def select(self, offset: int) -> None:
        index = self.selectable.index(self.selected)
        self.selected = self.selectable[(index + offset) % len(self.selectable)]
        self.pending_limit = None
        self.message = ""

    def digit(self, char: str) -> None:
        self.digits = (self.digits + char)[-2:]
        self.message = (
            f"channel {self.digits}_ -- Enter to select, any other key cancels"
        )

    def commit_digits(self) -> None:
        if not self.digits:
            return
        channel_id = int(self.digits)
        self.digits = ""
        if not 0 <= channel_id < MOTOR_COUNT:
            self.message = f"no channel {channel_id}"
        elif channel_id in self.coupling.followers:
            self.message = (
                f"ch{channel_id} is a coupling follower; driven from its master"
            )
        else:
            self.selected = channel_id
            self.pending_limit = None
            self.message = ""

    def cancel_digits(self) -> None:
        if self.digits:
            self.digits = ""
            self.message = ""

    def scale_step(self, factor: float) -> None:
        if self.step_deg is not None:
            self.step_deg = max(0.05, min(20.0, self.step_deg * factor))
        else:
            self.step_coeff = max(0.0005, min(0.25, self.step_coeff * factor))

    def to_neutral(self, every: bool) -> None:
        for channel_id in self.selectable if every else [self.selected]:
            self.coefficients[channel_id] = self.channels[
                channel_id
            ].neutral_coefficient
        self.pending_limit = None
        self.message = "every channel back to neutral" if every else ""
        self.push()

    def request_limit(self, which: str) -> None:
        """Limit jumps are two-step: min/max are hand-set and never verified."""
        now = time.monotonic()
        pending = self.pending_limit
        if pending and pending[0] == which and now - pending[1] <= CONFIRM_WINDOW_S:
            self.coefficients[self.selected] = 0.0 if which == "min" else 1.0
            self.pending_limit = None
            self.message = f"sent ch{self.selected} to {which}"
            self.push()
            return
        self.pending_limit = (which, now)
        target = (
            self.channel.min_applied if which == "min" else self.channel.max_applied
        )
        self.message = (
            f"press {'m' if which == 'min' else 'M'} again to send ch{self.selected} "
            f"to {which} ({target:.1f} deg) -- limits are hand-set and unverified"
        )

    def expected_movers(self) -> set[int]:
        """The selected channel, plus any follower its master drives."""
        movers = {self.selected}
        if self.coupling.master == self.selected:
            movers |= set(self.coupling.followers)
        return movers

    def strays(self, angles: list[float]) -> list[str]:
        """Channels sitting off neutral that were never asked to move."""
        movers = self.expected_movers()
        strays = []
        for channel in self.channels:
            if channel.id in movers:
                continue
            deviation = angles[channel.id] - channel.neutral_applied
            if abs(deviation) <= NEUTRAL_TOLERANCE_DEG:
                continue
            strays.append(f"ch{channel.id} {deviation:+.1f}")
        return strays


class Display:
    """Repaints a fixed status block in place."""

    def __init__(self) -> None:
        self.height = 0

    def paint(self, jogger: Jogger, angles: list[float] | None) -> None:
        channel = jogger.channel
        coefficient = jogger.coefficients[jogger.selected] or 0.0
        commanded = channel.min_applied + coefficient * channel.span
        step = jogger.step_for(channel)
        step_label = (
            f"{jogger.step_deg:.2f} deg = {step:.4f} coeff"
            if jogger.step_deg is not None
            else f"{jogger.step_coeff:.4f} coeff = {step * channel.span:.2f} deg"
        )

        observed = "observed      --"
        if angles is not None:
            actual = angles[jogger.selected]
            observed = f"observed {actual:7.1f}   delta {actual - commanded:+6.2f}"

        if angles is None:
            others = "waiting for the panel to log angles"
        else:
            strays = jogger.strays(angles)
            movers = sorted(jogger.expected_movers() - {jogger.selected})
            note = f"  (ch{movers[0]} follows by coupling)" if movers else ""
            others = (
                f"all {len(jogger.channels) - len(jogger.expected_movers())} "
                f"others at neutral{note}"
                if not strays
                else f"{len(strays)} off neutral: " + ", ".join(strays[:4])
            )

        lines = [
            f"ch{channel.id:<3d} {channel.name}"
            + ("" if channel.enabled else "    [DISABLED in config]"),
            (
                f"  range    min {channel.min_applied:7.1f}    neutral "
                f"{channel.neutral_applied:7.1f}    max {channel.max_applied:7.1f}"
            ),
            f"  command  coeff {coefficient:6.4f} -> {commanded:7.1f} deg    {observed}",
            f"  step     {step_label}",
            f"  others   {others}",
            f"  {jogger.message}",
            (
                "  up/down channel   left/right jog   digits+Enter goto"
                "   n neutral   N all   [ ] step   m/M limit   q quit"
            ),
        ]

        if self.height:
            sys.stdout.write(f"\x1b[{self.height}A")
        for line in lines:
            sys.stdout.write("\x1b[2K" + line + "\n")
        sys.stdout.flush()
        self.height = len(lines)


def wait_for_angles(tail: FrameLogTail, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        tail.drain()
        if tail.angles is not None:
            return True
        time.sleep(0.02)
    return False


def run(args: argparse.Namespace) -> int:
    channels, coupling = load_config(args.motor_config)
    if not args.frame_log.exists():
        raise SystemExit(
            f"{args.frame_log} does not exist. Start the panel app first -- its "
            f"frame log is where this tool reads the real angles back from."
        )

    hold = TargetHold(args.host, args.port)
    tail = FrameLogTail(args.frame_log)
    jogger = Jogger(channels, coupling, hold, args.step_deg, args.step)
    if args.channel is not None:
        jogger.digits = str(args.channel)
        jogger.commit_digits()
        if jogger.selected != args.channel:
            raise SystemExit(jogger.message)

    hold.start()
    jogger.push()  # the very first frame is every channel at neutral

    if not wait_for_angles(tail, STARTUP_TIMEOUT_S):
        hold.close()
        tail.close()
        raise SystemExit(
            f"no angles appeared in {args.frame_log} within {STARTUP_TIMEOUT_S:.0f}s. "
            f"Is the panel app running, and is its external input port {args.port}?"
        )

    print("holding every channel at neutral; jog one at a time\n")
    display = Display()
    try:
        with RawTerminal():
            while True:
                tail.drain()
                display.paint(jogger, tail.angles)

                key = read_key(0.1)
                if key is None:
                    continue
                if key in {"q", "Q", "ESC", "\x03"}:
                    break
                if key.isdigit():
                    jogger.digit(key)
                    continue
                if key in {"\r", "\n"}:
                    jogger.commit_digits()
                    continue
                jogger.cancel_digits()
                if key in {"UP", "k"}:
                    jogger.select(-1)
                elif key in {"DOWN", "j"}:
                    jogger.select(1)
                elif key in {"RIGHT", "+", "="}:
                    jogger.jog(1)
                elif key in {"LEFT", "-", "_"}:
                    jogger.jog(-1)
                elif key == "n":
                    jogger.to_neutral(every=False)
                elif key == "N":
                    jogger.to_neutral(every=True)
                elif key == "m":
                    jogger.request_limit("min")
                elif key == "M":
                    jogger.request_limit("max")
                elif key == "]":
                    jogger.scale_step(2.0)
                elif key == "[":
                    jogger.scale_step(0.5)
    finally:
        jogger.to_neutral(every=True)
        # Let the rate limiter walk everything home before the source is dropped
        # and idle behaviour takes over.
        time.sleep(1.0)
        hold.close()
        tail.close()
        print("\nreturned every channel to neutral")
    return 0


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=6100)
    parser.add_argument("--motor-config", type=Path, default=DEFAULT_MOTOR_CONFIG)
    parser.add_argument("--frame-log", type=Path, default=DEFAULT_FRAME_LOG)
    parser.add_argument("--channel", type=int, default=None, help="channel to start on")
    parser.add_argument(
        "--step",
        type=float,
        default=DEFAULT_STEP_COEFF,
        help=f"jog step in coefficient units (default {DEFAULT_STEP_COEFF})",
    )
    parser.add_argument(
        "--step-deg",
        type=float,
        default=None,
        help="jog in applied degrees instead, for linkage-ratio measurement",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
