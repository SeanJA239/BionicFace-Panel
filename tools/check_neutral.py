"""Checks the driver's mapping table against each channel's calibrated neutral.

Two things decide whether a channel behaves, and this reports both.

**Where it rests.** At a neutral face every mapped blendshape score is ~0, so the
driver's output is exactly the sum of its bias terms. That has to land on the
channel's calibrated neutral -- `neutralApplied`, which is `norm = 0` by
definition -- or the face sits in a subtly wrong pose the moment the driver takes
over, and every expression is measured from the wrong baseline.

**Whether it can move.** A blendshape score of 1.0 pushes the coefficient by the
sum of that direction's weights. If that push exceeds the travel left between the
neutral and the limit, the excursion clamps: the expression stops developing part
way and looks stuck. Deviation alone does not catch this, and it is the failure
that actually shows on the face.

Channels are classified from the mapping itself: a channel with both positive and
negative weights is bidirectional and needs room on both sides; one with weights
of a single sign is unidirectional and only needs room the way it pushes.

No preset is involved. The reference is the mechanical calibration, so it stays
valid as long as the channel limits do.

Usage:
    python tools/check_neutral.py [--config src-tauri/config/motor_config.json]
                                  [--tolerance 0.05]

Exits 1 if any driven channel misses its neutral or would clamp.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = REPO_ROOT / "src-tauri" / "config" / "motor_config.json"
DEFAULT_TOLERANCE = 0.05
# Weights are stored rounded to three decimals, so a direction's weights can sum
# to a hair over its remaining travel without meaning anything. Only report a
# clamp bigger than that rounding noise.
CLAMP_EPSILON = 1e-3


@dataclass(frozen=True)
class Row:
    """One channel's neutral and travel check."""

    channel_id: int
    name: str
    enabled: bool
    neutral: float | None  # coefficient the calibrated neutral corresponds to
    actual: float | None  # what the mapping emits at a neutral face
    push_up: float  # coefficient excursion a full blendshape score drives upward
    push_down: float  # ... and downward

    @property
    def mapped(self) -> bool:
        return self.actual is not None

    @property
    def driven(self) -> bool:
        return self.enabled and self.mapped

    @property
    def bidirectional(self) -> bool:
        return self.push_up > 0.0 and self.push_down > 0.0

    @property
    def deviation(self) -> float | None:
        if self.neutral is None or self.actual is None:
            return None
        return self.actual - self.neutral

    @property
    def clamped_up(self) -> float:
        """How much of the upward excursion is lost to the ceiling."""
        if self.neutral is None or self.push_up <= 0.0:
            return 0.0
        return max(0.0, self.push_up - (1.0 - self.neutral))

    @property
    def clamped_down(self) -> float:
        if self.neutral is None or self.push_down <= 0.0:
            return 0.0
        return max(0.0, self.push_down - self.neutral)

    def problems(self, tolerance: float) -> list[str]:
        if not self.driven:
            return []
        issues = []
        deviation = self.deviation
        if deviation is not None and abs(deviation) > tolerance:
            issues.append(f"neutral off by {deviation:+.3f}")
        if self.clamped_up > CLAMP_EPSILON:
            issues.append(f"clamps {self.clamped_up:.3f} going up")
        if self.clamped_down > CLAMP_EPSILON:
            issues.append(f"clamps {self.clamped_down:.3f} going down")
        return issues


def neutral_targets(config: dict[str, Any]) -> list[float | None]:
    """The unipolar coefficient each channel's calibrated neutral sits at.

    `norm = 0` is `neutralApplied` by definition, so this needs no norm
    conversion. This is the reference the driver's neutral output is judged
    against and the same reference the preview draws on its bars.
    """
    targets: list[float | None] = []
    for channel in config["channels"]:
        low, high = channel["minApplied"], channel["maxApplied"]
        span = high - low
        targets.append(None if span <= 0 else (channel["neutralApplied"] - low) / span)
    return targets


def build_rows(config: dict[str, Any]) -> list[Row]:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from mediapipe_driver import BLENDSHAPE_MAP, apply_blendshape_map

    # A neutral face means every mapped blendshape score is ~0; an empty dict
    # exercises exactly that, since the mapping reads scores with .get(name, 0).
    neutral_output = apply_blendshape_map({})
    targets = neutral_targets(config)

    rows = []
    for index, channel in enumerate(config["channels"]):
        channel_id = channel["id"]
        entries = BLENDSHAPE_MAP.get(channel_id, [])
        rows.append(
            Row(
                channel_id=channel_id,
                name=channel.get("name", f"ch{channel_id}"),
                enabled=bool(channel.get("enabled", True)),
                neutral=targets[index],
                actual=neutral_output[channel_id],
                push_up=sum(weight for _, weight, _ in entries if weight > 0),
                push_down=-sum(weight for _, weight, _ in entries if weight < 0),
            )
        )
    return rows


def report(rows: list[Row], tolerance: float) -> int:
    print(
        f"mapping table against each channel's calibrated neutral, "
        f"tolerance {tolerance:g}\n"
    )
    header = (
        f"{'ch':>3}  {'name':<24} {'kind':<6} {'neutral':>7} {'driver':>7} "
        f"{'delta':>7} {'room up':>8} {'push up':>8} {'room dn':>8} {'push dn':>8}"
    )
    print(header)
    print("-" * len(header))
    for row in rows:
        if not row.mapped:
            state = "disabled" if not row.enabled else "unmapped"
            print(f"{row.channel_id:>3}  {row.name:<24} {state}")
            continue
        neutral = row.neutral
        kind = "two-way" if row.bidirectional else "one-way"
        room_up = "  --  " if neutral is None else f"{1.0 - neutral:6.2f}"
        room_dn = "  --  " if neutral is None else f"{neutral:6.2f}"
        target = "  --  " if neutral is None else f"{neutral:6.3f}"
        delta = row.deviation
        print(
            f"{row.channel_id:>3}  {row.name:<24} {kind:<6} {target:>7} "
            f"{row.actual:7.3f} {'  --  ' if delta is None else f'{delta:+6.3f}':>7} "
            f"{room_up:>8} {row.push_up:8.2f} {room_dn:>8} {row.push_down:8.2f}"
        )

    offenders = [(row, row.problems(tolerance)) for row in rows]
    offenders = [(row, issues) for row, issues in offenders if issues]
    driven = sum(1 for row in rows if row.driven)
    print(f"\n{driven} driven channels, {len(offenders)} with problems")
    for row, issues in offenders:
        print(f"  ch {row.channel_id:>2} {row.name:<24} {'; '.join(issues)}")
    return 1 if offenders else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE)
    args = parser.parse_args(argv)

    config = json.loads(args.config.read_text(encoding="utf-8"))
    return report(build_rows(config), args.tolerance)


if __name__ == "__main__":
    sys.exit(main())
