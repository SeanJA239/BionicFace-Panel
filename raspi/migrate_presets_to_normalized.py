"""One-time migration: convert emotion.legacy.md's raw applied-degree
expression presets into presets.json, storing each channel as a bipolar
normalized coefficient in [-1, 1] instead of a raw physical angle.

Why bipolar norm instead of the unipolar [0, 1] `(applied - min) / (max -
min)` coefficient originally sketched for this migration: control.rs
already implements and depends on a bipolar, neutral-anchored norm space
(`MotorChannel::norm_to_applied` / `applied_to_norm`, see the "phase 0-4"
commits) for sliders, jaw coupling, and the nod action. That mapping is
piecewise about each channel's calibrated neutral, so a preset baked at one
calibration still lands at the same relative pose after recalibration, and
it is asymmetry-aware (a channel whose neutral sits off-center from its
limits keeps exact +1/-1 endpoints on both sides). Re-deriving this
migration around a second, incompatible min/max coefficient scheme would
fork the codebase's normalization semantics for no benefit, so this script
reuses the existing scheme instead. See README.md's "表情预设系统" section.

Run once, after which emotion.legacy.md is reference-only and no longer
feeds raspi/export_config_json.py.
"""

from __future__ import annotations

import json
import re
from pathlib import Path

import export_config_json as expcfg

ROOT = Path(__file__).resolve().parents[1]
LEGACY_EMOTION_PATH = ROOT / "emotion.legacy.md"
OUTPUT_PATH = ROOT / "presets.json"
NORM_EPSILON = 1e-6


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def applied_to_norm(applied: float, min_applied: float, max_applied: float, neutral_applied: float) -> float:
    """Mirrors MotorChannel::applied_to_norm in control.rs exactly."""
    applied = clamp(applied, min_applied, max_applied)
    if applied >= neutral_applied:
        span = max_applied - neutral_applied
        norm = 0.0 if span <= NORM_EPSILON else (applied - neutral_applied) / span
    else:
        span = neutral_applied - min_applied
        norm = 0.0 if span <= NORM_EPSILON else (applied - neutral_applied) / span
    return clamp(norm, -1.0, 1.0)


def norm_to_applied(norm: float, min_applied: float, max_applied: float, neutral_applied: float) -> float:
    """Mirrors MotorChannel::norm_to_applied in control.rs exactly."""
    norm = clamp(norm, -1.0, 1.0)
    if norm >= 0.0:
        applied = neutral_applied + norm * (max_applied - neutral_applied)
    else:
        applied = neutral_applied + norm * (neutral_applied - min_applied)
    return clamp(applied, min_applied, max_applied)


def parse_legacy_presets(path: Path) -> list[dict]:
    raw = path.read_text(encoding="utf-8")
    matches = list(re.finditer(r"^\s*(.+?)\s*[:：]\s*$", raw, re.MULTILINE))
    presets: list[dict] = []
    for index, match in enumerate(matches):
        label = match.group(1).strip()
        start = match.end()
        end = matches[index + 1].start() if index + 1 < len(matches) else len(raw)
        block = raw[start:end]
        numbers = [float(value) for value in re.findall(r"-?\d+(?:\.\d+)?", block)]
        if len(numbers) != 32:
            raise RuntimeError(
                f"Legacy preset '{label}' must contain exactly 32 angles, got {len(numbers)}"
            )
        presets.append({"id": label, "label": label, "angles": numbers})
    return presets


def main() -> None:
    module = expcfg.load_module(expcfg.CONFIG_PATH)
    channels = [expcfg.build_channel(module, motor_id) for motor_id in range(32)]
    names = getattr(module, "MOTOR_NAMES", {})

    legacy_presets = parse_legacy_presets(LEGACY_EMOTION_PATH)

    out_presets = []
    clamp_warnings: list[tuple[str, str, float, float]] = []
    reconstruction_errors: list[tuple[str, str, float, float]] = []

    for preset in legacy_presets:
        norm_values = []
        for motor_id, angle in enumerate(preset["angles"]):
            channel = channels[motor_id]
            name = names.get(motor_id, f"motor_{motor_id:02d}")

            if not channel["enabled"]:
                # Disabled channels are ignored by apply_motor_target_norm's
                # !enabled guard regardless of stored value; 0.0 (this
                # channel's own norm-space neutral) documents "no signal"
                # more literally than the 0.5 written by the doc's original
                # unipolar scheme.
                norm_values.append(0.0)
                continue

            min_applied = channel["minApplied"]
            max_applied = channel["maxApplied"]
            neutral_applied = channel["neutralApplied"]

            clamped = clamp(angle, min_applied, max_applied)
            if clamped != angle:
                clamp_warnings.append((preset["label"], name, angle, clamped))

            norm = applied_to_norm(clamped, min_applied, max_applied, neutral_applied)
            norm_values.append(norm)

            reconstructed = norm_to_applied(norm, min_applied, max_applied, neutral_applied)
            if clamped == angle and abs(reconstructed - angle) >= 0.5:
                reconstruction_errors.append((preset["label"], name, angle, reconstructed))

        out_presets.append({"id": preset["id"], "label": preset["label"], "norm": norm_values})

    if clamp_warnings:
        print(f"WARNING: {len(clamp_warnings)} channel(s) were out of MOTOR_LIMITS range and got clamped:")
        for label, name, orig, clamped in clamp_warnings:
            print(f"  preset={label!r} channel={name!r} orig={orig} clamped={clamped}")

    if reconstruction_errors:
        print(f"WARNING: {len(reconstruction_errors)} in-range channel(s) failed the <0.5deg round-trip check:")
        for label, name, orig, reconstructed in reconstruction_errors:
            print(f"  preset={label!r} channel={name!r} orig={orig} reconstructed={reconstructed}")
    else:
        print("Round-trip check passed: every in-range channel reconstructs within 0.5deg.")

    OUTPUT_PATH.write_text(
        json.dumps({"presets": out_presets}, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(f"Wrote {OUTPUT_PATH} ({len(out_presets)} presets)")


if __name__ == "__main__":
    main()
