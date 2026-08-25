"""Compares the driver's neutral output against the rest preset, channel by channel.

At a neutral face every mapped blendshape score is ~0, so what the driver emits
is exactly the sum of the bias terms in its mapping table. That number should
match where the rest preset puts each channel -- otherwise the twin face sits in
a subtly wrong pose the moment the driver takes over, and every expression is
measured from the wrong baseline.

Target side: the rest preset is stored in the bipolar, neutral-anchored norm
space; the external input port consumes flat unipolar [0, 1] over the full
applied range. So the target is norm -> applied -> unipolar.

The norm -> applied step is deliberately *imported* from
raspi/migrate_presets_to_normalized.py rather than rewritten here. That module
already carries a Python mirror of control.rs's MotorChannel::norm_to_applied,
with the rationale documented; a third copy of the same arithmetic is exactly
how the two drift apart.

Usage:
    python tools/check_neutral.py [--config src-tauri/config/motor_config.json]
                                  [--preset rest] [--tolerance 0.05]

Exits 1 if any driven channel is off by more than the tolerance.
"""

from __future__ import annotations

import argparse
import importlib.util
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = REPO_ROOT / "src-tauri" / "config" / "motor_config.json"
NORM_MODULE = REPO_ROOT / "raspi" / "migrate_presets_to_normalized.py"
DEFAULT_TOLERANCE = 0.05


def _load_norm_to_applied() -> Any:
    """Imports the existing Python mirror of control.rs's norm conversion.

    Loaded by path because raspi/ is a script directory, not a package, and
    importing it must not run its migration main().
    """
    spec = importlib.util.spec_from_file_location("_norm_mirror", NORM_MODULE)
    if spec is None or spec.loader is None:
        raise SystemExit(f"cannot load the norm conversion from {NORM_MODULE}")
    module = importlib.util.module_from_spec(spec)
    # That module imports a sibling by bare name, so raspi/ has to be importable.
    # Both it and the sibling guard their entry points behind __main__, so this
    # only defines functions.
    sys.path.insert(0, str(NORM_MODULE.parent))
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path.remove(str(NORM_MODULE.parent))
    return module.norm_to_applied


@dataclass(frozen=True)
class Row:
    """One channel's comparison."""

    channel_id: int
    name: str
    enabled: bool
    rest_norm: float
    target: float | None  # unipolar coefficient the rest preset implies
    actual: float | None  # what the driver emits at a neutral face
    span: float

    @property
    def driven(self) -> bool:
        return self.enabled and self.actual is not None

    @property
    def deviation(self) -> float | None:
        if self.target is None or self.actual is None:
            return None
        return self.actual - self.target

    def status(self, tolerance: float) -> str:
        if not self.enabled:
            return "disabled"
        if self.actual is None:
            return "not mapped"
        if self.target is None:
            return "no target"
        return "ok" if abs(self.deviation or 0.0) <= tolerance else "OFF"


def _preset_norm(config: dict[str, Any], preset_id: str) -> list[float]:
    presets = {
        preset.get("id"): preset for preset in config.get("expressionPresets", [])
    }
    if preset_id not in presets:
        raise SystemExit(
            f"no preset {preset_id!r} in the config; found: {', '.join(sorted(presets))}"
        )
    norm = presets[preset_id]["norm"]
    channels = config["channels"]
    if len(norm) != len(channels):
        raise SystemExit(
            f"preset {preset_id!r} has {len(norm)} entries but the config has "
            f"{len(channels)} channels"
        )
    return [float(value) for value in norm]


def neutral_targets(
    config: dict[str, Any], preset_id: str = "rest"
) -> list[float | None]:
    """The unipolar coefficient each channel should sit at for `preset_id`.

    This is the reference the driver's neutral output is judged against, and the
    same reference the preview draws on its bars, so both read it from here
    instead of each deriving it.
    """
    norm_to_applied = _load_norm_to_applied()
    preset_norm = _preset_norm(config, preset_id)
    targets: list[float | None] = []
    for channel in config["channels"]:
        low, high = channel["minApplied"], channel["maxApplied"]
        span = high - low
        if span <= 0:
            targets.append(None)
            continue
        applied = norm_to_applied(
            preset_norm[channel["id"]], low, high, channel["neutralApplied"]
        )
        targets.append((applied - low) / span)
    return targets


def build_rows(
    config: dict[str, Any], preset_id: str, neutral_output: list[float | None]
) -> list[Row]:
    preset_norm = _preset_norm(config, preset_id)
    targets = neutral_targets(config, preset_id)
    rows = []
    for index, channel in enumerate(config["channels"]):
        channel_id = channel["id"]
        rows.append(
            Row(
                channel_id=channel_id,
                name=channel.get("name", f"ch{channel_id}"),
                enabled=bool(channel.get("enabled", True)),
                rest_norm=preset_norm[channel_id],
                target=targets[index],
                actual=neutral_output[channel_id],
                span=channel["maxApplied"] - channel["minApplied"],
            )
        )
    return rows


def report(rows: list[Row], tolerance: float, preset_id: str) -> int:
    print(
        f"neutral alignment against preset {preset_id!r}, tolerance {tolerance:g}\n"
        f"target = what the preset implies, driver = what the mapping table emits "
        f"at a neutral face\n"
    )
    header = f"{'ch':>3}  {'name':<22} {'norm':>6} {'target':>7} {'driver':>7} {'delta':>7}  status"
    print(header)
    print("-" * len(header))
    for row in rows:
        target = "  --  " if row.target is None else f"{row.target:6.3f}"
        actual = "  --  " if row.actual is None else f"{row.actual:6.3f}"
        delta = row.deviation
        delta_text = "  --  " if delta is None else f"{delta:+6.3f}"
        print(
            f"{row.channel_id:>3}  {row.name:<22} {row.rest_norm:>6.2f} {target:>7} "
            f"{actual:>7} {delta_text:>7}  {row.status(tolerance)}"
        )

    offenders = [
        row
        for row in rows
        if row.driven and row.deviation is not None and abs(row.deviation) > tolerance
    ]
    driven = [row for row in rows if row.driven]
    print(f"\n{len(driven)} driven channels, {len(offenders)} beyond tolerance")

    if offenders:
        print(
            "\nThe driver's neutral output is the sum of its bias terms, so each fix is a\n"
            "bias adjustment in the mapping table. Required change per channel:\n"
        )
        for row in offenders:
            delta = row.deviation or 0.0
            print(
                f"  ch {row.channel_id:>2} {row.name:<22} bias {-delta:+.3f}  "
                f"({row.actual:.3f} -> {row.target:.3f})"
            )
    return 1 if offenders else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--preset", default="rest", help="preset id to align against")
    parser.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE)
    args = parser.parse_args(argv)

    import json

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from mediapipe_driver import apply_blendshape_map

    config = json.loads(args.config.read_text(encoding="utf-8"))
    # A neutral face means every mapped blendshape score is ~0; passing an empty
    # dict exercises exactly that, since the mapping reads scores with .get(name, 0).
    neutral_output = apply_blendshape_map({})
    rows = build_rows(config, args.preset, neutral_output)
    return report(rows, args.tolerance, args.preset)


if __name__ == "__main__":
    sys.exit(main())
