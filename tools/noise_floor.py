"""Measures the landmark noise floor: how much MediaPipe jitters on a still target.

Board 2 step 2 of the capture-pipeline handbook. This is the foundation number
for the whole self-modelling effort: it says how much landmark movement is pure
measurement noise, so that later, when a servo channel moves and the landmarks
barely change, you know whether to believe the data or fix the lighting.

Method: point the fixed camera at a *still* target (the handbook asks for a
printed face on a wall, which is stricter than a live person holding still),
record N seconds, and take the per-landmark standard deviation across frames.
With a static scene and static camera, everything that moves is noise.

Per point, sigma = sqrt(std_x^2 + std_y^2), reported in pixels and in per-mille
of face width. The normalised figure is what makes two lighting setups
comparable when the camera sat at slightly different distances.

Run it once per lighting setup with a distinct --label; results append to the
report file so the setups can be compared side by side, and the lowest-noise one
becomes the standard lighting for the capture room.

Usage:
    python tools/noise_floor.py --label "ring-light-diffused" [--seconds 30]
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

from camera_capture import Camera, CameraError, CaptureConfig
from facemesh import DEFAULT_MODEL, LandmarkExtractor, mesh_indices, regions

MIN_FRAMES = 10


@dataclass(frozen=True)
class RegionNoise:
    """Noise statistics over one set of landmark indices."""

    name: str
    points: int
    mean_px: float
    p95_px: float
    max_px: float
    mean_permille: float


@dataclass(frozen=True)
class NoiseFloor:
    """The outcome of one recording session."""

    label: str
    frames_used: int
    frames_total: int
    seconds: float
    face_width_px: float
    sigma_px: np.ndarray  # (N,) per-landmark sigma
    mean_positions: np.ndarray  # (N, 2) mean pixel position
    regions: list[RegionNoise]

    @property
    def detection_rate(self) -> float:
        return self.frames_used / self.frames_total if self.frames_total else 0.0


def _region_noise(
    name: str, indices: np.ndarray, sigma_px: np.ndarray, face_width_px: float
) -> RegionNoise:
    values = sigma_px[indices]
    return RegionNoise(
        name=name,
        points=len(indices),
        mean_px=float(values.mean()),
        p95_px=float(np.percentile(values, 95)),
        max_px=float(values.max()),
        mean_permille=float(values.mean() / face_width_px * 1000.0),
    )


def record(
    camera: Camera, extractor: LandmarkExtractor, seconds: float, label: str
) -> NoiseFloor:
    """Records for `seconds` and reduces the run to per-landmark sigma."""
    tracks: list[np.ndarray] = []
    total = 0
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        frame = camera.grab()
        total += 1
        landmarks = extractor.detect(frame.image)
        if landmarks is not None:
            tracks.append(landmarks)

    if len(tracks) < MIN_FRAMES:
        raise CameraError(
            f"only {len(tracks)} of {total} frames produced landmarks (need "
            f"{MIN_FRAMES}); the target is not being detected -- check framing, "
            f"focus and lighting"
        )
    if len({track.shape for track in tracks}) != 1:
        raise CameraError("landmark count changed mid-run; re-run with num_faces=1")

    stack = np.stack(tracks)  # (F, N, 2)
    std_xy = stack.std(axis=0, ddof=1)  # (N, 2)
    sigma_px = np.hypot(std_xy[:, 0], std_xy[:, 1])
    mean_positions = stack.mean(axis=0)

    oval = np.fromiter(sorted(regions()["face_oval"]), dtype=int)
    face_width_px = float(mean_positions[oval, 0].max() - mean_positions[oval, 0].min())
    if face_width_px <= 0:
        raise CameraError("degenerate face width; landmarks are collapsed")

    mesh = np.fromiter(sorted(mesh_indices()), dtype=int)
    breakdown = [_region_noise("mesh (468)", mesh, sigma_px, face_width_px)]
    for name, index_set in regions().items():
        indices = np.fromiter(sorted(index_set), dtype=int)
        indices = indices[indices < len(sigma_px)]
        if len(indices):
            breakdown.append(_region_noise(name, indices, sigma_px, face_width_px))

    return NoiseFloor(
        label=label,
        frames_used=len(tracks),
        frames_total=total,
        seconds=seconds,
        face_width_px=face_width_px,
        sigma_px=sigma_px,
        mean_positions=mean_positions,
        regions=breakdown,
    )


def draw_heatmap(result: NoiseFloor, size: tuple[int, int], path: Path) -> None:
    """Scatters the mean landmark positions, coloured by that point's sigma."""
    width, height = size
    canvas = np.full((height, width, 3), 24, dtype=np.uint8)
    ceiling = float(np.percentile(result.sigma_px, 95)) or 1e-6
    lut = cv2.applyColorMap(
        np.arange(256, dtype=np.uint8).reshape(256, 1), cv2.COLORMAP_TURBO
    )

    for (x, y), sigma in zip(result.mean_positions, result.sigma_px, strict=True):
        level = int(np.clip(sigma / ceiling, 0.0, 1.0) * 255)
        colour = tuple(int(channel) for channel in lut[level, 0])
        cv2.circle(
            canvas, (round(float(x)), round(float(y))), 2, colour, -1, cv2.LINE_AA
        )

    bar_x, bar_y, bar_w, bar_h = 12, height - 28, min(240, width - 24), 10
    for offset in range(bar_w):
        colour = tuple(int(c) for c in lut[int(offset / bar_w * 255), 0])
        cv2.line(
            canvas, (bar_x + offset, bar_y), (bar_x + offset, bar_y + bar_h), colour, 1
        )
    _text(canvas, "0", (bar_x, bar_y - 4))
    _text(canvas, f"{ceiling:.2f}px (p95)", (bar_x + bar_w - 78, bar_y - 4))
    _text(canvas, f"{result.label}", (12, 18))
    _text(
        canvas,
        f"sigma mean {result.regions[0].mean_px:.3f}px  "
        f"({result.regions[0].mean_permille:.2f} permille face width)",
        (12, 34),
    )
    _text(canvas, f"{result.frames_used} frames / {result.seconds:g}s", (12, 50))

    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), canvas):
        raise CameraError(f"could not write heatmap to {path}")


def _text(canvas: np.ndarray, text: str, origin: tuple[int, int]) -> None:
    cv2.putText(
        canvas,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        (235, 235, 235),
        1,
        cv2.LINE_AA,
    )


def render_report(
    result: NoiseFloor,
    config: CaptureConfig,
    negotiated: dict[str, object],
    locked: dict[str, int],
    heatmap: Path,
    stamped: str,
) -> str:
    fingerprint = ", ".join(f"{name}={value}" for name, value in sorted(locked.items()))
    lines = [
        f"## {result.label} — {stamped}",
        "",
        f"- 设备:`{config.device}`",
        (
            f"- 格式:{negotiated['width']}×{negotiated['height']} "
            f"{negotiated['fourcc']} @ {negotiated['fps']}fps"
        ),
        (
            f"- 采样:{result.seconds:g}s,有效帧 "
            f"{result.frames_used}/{result.frames_total} "
            f"({result.detection_rate * 100:.1f}%)"
        ),
        f"- 脸宽基准:{result.face_width_px:.1f}px",
        f"- 参数指纹:`{fingerprint}`",
        f"- 热力图:`{heatmap.name}`",
        "",
        "| 区域 | 点数 | σ 均值(px) | σ p95(px) | σ 最大(px) | σ 均值(‰脸宽) |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for region in result.regions:
        lines.append(
            f"| {region.name} | {region.points} | {region.mean_px:.3f} | "
            f"{region.p95_px:.3f} | {region.max_px:.3f} | {region.mean_permille:.2f} |"
        )
    lines.append("")
    return "\n".join(lines)


def append_report(path: Path, section: str) -> None:
    header = "" if path.exists() else "# 关键点噪声底测量记录\n\n"
    with path.open("a", encoding="utf-8") as handle:
        handle.write(header + section + "\n")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--label", required=True, help="lighting setup being measured")
    parser.add_argument("--config", type=Path, default=Path("tools/camera_params.json"))
    parser.add_argument("--device", default=None, help="overrides the config's device")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--report", type=Path, default=Path("noise_floor_report.md"))
    parser.add_argument(
        "--heatmap",
        type=Path,
        default=None,
        help="defaults to noise_floor_<label>.png beside the report",
    )
    args = parser.parse_args(argv)

    slug = "".join(c if c.isalnum() or c in "-_" else "-" for c in args.label)
    heatmap = args.heatmap or args.report.parent / f"noise_floor_{slug}.png"

    try:
        config = CaptureConfig.load(args.config)
        if args.device is not None:
            config.device = args.device
        with Camera(config) as camera:
            locked = camera.lock_params()
            negotiated = camera.negotiated_format()
            print(f"{config.device}: {negotiated}, {len(locked)} controls locked")
            print(
                f"recording {args.seconds:g}s on a still target -- do not move anything\n"
            )
            with LandmarkExtractor(args.model) as extractor:
                result = record(camera, extractor, args.seconds, args.label)
            size = (int(negotiated["width"]), int(negotiated["height"]))
        draw_heatmap(result, size, heatmap)
        stamped = (
            datetime.now(timezone.utc).astimezone().strftime("%Y-%m-%d %H:%M:%S %z")
        )
        section = render_report(result, config, negotiated, locked, heatmap, stamped)
        append_report(args.report, section)
        print(section)
        print(f"appended to {args.report}; heatmap written to {heatmap}")
        return 0
    except (CameraError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
