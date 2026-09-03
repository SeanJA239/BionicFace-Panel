"""Flips through a collected session: image, landmarks, and the command behind it.

Left is the captured frame with its landmarks drawn on; right is the coefficient
that produced it, one bar per sampled channel. Arrow keys move between samples,
`b` marks the current one as bad.

The point is human spot-checking before training on anything: a sample can be
wrong in ways the collector cannot see -- the subject moved, the face was
half out of frame, landmarks latched onto something else -- and those need an
eye, not a threshold.

Overlay text is ASCII only: cv2.putText cannot render CJK.

Usage:
    python tools/dataset_browser.py dataset/20260826T141126Z
    keys: left/right or a/d = prev/next, b = mark bad, q = quit
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

BAD_FILE = "bad_samples.txt"
FONT = cv2.FONT_HERSHEY_SIMPLEX
PANEL_W = 320
GREEN = (110, 220, 140)
AMBER = (80, 190, 240)
RED = (90, 90, 235)
GREY = (150, 150, 150)


@dataclass(frozen=True)
class Sample:
    """One row of samples.jsonl plus where its image lives."""

    index: int
    raw: dict[str, Any]
    image_path: Path

    @property
    def sample_id(self) -> int:
        return int(self.raw["id"])

    @property
    def landmarks(self) -> np.ndarray | None:
        points = self.raw.get("landmarks")
        return None if points is None else np.asarray(points, dtype=np.float32)

    @property
    def driven(self) -> list[tuple[int, float]]:
        return [
            (channel_id, value)
            for channel_id, value in enumerate(self.raw["target_coefficients"])
            if value is not None
        ]


def load_session(session_dir: Path) -> tuple[list[Sample], dict[str, Any]]:
    meta_path = session_dir / "samples.jsonl"
    if not meta_path.exists():
        raise SystemExit(f"no samples.jsonl in {session_dir}")
    manifest_path = session_dir / "manifest.json"
    manifest = (
        json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest_path.exists()
        else {}
    )
    samples = []
    for index, line in enumerate(meta_path.read_text(encoding="utf-8").splitlines()):
        line = line.strip()
        if not line:
            continue
        raw = json.loads(line)
        samples.append(Sample(index, raw, session_dir / raw["image"]))
    if not samples:
        raise SystemExit(f"{meta_path} is empty")
    return samples, manifest


def read_bad(session_dir: Path) -> set[int]:
    path = session_dir / BAD_FILE
    if not path.exists():
        return set()
    return {
        int(line.split()[0])
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip() and line.split()[0].isdigit()
    }


def write_bad(session_dir: Path, bad: set[int]) -> None:
    path = session_dir / BAD_FILE
    lines = [f"{sample_id}\n" for sample_id in sorted(bad)]
    path.write_text("".join(lines), encoding="utf-8")


def render(
    sample: Sample, manifest: dict[str, Any], total: int, is_bad: bool
) -> np.ndarray:
    image = cv2.imread(str(sample.image_path))
    if image is None:
        image = np.full((480, 640, 3), 30, dtype=np.uint8)
        cv2.putText(image, "image missing", (20, 240), FONT, 0.8, RED, 2, cv2.LINE_AA)
    height = image.shape[0]

    landmarks = sample.landmarks
    if landmarks is not None:
        for x, y in landmarks:
            cv2.circle(image, (round(float(x)), round(float(y))), 1, GREEN, -1)
    else:
        cv2.putText(
            image, "NO LANDMARKS", (16, height - 16), FONT, 0.7, RED, 2, cv2.LINE_AA
        )

    panel = np.full((height, PANEL_W, 3), 24, dtype=np.uint8)
    driven = sample.driven
    bar_w = PANEL_W - 96
    row_h = max(10, min(16, (height - 96) // max(len(driven), 1)))
    for row, (channel_id, value) in enumerate(driven):
        y = 78 + row * row_h
        if y + row_h > height - 8:
            break
        cv2.rectangle(panel, (56, y), (56 + bar_w, y + row_h - 4), (60, 60, 60), -1)
        cv2.rectangle(
            panel, (56, y), (56 + int(bar_w * value), y + row_h - 4), GREEN, -1
        )
        cv2.putText(
            panel,
            f"{channel_id:02d}",
            (8, y + row_h - 6),
            FONT,
            0.35,
            GREY,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            panel,
            f"{value:.2f}",
            (60 + bar_w, y + row_h - 6),
            FONT,
            0.32,
            GREY,
            1,
            cv2.LINE_AA,
        )

    header = [
        (f"sample {sample.sample_id}  ({sample.index + 1}/{total})", 0.5, GREY),
        (
            f"subject: {manifest.get('subject', sample.raw.get('subject', '?'))}",
            0.42,
            GREY,
        ),
        (
            "settled" if sample.raw.get("settled") else "NOT SETTLED",
            0.42,
            GREY if sample.raw.get("settled") else AMBER,
        ),
        ("MARKED BAD" if is_bad else "ok", 0.46, RED if is_bad else GREEN),
    ]
    y = 20
    for text, scale, colour in header:
        cv2.putText(panel, text, (8, y), FONT, scale, colour, 1, cv2.LINE_AA)
        y += 16
    return np.hstack([image, panel])


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("session", type=Path, help="a dataset session directory")
    args = parser.parse_args(argv)

    samples, manifest = load_session(args.session)
    bad = read_bad(args.session)
    window = "dataset browser -- arrows/ad move, b marks bad, q quits"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cursor = 0
    print(f"{len(samples)} samples, {len(bad)} already marked bad")
    while True:
        sample = samples[cursor]
        cv2.imshow(
            window, render(sample, manifest, len(samples), sample.sample_id in bad)
        )
        key = cv2.waitKey(30) & 0xFF
        if key in (ord("q"), 27):
            break
        if key in (ord("d"), 83):  # right arrow
            cursor = min(cursor + 1, len(samples) - 1)
        elif key in (ord("a"), 81):  # left arrow
            cursor = max(cursor - 1, 0)
        elif key == ord("b"):
            if sample.sample_id in bad:
                bad.discard(sample.sample_id)
            else:
                bad.add(sample.sample_id)
            write_bad(args.session, bad)
            print(
                f"sample {sample.sample_id}: {'bad' if sample.sample_id in bad else 'ok'}"
            )
    cv2.destroyAllWindows()
    print(f"{len(bad)} marked bad -> {args.session / BAD_FILE}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
