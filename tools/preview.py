"""Live preview with framing guides, for placing the camera (board 2 step 3).

The handbook wants the face to fill 60-80% of the frame height. That is
impossible to hit blind, so this shows the camera feed with the target band
drawn on it and the measured face height reported live.

Parameters are locked first, same as every other tool here -- you want to aim
the camera under the exposure you will actually record with, not under
autoexposure that re-brightens whatever you point it at.

Overlay text is ASCII only: cv2.putText cannot render CJK.

Usage:
    python tools/preview.py [--config tools/camera_params.json] [--no-landmarks]
    keys: q or Esc quit, s save a snapshot next to the config
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from camera_capture import Camera, CameraError, CaptureConfig
from facemesh import DEFAULT_MODEL, LandmarkExtractor

TARGET_LOW, TARGET_HIGH = 0.60, 0.80
# Mean grey inside the face box. Wide on purpose -- this is a "not obviously
# wrong" band, not an optimum; the noise-floor measurement is what picks the
# actual best exposure for a given lighting setup.
EXPOSURE_LOW, EXPOSURE_HIGH = 90.0, 190.0
FONT = cv2.FONT_HERSHEY_SIMPLEX
GREY = (150, 150, 150)
GREEN = (110, 220, 140)
AMBER = (80, 190, 240)
RED = (90, 90, 235)


def draw_target_band(canvas: np.ndarray) -> None:
    """Rectangle whose height is the middle of the target range."""
    height, width = canvas.shape[:2]
    band_h = int(height * (TARGET_LOW + TARGET_HIGH) / 2)
    band_w = int(band_h * 0.72)  # rough face aspect
    x0 = (width - band_w) // 2
    y0 = (height - band_h) // 2
    cv2.rectangle(canvas, (x0, y0), (x0 + band_w, y0 + band_h), GREY, 1)
    cv2.putText(canvas, "target", (x0 + 4, y0 - 6), FONT, 0.4, GREY, 1, cv2.LINE_AA)
    cx, cy = width // 2, height // 2
    cv2.line(canvas, (cx - 8, cy), (cx + 8, cy), GREY, 1)
    cv2.line(canvas, (cx, cy - 8), (cx, cy + 8), GREY, 1)


def draw_face(
    canvas: np.ndarray, points: np.ndarray
) -> tuple[float, tuple[int, int, int, int]]:
    """Draws landmarks plus bounding box.

    Returns (face height as a fraction of frame height, the bounding box). The
    box is what exposure should actually be judged on: whole-frame brightness
    says nothing useful when the subject is lit and the background is not.
    """
    for x, y in points:
        cv2.circle(canvas, (round(float(x)), round(float(y))), 1, GREEN, -1)
    x0, y0 = points.min(axis=0)
    x1, y1 = points.max(axis=0)
    box = (round(float(x0)), round(float(y0)), round(float(x1)), round(float(y1)))
    cv2.rectangle(canvas, (box[0], box[1]), (box[2], box[3]), GREEN, 1)
    return float(y1 - y0) / canvas.shape[0], box


def meter(grey: np.ndarray) -> tuple[float, float, float]:
    """Mean brightness plus the share of pixels crushed to black or blown out."""
    return (
        float(grey.mean()),
        float((grey > 250).mean() * 100),
        float((grey < 5).mean() * 100),
    )


def draw_readout(
    canvas: np.ndarray, lines: list[tuple[str, tuple[int, int, int]]]
) -> None:
    height = 8 + 20 * len(lines)
    overlay = canvas[0:height, 0:260].copy()
    cv2.rectangle(overlay, (0, 0), (260, height), (0, 0, 0), -1)
    canvas[0:height, 0:260] = cv2.addWeighted(
        overlay, 0.55, canvas[0:height, 0:260], 0.45, 0
    )
    for i, (text, colour) in enumerate(lines):
        cv2.putText(canvas, text, (8, 18 + 20 * i), FONT, 0.45, colour, 1, cv2.LINE_AA)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=Path("tools/camera_params.json"))
    parser.add_argument("--device", default=None)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument(
        "--no-landmarks", action="store_true", help="skip face detection"
    )
    parser.add_argument(
        "--seconds", type=float, default=900.0, help="auto-quit safety timeout"
    )
    args = parser.parse_args(argv)

    try:
        config = CaptureConfig.load(args.config)
        if args.device is not None:
            config.device = args.device
        extractor = None if args.no_landmarks else LandmarkExtractor(args.model)
        with Camera(config) as camera:
            camera.lock_params()
            window = "BionicFace camera preview -- q to quit, s to snapshot"
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            deadline = time.monotonic() + args.seconds
            stamps: list[float] = []
            saved = 0
            while time.monotonic() < deadline:
                frame = camera.grab()
                canvas = frame.image.copy()
                stamps = ([*stamps, frame.timestamp])[-30:]
                fps = (
                    (len(stamps) - 1) / (stamps[-1] - stamps[0])
                    if len(stamps) > 1 and stamps[-1] > stamps[0]
                    else 0.0
                )
                draw_target_band(canvas)

                grey = cv2.cvtColor(frame.image, cv2.COLOR_BGR2GRAY)
                format_line = f"fps {fps:5.1f}   {canvas.shape[1]}x{canvas.shape[0]} {config.fourcc}"
                mean_all, hi_all, lo_all = meter(grey)
                exposure_line = (
                    f"frame  mean {mean_all:5.1f}  hi {hi_all:4.1f}%  lo {lo_all:4.1f}%"
                )
                lines = [(format_line, GREY), (exposure_line, GREY)]
                if extractor is not None:
                    points = extractor.detect(frame.image)
                    if points is None:
                        lines.append(("NO FACE", RED))
                    else:
                        share, (bx0, by0, bx1, by1) = draw_face(canvas, points)
                        ok = TARGET_LOW <= share <= TARGET_HIGH
                        face_line = (
                            f"face {share * 100:4.1f}%  target {TARGET_LOW * 100:.0f}"
                            f"-{TARGET_HIGH * 100:.0f}%  {'OK' if ok else 'ADJUST'}"
                        )
                        lines.append((face_line, GREEN if ok else AMBER))
                        crop = grey[max(by0, 0) : by1 + 1, max(bx0, 0) : bx1 + 1]
                        if crop.size:
                            mean_face, hi_face, lo_face = meter(crop)
                            exposed = EXPOSURE_LOW <= mean_face <= EXPOSURE_HIGH
                            verdict = (
                                "OK"
                                if exposed
                                else "DARK"
                                if mean_face < EXPOSURE_LOW
                                else "BRIGHT"
                            )
                            face_meter = (
                                f"face   mean {mean_face:5.1f}  hi {hi_face:4.1f}%"
                                f"  lo {lo_face:4.1f}%  {verdict}"
                            )
                            lines.append((face_meter, GREEN if exposed else AMBER))
                draw_readout(canvas, lines)
                cv2.imshow(window, canvas)

                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("s"):
                    saved += 1
                    path = args.config.parent / f"preview_snap_{saved:02d}.jpg"
                    cv2.imwrite(str(path), frame.image, [cv2.IMWRITE_JPEG_QUALITY, 92])
                    print(f"saved {path}")
            cv2.destroyAllWindows()
        if extractor is not None:
            extractor.close()
        return 0
    except (CameraError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
