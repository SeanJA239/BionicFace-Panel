"""Chessboard intrinsic calibration under the locked imaging parameters.

Board 2 step 3 wants to know whether the 2.1mm wide-angle lens displaces
landmarks enough to matter. That needs the camera matrix and distortion
coefficients for THIS unit at THE format we actually record with.

Two things this does that the vendor's correct.py does not:

  * Locks the imaging parameters first. correct.py opens a bare VideoCapture,
    so it calibrates under autoexposure that re-brightens every view -- corner
    detection quality then varies frame to frame, and the result belongs to no
    reproducible imaging setup.

  * Guides the view distribution and refuses to calibrate on a bad one. Lens
    distortion lives at the frame edges, so a stack of centred fronto-parallel
    views leaves it barely observable and the solver fits noise into the
    high-order terms. The vendor's shipped camera_params.yaml shows exactly
    that signature -- k2 = -0.69 fighting k3 = +0.78 -- and their script will
    calibrate on as few as five views.

Intrinsics are per-format: cropping and scaling differ between modes, so a
result taken at 640x480 does not transfer to 1280x720. The output records the
format and the full locked control set alongside the numbers.

If the reported RMS stays poor with good coverage, the next thing to try is
OpenCV's fisheye model -- the polynomial model can struggle past ~100 deg.

Usage:
    python tools/calibrate_intrinsics.py [--config tools/camera_params.json]
    keys: space capture a view, c calibrate, u toggle undistorted, q quit
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from camera_capture import Camera, CameraError, CaptureConfig

# The board shipped with the C100 kit: docs/.../标定板A4_9x6_25mm.pdf. The sheet
# is 7x10 squares of 25mm -- 6x9 inner corners, and the pattern runs right to the
# paper edge with no white margin. Neither matters here: findChessboardCorners
# was checked against a render of that exact sheet and detects it with or without
# a border, at either orientation, and rotated. Searching for the transpose just
# rotates the board's own coordinate frame, which each view's pose absorbs.
BOARD_COLS, BOARD_ROWS = 9, 6
SQUARE_MM = 25.0

MIN_VIEWS = 12
GOOD_VIEWS = 20
# Frame split into this many cells per axis for coverage accounting.
GRID = 3
# A view counts as tilted when opposite edges differ in length by at least this
# fraction -- i.e. there is real perspective foreshortening to constrain the
# solve, not another fronto-parallel shot.
TILT_RATIO = 0.08
MIN_TILTED_FRACTION = 0.3

# The board's inner-corner span, as a fraction of frame width, that leaves room
# to move it into every frame region. Filling the frame is the wrong target: it
# forces the board closer than this fixed-focus lens can resolve, and it makes
# the edge placements impossible.
# Lower bound is set by corner localisation, not by filling the frame: at 0.20
# the 8-square inner span is ~128px, so squares are ~16px and cornerSubPix still
# has plenty to work with. This lens is fixed-focus and focuses far, so the
# board often has to sit far enough back that a larger extent is not available --
# sharpness wins over size, and a smaller board just means more placements.
EXTENT_LOW, EXTENT_HIGH = 0.20, 0.55
# Minimap geometry. Coverage used to be drawn as a 3x3 grid over the whole
# frame, which reads as detected chessboard squares when a chessboard is what
# you are pointing at. A corner minimap cannot be mistaken for detection.
MINIMAP_W, MINIMAP_H, MINIMAP_MARGIN = 96, 72, 10
# How long a rejected-keypress explanation stays on screen.
MESSAGE_SECONDS = 2.5

FONT = cv2.FONT_HERSHEY_SIMPLEX
GREEN = (110, 220, 140)
AMBER = (80, 190, 240)
RED = (90, 90, 235)
GREY = (150, 150, 150)

SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
FIND_FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE


def object_points() -> np.ndarray:
    """One board's corner positions in board-local millimetres, z = 0."""
    grid = np.zeros((BOARD_ROWS * BOARD_COLS, 3), np.float32)
    grid[:, :2] = np.mgrid[0:BOARD_COLS, 0:BOARD_ROWS].T.reshape(-1, 2) * SQUARE_MM
    return grid


@dataclass
class View:
    """One accepted board sighting."""

    corners: np.ndarray
    image: np.ndarray
    cells: frozenset[int]
    tilted: bool


def board_edges(corners: np.ndarray) -> tuple[float, float, float, float]:
    """Lengths of the board's four outer edges, in pixels."""
    pts = corners.reshape(BOARD_ROWS, BOARD_COLS, 2)
    tl, tr = pts[0, 0], pts[0, -1]
    bl, br = pts[-1, 0], pts[-1, -1]
    return (
        float(np.linalg.norm(tr - tl)),
        float(np.linalg.norm(br - bl)),
        float(np.linalg.norm(bl - tl)),
        float(np.linalg.norm(br - tr)),
    )


def is_tilted(corners: np.ndarray) -> bool:
    top, bottom, left, right = board_edges(corners)
    horizontal = abs(top - bottom) / max(top, bottom, 1e-6)
    vertical = abs(left - right) / max(left, right, 1e-6)
    return max(horizontal, vertical) >= TILT_RATIO


def cells_covered(corners: np.ndarray, size: tuple[int, int]) -> frozenset[int]:
    """Every GRID x GRID cell holding at least one detected corner.

    Coverage is about where the observations land, not where the board's centre
    is. Distortion is largest at the periphery, so what the solve needs is
    corners out there -- and a board spanning 40% of the width already reaches
    several cells at once, which a centroid test throws away.
    """
    width, height = size
    pts = corners.reshape(-1, 2)
    cols = np.clip((pts[:, 0] / width * GRID).astype(int), 0, GRID - 1)
    rows = np.clip((pts[:, 1] / height * GRID).astype(int), 0, GRID - 1)
    return frozenset((rows * GRID + cols).tolist())


def board_extent(corners: np.ndarray, width: int) -> float:
    """Board's inner-corner span as a fraction of frame width."""
    xs = corners.reshape(-1, 2)[:, 0]
    return float(xs.max() - xs.min()) / width


def sharpness(grey: np.ndarray, corners: np.ndarray | None) -> float:
    """Laplacian variance over the board, or the frame centre if not found.

    This lens is fixed-focus -- the real-hardware sweep found focus_absolute has
    no effect -- so the only way to get a sharp board is to move it, and the only
    way to know when it is sharp is to measure. Absolute values are
    scene-dependent, so the display compares against the best seen this session
    rather than against a threshold.
    """
    if corners is not None:
        pts = corners.reshape(-1, 2)
        x0, y0 = pts.min(axis=0)
        x1, y1 = pts.max(axis=0)
        roi = grey[max(0, int(y0)) : int(y1) + 1, max(0, int(x0)) : int(x1) + 1]
    else:
        height, width = grey.shape
        roi = grey[height // 4 : 3 * height // 4, width // 4 : 3 * width // 4]
    if roi.size == 0:
        return 0.0
    return float(cv2.Laplacian(roi, cv2.CV_64F).var())


def coverage_gaps(views: list[View]) -> list[str]:
    """Human-readable reasons the captured set is not good enough yet."""
    gaps: list[str] = []
    if len(views) < MIN_VIEWS:
        gaps.append(f"need {MIN_VIEWS} views, have {len(views)}")
    filled = {cell for v in views for cell in v.cells}
    if len(filled) < GRID * GRID:
        missing = sorted(set(range(GRID * GRID)) - filled)
        gaps.append(f"{len(missing)} of {GRID * GRID} frame cells never covered")
    tilted = sum(1 for v in views if v.tilted)
    if views and tilted / len(views) < MIN_TILTED_FRACTION:
        gaps.append(f"only {tilted}/{len(views)} views tilted, want a third")
    return gaps


def fov_degrees(matrix: np.ndarray, size: tuple[int, int]) -> dict[str, float]:
    width, height = size
    fx, fy = float(matrix[0, 0]), float(matrix[1, 1])
    half_x, half_y = width / (2 * fx), height / (2 * fy)
    return {
        "horizontal": math.degrees(2 * math.atan(half_x)),
        "vertical": math.degrees(2 * math.atan(half_y)),
        "diagonal": math.degrees(2 * math.atan(math.hypot(half_x, half_y))),
    }


def per_view_errors(
    objects: list[np.ndarray],
    images: list[np.ndarray],
    rvecs: list[np.ndarray],
    tvecs: list[np.ndarray],
    matrix: np.ndarray,
    dist: np.ndarray,
) -> list[float]:
    errors = []
    for obj, img, rvec, tvec in zip(objects, images, rvecs, tvecs, strict=True):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, matrix, dist)
        # Deliberately not cv2.norm: it demands identical type AND channel count,
        # and cornerSubPix output vs projectPoints output do not reliably agree on
        # either -- that crashed a finished 21-view solve. Same RMS convention
        # calibrateCamera reports.
        delta = img.reshape(-1, 2).astype(float) - projected.reshape(-1, 2).astype(
            float
        )
        errors.append(float(np.sqrt((delta**2).sum() / len(delta))))
    return errors


def draw_minimap(canvas: np.ndarray, filled: set[int], current: frozenset[int]) -> None:
    """Green = captured, amber outline = where the board is pointing right now.

    The live cell matters: without it the map only ever changes on a keypress,
    so moving the board looks like it does nothing.
    """
    width = canvas.shape[1]
    x0 = width - MINIMAP_W - MINIMAP_MARGIN
    y0 = MINIMAP_MARGIN
    cell_w, cell_h = MINIMAP_W // GRID, MINIMAP_H // GRID
    for index in range(GRID * GRID):
        row, col = divmod(index, GRID)
        top_left = (x0 + col * cell_w, y0 + row * cell_h)
        bottom_right = (top_left[0] + cell_w - 1, top_left[1] + cell_h - 1)
        if index in filled:
            cv2.rectangle(canvas, top_left, bottom_right, GREEN, -1)
        cv2.rectangle(canvas, top_left, bottom_right, GREY, 1)
        if index in current:
            cv2.rectangle(canvas, top_left, bottom_right, AMBER, 2)
    cv2.putText(canvas, "coverage", (x0, y0 + MINIMAP_H + 14), FONT, 0.4, GREY, 1)


def draw_hud(
    canvas: np.ndarray,
    views: list[View],
    calibrated: bool,
    extent: float | None,
    sharp: float,
    best_sharp: float,
    current: frozenset[int],
    message: str,
) -> None:
    height = canvas.shape[0]
    filled = {cell for v in views for cell in v.cells}
    draw_minimap(canvas, filled, current)

    gaps = coverage_gaps(views)
    tilted = sum(1 for v in views if v.tilted)
    ready = not gaps
    head = f"views {len(views)} (tilted {tilted})  cells {len(filled)}/{GRID * GRID}"
    cv2.putText(canvas, head, (10, 24), FONT, 0.6, GREEN if ready else AMBER, 2)

    if extent is None:
        cv2.putText(
            canvas,
            "board not detected - get the WHOLE board in frame",
            (10, 48),
            FONT,
            0.55,
            RED,
            2,
        )
    else:
        if extent < EXTENT_LOW:
            note, colour = "too far", AMBER
        elif extent > EXTENT_HIGH:
            note, colour = "TOO CLOSE - move back", RED
        else:
            note, colour = "distance OK", GREEN
        cv2.putText(
            canvas,
            f"board {extent * 100:.0f}% of width "
            f"(target {EXTENT_LOW * 100:.0f}-{EXTENT_HIGH * 100:.0f}) {note}",
            (10, 48),
            FONT,
            0.55,
            colour,
            2,
        )

    ratio = sharp / best_sharp if best_sharp > 1e-6 else 0.0
    sharp_colour = GREEN if ratio >= 0.85 else (AMBER if ratio >= 0.6 else RED)
    cv2.putText(
        canvas,
        f"sharpness {sharp:.0f} (best {best_sharp:.0f})",
        (10, 70),
        FONT,
        0.55,
        sharp_colour,
        2,
    )

    for offset, gap in enumerate(gaps):
        cv2.putText(canvas, gap, (10, 96 + offset * 20), FONT, 0.5, AMBER, 1)
    if ready:
        quality = "READY" if len(views) >= GOOD_VIEWS else f"ok, {GOOD_VIEWS}+ better"
        cv2.putText(
            canvas, f"c to calibrate ({quality})", (10, 96), FONT, 0.6, GREEN, 2
        )

    if message:
        cv2.putText(canvas, message, (10, height - 36), FONT, 0.55, RED, 2)

    keys = "space capture | c calibrate | u undistort | q quit"
    if calibrated:
        keys = "u toggle undistorted | space more views | c recalibrate | q quit"
    cv2.putText(canvas, keys, (10, height - 12), FONT, 0.5, GREY, 1)


def write_result(
    path: Path,
    config: CaptureConfig,
    controls: dict[str, int],
    size: tuple[int, int],
    views: int,
    rms: float,
    matrix: np.ndarray,
    dist: np.ndarray,
    errors: list[float],
) -> None:
    payload = {
        "generated_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "device": config.device,
        "format": {
            "width": size[0],
            "height": size[1],
            "fourcc": config.fourcc,
            "fps": config.fps,
        },
        "board": {"cols": BOARD_COLS, "rows": BOARD_ROWS, "square_mm": SQUARE_MM},
        "views": views,
        "rms_reprojection_error_px": round(rms, 4),
        "worst_view_error_px": round(max(errors), 4) if errors else None,
        "camera_matrix": matrix.tolist(),
        "dist_coeffs": dist.ravel().tolist(),
        "fov_deg": {k: round(v, 2) for k, v in fov_degrees(matrix, size).items()},
        # Intrinsics belong to the imaging setup they were measured under; this
        # is the same fingerprint camera_params.json carries.
        "controls": controls,
    }
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def detect(grey: np.ndarray) -> tuple[bool, np.ndarray | None]:
    """Board corners, refined to sub-pixel, or (False, None)."""
    found, corners = cv2.findChessboardCorners(
        grey, (BOARD_COLS, BOARD_ROWS), FIND_FLAGS
    )
    if not found:
        return False, None
    return True, cv2.cornerSubPix(grey, corners, (11, 11), (-1, -1), SUBPIX_CRITERIA)


def load_views(directory: Path, size: tuple[int, int]) -> list[View]:
    """Re-detect corners in previously saved frames.

    Views used to live only in memory, so a crash or a restart threw away every
    capture. They are written to disk the moment they are accepted now, and this
    reads them back.
    """
    views: list[View] = []
    failed = 0
    for path in sorted(directory.glob("view_*.png")):
        image = cv2.imread(str(path))
        if image is None:
            failed += 1
            continue
        found, corners = detect(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        if not found:
            failed += 1
            continue
        views.append(
            View(corners, image, cells_covered(corners, size), is_tilted(corners))
        )
    note = f", {failed} unusable" if failed else ""
    print(f"resumed {len(views)} views from {directory}{note}", flush=True)
    return views


def run(camera: Camera, controls: dict[str, int], args: argparse.Namespace) -> int:
    grid = object_points()
    views: list[View] = []
    matrix: np.ndarray | None = None
    dist: np.ndarray | None = None
    show_undistorted = False
    size: tuple[int, int] | None = None
    best_sharp = 0.0
    message = ""
    message_at = 0.0
    resumed = False
    args.save_views.mkdir(parents=True, exist_ok=True)

    while True:
        frame = camera.grab()
        image = frame.image
        if size is None:
            size = (image.shape[1], image.shape[0])

        if args.resume and not resumed:
            views = load_views(args.resume, size)
            resumed = True
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners = detect(grey)
        extent = board_extent(corners, size[0]) if found else None
        current = cells_covered(corners, size) if found else frozenset()
        sharp = sharpness(grey, corners if found else None)
        best_sharp = max(best_sharp, sharp)
        if message and time.monotonic() - message_at > MESSAGE_SECONDS:
            message = ""

        if show_undistorted and matrix is not None:
            canvas = cv2.undistort(image, matrix, dist)
        else:
            canvas = image.copy()
            if found:
                cv2.drawChessboardCorners(
                    canvas, (BOARD_COLS, BOARD_ROWS), corners, found
                )

        draw_hud(
            canvas,
            views,
            matrix is not None,
            extent,
            sharp,
            best_sharp,
            current,
            message,
        )
        if found and not show_undistorted:
            tag = "TILTED" if is_tilted(corners) else "flat-on"
            cv2.putText(canvas, tag, (170, 24), FONT, 0.6, GREEN, 2)
        cv2.imshow("intrinsic calibration", canvas)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord(" "):
            if not found:
                message = "no board detected - nothing captured"
                message_at = time.monotonic()
                print("space ignored: board not detected", flush=True)
            else:
                views.append(View(corners, image.copy(), current, is_tilted(corners)))
                cv2.imwrite(str(args.save_views / f"view_{len(views):03d}.png"), image)
                off = "" if EXTENT_LOW <= extent <= EXTENT_HIGH else " (distance off)"
                print(
                    f"captured view {len(views)} covering cells {sorted(current)}, "
                    f"extent {extent * 100:.0f}%, sharpness {sharp:.0f}{off}",
                    flush=True,
                )
        elif key == ord("u"):
            show_undistorted = not show_undistorted
        elif key == ord("c"):
            gaps = coverage_gaps(views)
            if gaps:
                print("not calibrating -- " + "; ".join(gaps), file=sys.stderr)
                continue
            objects = [grid] * len(views)
            images = [v.corners for v in views]
            rms, matrix, dist, rvecs, tvecs = cv2.calibrateCamera(
                objects, images, size, None, None
            )
            errors = per_view_errors(objects, images, rvecs, tvecs, matrix, dist)
            fov = fov_degrees(matrix, size)
            print(f"\n{len(views)} views, RMS reprojection error {rms:.4f} px")
            print(f"  worst view {max(errors):.4f} px")
            print(f"  fx {matrix[0, 0]:.2f}  fy {matrix[1, 1]:.2f}")
            print(f"  cx {matrix[0, 2]:.2f}  cy {matrix[1, 2]:.2f}")
            print("  dist " + " ".join(f"{c:+.4f}" for c in dist.ravel()))
            print(
                f"  FOV  H {fov['horizontal']:.1f} deg  "
                f"V {fov['vertical']:.1f} deg  D {fov['diagonal']:.1f} deg"
            )
            write_result(
                args.output,
                camera.config,
                controls,
                size,
                len(views),
                rms,
                matrix,
                dist,
                errors,
            )
            print(f"  wrote {args.output}")
            show_undistorted = True

    cv2.destroyAllWindows()
    return 0 if matrix is not None else 1


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=Path("tools/camera_params.json"))
    parser.add_argument(
        "--output", type=Path, default=Path("tools/camera_intrinsics.json")
    )
    parser.add_argument(
        "--save-views",
        type=Path,
        default=Path("dataset/calib_views"),
        help="directory each accepted frame is written to as it is captured, so "
        "a crash or a restart never costs the session's work "
        "(default dataset/calib_views)",
    )
    parser.add_argument(
        "--resume",
        type=Path,
        default=None,
        help="reload views from a --save-views directory and carry on",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    config = CaptureConfig.load(args.config)
    camera = Camera(config)
    camera.open()
    try:
        controls = camera.lock_params()
        return run(camera, controls, args)
    finally:
        camera.close()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except CameraError as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
