"""MediaPipe Face Landmarker -> BionicFace external coefficient stream.

Independent process: reads the default webcam, runs MediaPipe's Face
Landmarker (tasks API, blendshapes enabled), maps blendshape scores to the
32 motor channels through a data-driven table, smooths each output with a
One Euro Filter, and sends coefficient frames at a fixed 30Hz to the Rust
ControlService's external input port (task 4, default 127.0.0.1:6100).

This process never talks to the Raspberry Pi or bypasses ControlService --
it only ever writes to the external-input UDP port, which itself goes
through Rust's clamp/jaw-coupling/rate-limiter pipeline like every other
command source.

Usage:
    python3 tools/mediapipe_driver.py --model face_landmarker.task [--preview]
    python3 tools/mediapipe_driver.py --camera-config tools/camera_params.json

The second form opens the camera through tools/camera_capture.py, which locks
and verifies every imaging parameter first. Use it for anything whose output
gets recorded -- see docs/camera/PARAM_LOCK.md for why an unlocked camera makes
a dataset unreproducible.

See README.md for dependency installation and the model download link.
"""

from __future__ import annotations

import argparse
import json
import socket
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

MOTOR_COUNT = 32
# Matches tools/check_neutral.py's tolerance so the preview colours agree with
# what that tool reports as off.
NEUTRAL_TOLERANCE = 0.05
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 6100
SEND_HZ = 30

# --- Blendshape -> channel mapping -----------------------------------------
#
# Each channel maps to a list of (blendshape_name, weight, bias) entries;
# a channel's coefficient is `clamp(sum(weight * blendshape_score + bias for
# each entry), 0, 1)`. Coefficients are the same flat unipolar [0, 1] space
# as the rest of the external-input wire protocol (0 -> that channel's
# minApplied, 1 -> maxApplied, see README's "控制源仲裁与外部输入"), NOT
# control.rs's internal bipolar preset norm space.
#
# The numbers below are derived from the channel calibration rather than
# hand-picked, by two rules:
#
#   bias   -- the entries' biases sum to the coefficient of the channel's
#             calibrated neutral, `(neutralApplied - minApplied) / span`. At a
#             neutral face every score is ~0, so this is exactly where the
#             channel rests, and it has to be the calibrated neutral or the
#             pose is wrong the moment this process takes over.
#   weight -- weights pushing the same way are scaled so their sum equals the
#             travel remaining in that direction, so a blendshape score of 1.0
#             reaches the limit without clamping. Scaling the group
#             proportionally preserves the authored relative influence between
#             blendshapes on one channel.
#
# That makes this table a function of the channel limits: rerun
# `tools/check_neutral.py` after any recalibration, and expect to regenerate
# these numbers if a limit or neutral moves.
#
# MediaPipe blendshape L/R names refer to the SUBJECT's own left/right (not
# camera/mirror left/right), which is assumed here to line up with this
# repo's channel naming (also subject-relative, e.g. `eyebrow_left_inner`).
# Verify this assumption once real hardware is on the bench -- if the face
# mirrors instead of matching, swap the Left/Right blendshape names below.
#
# Weight *signs* are still mostly unverified: they encode which applied
# direction each blendshape moves a channel, which is a mounting fact. Only the
# eyelid signs have hardware evidence so far (see channel 11 below). Use
# `--preview`, whose bars now carry a neutral reference tick, to check each
# direction, its range and its cross-talk one action at a time.
#
# Channels not listed here are left unmapped (the frame sends `null` for
# them, meaning "not driven by this frame" per control.rs's external-input
# format) -- 26/27 are deliberately excluded so Rust's own jaw coupling
# drives them from channel 25, and channels with no obvious blendshape
# analogue (cheek/nose tendons, jaw_horizontal fine detail, neck -- there is
# no head-pose blendshape) are left for a future mapping pass.
BLENDSHAPE_MAP: dict[int, list[tuple[str, float, float]]] = {
    # Eyebrows (0-3). browInnerUp lifts both inner brows together; browDown
    # is per-side.
    #
    # Channel 0's calibrated neutral sits exactly at its maxApplied, so it has
    # no upward travel at all and browInnerUp cannot move it -- that entry is
    # left at its authored weight so check_neutral keeps reporting the clamp,
    # rather than being scaled to zero and disappearing quietly. Its mirror,
    # channel 2, rests near the opposite end (coefficient 0.133), which looks
    # like the same mirrored-mounting story already confirmed on eyelid channel
    # 11. Settle it on hardware: either browInnerUp's sign is inverted here, or
    # this channel's neutral/limits need recalibrating.
    0: [("browInnerUp", 0.6, 1.0), ("browDownRight", -1.0, 0.0)],  # eyebrow_right_inner
    1: [("browDownRight", -0.625, 0.625)],  # eyebrow_right_outer
    # Channel 2 rests at coefficient 0.133, so brow-down has only 15% of the
    # travel that brow-up does. Not a mapping fault -- the neutral is calibrated
    # near the bottom of this channel's range.
    2: [
        ("browInnerUp", 0.867, 0.133),
        ("browDownLeft", -0.133, 0.0),
    ],  # eyebrow_left_inner
    3: [("browDownLeft", -0.55, 0.55)],  # eyebrow_left_outer
    # Eyes (8-13). 8/13 are each a *single shared* mechanism driving both
    # eyeballs (see config.py's MOTOR_MAP comments), so both eyes' gaze
    # blendshapes are averaged into one signed value around the neutral.
    8: [  # eye_horizontal (shared gaze X); out has 3x the travel of in
        ("eyeLookOutRight", 0.375, 0.25),
        ("eyeLookInRight", -0.125, 0.0),
        ("eyeLookInLeft", 0.375, 0.0),
        ("eyeLookOutLeft", -0.125, 0.0),
    ],
    13: [  # eye_vertical (shared gaze Y)
        ("eyeLookUpRight", 0.143, 0.714),
        ("eyeLookDownRight", -0.357, 0.0),
        ("eyeLookUpLeft", 0.143, 0.0),
        ("eyeLookDownLeft", -0.357, 0.0),
    ],
    # Eyelids (9-12): the blendshape is eye *closure*, so each weight points
    # from the resting-open coefficient towards that channel's closed end.
    # Which end that is comes from control.rs's BLINK_CLOSE_DIRECTIONS, which
    # was checked against real hardware: closed is the high end for 9/10/12 and
    # the low end for 11, whose servo is mounted mirrored -- hence the negative
    # weight there. Channel 12's direction is still marked unconfirmed in that
    # table; if idle blink looks wrong on the right lower lid, flip 12 too.
    # Channels 9 and 12 rest at 0.722, leaving only 0.278 of closing travel, so
    # a full blink will not look like much until those neutrals are revisited.
    9: [("eyeBlinkLeft", 0.278, 0.722)],  # eye_left_upper
    10: [("eyeBlinkLeft", 0.578, 0.422)],  # eye_left_lower
    11: [("eyeBlinkRight", -0.677, 0.677)],  # eye_right_upper (mirrored mount)
    12: [("eyeBlinkRight", 0.278, 0.722)],  # eye_right_lower
    # Mouth (14-23). Upper/lower lip channels respond to pucker (both sides)
    # plus their own side's "upper lip up"/"lower lip down"; the corner channels
    # are driven by smile against frown, with the lower corners moving opposite
    # to the upper ones.
    14: [
        ("mouthUpperUpLeft", 0.139, 0.75),
        ("mouthPucker", 0.111, 0.0),
    ],  # upper_lip_left
    15: [("mouthPucker", 0.75, 0.25)],  # upper_lip_mid
    16: [
        ("mouthUpperUpRight", 0.435, 0.217),
        ("mouthPucker", 0.348, 0.0),
    ],  # upper_lip_right
    17: [
        ("mouthSmileRight", 0.556, 0.444),
        ("mouthFrownRight", -0.444, 0.0),
    ],  # mouth_right_corner_upper
    18: [
        ("mouthFrownRight", 0.5, 0.5),
        ("mouthSmileRight", -0.5, 0.0),
    ],  # mouth_right_corner_lower
    19: [
        ("mouthSmileLeft", 0.55, 0.45),
        ("mouthFrownLeft", -0.45, 0.0),
    ],  # mouth_left_corner_upper
    20: [
        ("mouthFrownLeft", 0.667, 0.333),
        ("mouthSmileLeft", -0.333, 0.0),
    ],  # mouth_left_corner_lower
    21: [
        ("mouthLowerDownLeft", 0.486, 0.125),
        ("mouthPucker", 0.389, 0.0),
    ],  # lower_lip_left
    22: [
        ("mouthLowerDownRight", 0.556, 0.0),
        ("mouthPucker", 0.444, 0.0),
    ],  # lower_lip_right
    23: [("mouthPucker", 0.364, 0.636)],  # lower_lip_mid_tendon
    # Jaw (24/25). jawOpen drives the primary jaw-open channel per the task
    # spec; 26/27 are intentionally absent -- control.rs's jaw coupling
    # drives them from 25's target, not this process.
    24: [("jawLeft", 0.508, 0.492), ("jawRight", -0.492, 0.0)],  # jaw_horizontal
    25: [("jawOpen", 0.471, 0.529)],  # jaw_right_upper (main jaw-open axis)
}

# Every mapped channel gets its own One Euro Filter instance; channels not
# in BLENDSHAPE_MAP never appear here and always send null.
_MAPPED_CHANNEL_IDS = tuple(sorted(BLENDSHAPE_MAP.keys()))


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def apply_blendshape_map(scores: dict[str, float]) -> list[float | None]:
    """Maps a {blendshape_name: score} dict to 32 coefficients (None for
    unmapped channels)."""
    coefficients: list[float | None] = [None] * MOTOR_COUNT
    for channel_id, entries in BLENDSHAPE_MAP.items():
        total = 0.0
        for blendshape_name, weight, bias in entries:
            total += weight * scores.get(blendshape_name, 0.0) + bias
        coefficients[channel_id] = clamp01(total)
    return coefficients


class OneEuroFilter:
    """Minimal One Euro Filter (Casiez et al. 2012): a low-pass filter whose
    cutoff frequency increases with signal speed, so it stays smooth when
    still and responsive when moving fast. `min_cutoff` sets the baseline
    smoothing at zero speed; `beta` controls how much speed increases the
    cutoff; `d_cutoff` smooths the speed estimate itself.
    """

    def __init__(
        self, min_cutoff: float = 1.0, beta: float = 0.007, d_cutoff: float = 1.0
    ) -> None:
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self._x_prev: float | None = None
        self._dx_prev = 0.0
        self._t_prev: float | None = None

    @staticmethod
    def _alpha(cutoff: float, dt: float) -> float:
        tau = 1.0 / (2.0 * 3.14159265358979 * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def filter(self, value: float, timestamp: float) -> float:
        if self._t_prev is None:
            self._x_prev = value
            self._t_prev = timestamp
            return value

        dt = max(timestamp - self._t_prev, 1e-6)
        self._t_prev = timestamp

        dx = (value - self._x_prev) / dt
        dx_alpha = self._alpha(self.d_cutoff, dt)
        dx_hat = dx_alpha * dx + (1.0 - dx_alpha) * self._dx_prev
        self._dx_prev = dx_hat

        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        x_alpha = self._alpha(cutoff, dt)
        x_hat = x_alpha * value + (1.0 - x_alpha) * self._x_prev
        self._x_prev = x_hat
        return x_hat


@dataclass
class FilterBank:
    """One OneEuroFilter per mapped channel, all sharing the same tuning."""

    min_cutoff: float
    beta: float
    d_cutoff: float
    filters: dict[int, OneEuroFilter] = field(default_factory=dict)

    def apply(
        self, coefficients: list[float | None], timestamp: float
    ) -> list[float | None]:
        smoothed = list(coefficients)
        for channel_id in _MAPPED_CHANNEL_IDS:
            value = coefficients[channel_id]
            if value is None:
                continue
            if channel_id not in self.filters:
                self.filters[channel_id] = OneEuroFilter(
                    self.min_cutoff, self.beta, self.d_cutoff
                )
            smoothed[channel_id] = self.filters[channel_id].filter(value, timestamp)
        return smoothed


def build_frame(seq: int, coefficients: list[float | None]) -> dict[str, Any]:
    return {
        "seq": seq,
        "timestampNs": time.time_ns(),
        "coefficients": coefficients,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="MediaPipe -> BionicFace external coefficient driver"
    )
    parser.add_argument(
        "--host",
        default=DEFAULT_HOST,
        help=f"External input host (default {DEFAULT_HOST})",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help=f"External input port (default {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--camera", type=int, default=0, help="OpenCV camera index (default 0)"
    )
    parser.add_argument(
        "--no-neutral-reference",
        action="store_true",
        help=(
            "Omit the neutral reference tick from --preview's bars. The tick "
            "marks each channel's calibrated neutral, which is what the bars are "
            "read against when checking a channel's direction and range."
        ),
    )
    parser.add_argument(
        "--camera-config",
        type=Path,
        default=None,
        help=(
            "Path to a camera_capture.py parameter set (e.g. tools/camera_params.json). "
            "When given, the camera is opened through camera_capture.Camera and its "
            "imaging parameters are locked and verified -- required for capture runs "
            "that must be reproducible. When omitted, a plain VideoCapture is used."
        ),
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=Path(__file__).resolve().with_name("face_landmarker.task"),
        help="Path to the MediaPipe face_landmarker.task model file",
    )
    parser.add_argument(
        "--min-cutoff",
        type=float,
        default=1.0,
        help="One Euro Filter min_cutoff (default 1.0)",
    )
    parser.add_argument(
        "--beta", type=float, default=0.007, help="One Euro Filter beta (default 0.007)"
    )
    parser.add_argument(
        "--d-cutoff",
        type=float,
        default=1.0,
        help="One Euro Filter d_cutoff (default 1.0)",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Show a debug window (camera + landmarks + bars)",
    )
    return parser.parse_args()


def draw_preview(
    surface,
    frame_bgr,
    landmarks,
    coefficients: list[float | None],
    neutral: list[float | None] | None = None,
) -> None:
    """Renders the camera frame (mirrored to a pygame surface), landmark
    dots, and a bar chart of the mapped output coefficients. Imports pygame
    lazily so --preview is the only code path requiring a display.

    `neutral` is the coefficient each channel should sit at for a neutral face,
    drawn as a tick on every bar. Without it the bars only show "some value came
    out"; with it you can see, per channel, whether a deliberate expression
    moves the right way, how much of the channel's travel it actually uses, and
    which other channels moved when they should not have.
    """
    import pygame

    height, width = frame_bgr.shape[:2]
    # OpenCV is BGR, row-major (H, W, 3); pygame wants (W, H, 3) RGB.
    rgb = frame_bgr[:, :, ::-1]
    cam_surface = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))
    surface.blit(cam_surface, (0, 0))

    if landmarks:
        for point in landmarks:
            x, y = int(point.x * width), int(point.y * height)
            pygame.draw.circle(surface, (0, 255, 0), (x, y), 1)

    bar_x0 = width + 10
    bar_w = 200
    font = pygame.font.SysFont("monospace", 12)
    for row, channel_id in enumerate(_MAPPED_CHANNEL_IDS):
        value = coefficients[channel_id]
        reference = None if neutral is None else neutral[channel_id]
        y = 10 + row * 14
        pygame.draw.rect(surface, (60, 60, 60), (bar_x0, y, bar_w, 10))

        delta = None
        if value is not None:
            if reference is not None:
                delta = value - reference
            on_target = delta is not None and abs(delta) <= NEUTRAL_TOLERANCE
            colour = (80, 200, 120) if on_target or delta is None else (230, 170, 70)
            pygame.draw.rect(surface, colour, (bar_x0, y, int(bar_w * value), 10))

        if reference is not None:
            tick_x = bar_x0 + int(bar_w * reference)
            pygame.draw.line(
                surface, (235, 235, 235), (tick_x, y - 2), (tick_x, y + 12)
            )

        label = font.render(f"{channel_id:02d}", True, (220, 220, 220))
        surface.blit(label, (bar_x0 - 24, y))
        if delta is not None:
            readout = font.render(f"{delta:+.2f}", True, (170, 180, 185))
            surface.blit(readout, (bar_x0 + bar_w + 6, y))


def open_frame_source(
    args: argparse.Namespace, cv2: Any
) -> tuple[Any, tuple[int, int], Any]:
    """Opens the camera and returns (read, (width, height), close).

    `read()` returns a BGR frame, or None when the stream ends cleanly.

    Two modes on purpose. With --camera-config the frames come from
    camera_capture.Camera, which negotiates the format strictly and locks every
    imaging parameter -- that is the mode any recorded dataset must use. Without
    it a plain VideoCapture is used, but the format is still requested
    explicitly: leaving it to the driver's default is how this pipeline ended up
    negotiating YUYV 640x480, which on the WHEELTEC C100 caps at 5 fps and
    measured 3.3 -- while this process happily claimed to send at 30Hz.
    """
    if args.camera_config is not None:
        from camera_capture import Camera, CaptureConfig

        config = CaptureConfig.load(args.camera_config)
        camera = Camera(config)
        camera.open()
        locked = camera.lock_params()
        print(
            f"camera {config.device}: {camera.negotiated_format()}, "
            f"{len(locked)} imaging parameters locked"
        )
        return (
            (lambda: camera.grab().image),
            (config.width, config.height),
            camera.close,
        )

    capture = cv2.VideoCapture(args.camera)
    if not capture.isOpened():
        raise SystemExit(f"Could not open camera index {args.camera}")
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    capture.set(cv2.CAP_PROP_FPS, SEND_HZ)

    fourcc = int(capture.get(cv2.CAP_PROP_FOURCC))
    negotiated_fps = capture.get(cv2.CAP_PROP_FPS)
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)) or 640
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 480
    print(
        f"camera index {args.camera}: "
        f"{''.join(chr((fourcc >> shift) & 0xFF) for shift in (0, 8, 16, 24))} "
        f"{width}x{height} @ {negotiated_fps:g}fps"
    )
    if negotiated_fps < SEND_HZ * 0.9:
        print(
            f"WARNING: the camera negotiated {negotiated_fps:g} fps but this process "
            f"sends at {SEND_HZ}Hz, so most frames will be stale repeats. Pick a mode "
            f"the camera can actually sustain (`v4l2-ctl --list-formats-ext`), or pass "
            f"--camera-config to negotiate and lock a known-good one."
        )

    def read() -> Any:
        ok, frame = capture.read()
        return frame if ok else None

    return read, (width, height), capture.release


def main() -> None:
    args = parse_args()

    # Heavy/optional dependencies are imported here, not at module scope, so
    # apply_blendshape_map/OneEuroFilter/build_frame stay importable and
    # unit-testable on a machine without mediapipe/opencv/pygame installed.
    import cv2
    import mediapipe as mp

    # Matches the import style in Google's official FaceLandmarker Python
    # guide verbatim (mp.tasks.* attribute access), rather than guessing at
    # submodule import paths.
    base_options_cls = mp.tasks.BaseOptions
    face_landmarker_cls = mp.tasks.vision.FaceLandmarker
    face_landmarker_options_cls = mp.tasks.vision.FaceLandmarkerOptions
    running_mode = mp.tasks.vision.RunningMode

    if not args.model.exists():
        raise SystemExit(
            f"Model file not found: {args.model}\n"
            "Download face_landmarker.task per README.md's MediaPipe section."
        )

    options = face_landmarker_options_cls(
        base_options=base_options_cls(model_asset_path=str(args.model)),
        output_face_blendshapes=True,
        output_facial_transformation_matrixes=False,
        running_mode=running_mode.VIDEO,
        num_faces=1,
    )
    landmarker = face_landmarker_cls.create_from_options(options)

    read_frame, (cam_width, cam_height), close_camera = open_frame_source(args, cv2)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    filter_bank = FilterBank(
        min_cutoff=args.min_cutoff, beta=args.beta, d_cutoff=args.d_cutoff
    )

    preview_surface = None
    neutral_reference: list[float | None] | None = None
    if args.preview:
        import pygame

        pygame.init()
        if not args.no_neutral_reference:
            # The reference comes from check_neutral so there is one definition
            # of "where neutral should be", rather than this file deriving it.
            from check_neutral import DEFAULT_CONFIG, neutral_targets

            neutral_reference = neutral_targets(
                json.loads(DEFAULT_CONFIG.read_text(encoding="utf-8"))
            )
        preview_surface = pygame.display.set_mode(
            (cam_width + 320, max(cam_height, 32 * 14))
        )
        pygame.display.set_caption("mediapipe_driver preview")

    seq = 0
    frame_interval = 1.0 / SEND_HZ
    next_send_at = time.monotonic()
    start_time = time.monotonic()

    try:
        while True:
            frame_bgr = read_frame()
            if frame_bgr is None:
                break

            if preview_surface is not None:
                import pygame

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return

            rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            timestamp_ms = int((time.monotonic() - start_time) * 1000)
            result = landmarker.detect_for_video(mp_image, timestamp_ms)

            scores: dict[str, float] = {}
            landmarks = None
            if result.face_blendshapes:
                scores = {
                    category.category_name: category.score
                    for category in result.face_blendshapes[0]
                }
            if result.face_landmarks:
                landmarks = result.face_landmarks[0]

            raw_coefficients = apply_blendshape_map(scores)
            smoothed = filter_bank.apply(raw_coefficients, time.monotonic())

            now = time.monotonic()
            if now >= next_send_at:
                seq += 1
                frame = build_frame(seq, smoothed)
                sock.sendto(json.dumps(frame).encode("utf-8"), (args.host, args.port))
                next_send_at += frame_interval
                if next_send_at < now:
                    next_send_at = now + frame_interval

            if preview_surface is not None:
                draw_preview(
                    preview_surface, frame_bgr, landmarks, smoothed, neutral_reference
                )
                import pygame

                pygame.display.flip()
    finally:
        close_camera()
        if preview_surface is not None:
            import pygame

            pygame.quit()


if __name__ == "__main__":
    main()
