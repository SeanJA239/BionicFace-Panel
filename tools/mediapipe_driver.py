"""MediaPipe Face Landmarker -> BionicFace external coefficient stream.

Independent process: reads the default webcam, runs MediaPipe's Face
Landmarker (tasks API, blendshapes and transformation matrix enabled), rebases
the blendshape scores against a per-subject rest baseline, maps them to the 32
motor channels through a data-driven table and head pose to the two neck
channels, smooths each output with a One Euro Filter, and
sends coefficient frames at a fixed 30Hz to the Rust ControlService's external
input port (task 4, default 127.0.0.1:6100).

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

MOTOR_COUNT = 33
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
# Weight *signs* encode which applied direction each blendshape moves a channel,
# which is a mounting fact. The subject's-right servos turn out to be mounted
# mirrored relative to the subject's-left ones, so a symmetric expression needs
# opposite norm signs on the two sides.
#
# That was read off the emotion presets, which are authored on hardware and
# reach their poses, so they serve as ground truth. Restricting the comparison
# to the presets that should be left/right symmetric (喜悦/悲伤/愤怒/惊讶/恐惧 --
# 困惑 and wink are deliberately asymmetric), five right-side channels come out
# unambiguously mirrored: 0, 1, 11, 17, 18. Their weights are negated here
# relative to their left-side twins. Channel 11 was independently confirmed on
# hardware for the idle blink (see control.rs BLINK_CLOSE_DIRECTIONS), which is
# what gives this inference its credibility.
#
# Three right-side channels contradict themselves across those presets and are
# left alone until someone checks them on hardware: 12 (eye_right_lower, 2 vs 2),
# 16 (upper_lip_right, 1 vs 1) and 22 (lower_lip_right, 2 vs 1). Channel 25 has
# no mapped left twin (26/27 come from Rust's jaw coupling), so nothing to
# compare against.
#
# Everything else about the signs is still unverified. Use `--preview`, whose
# bars carry a neutral reference tick, to check each direction, its range and
# its cross-talk one action at a time.
#
# Channels not listed here are left unmapped (the frame sends `null` for
# them, meaning "not driven by this frame" per control.rs's external-input
# format) -- 26/27 are deliberately excluded so Rust's own jaw coupling
# drives them from channel 25, and channels with no obvious blendshape
# analogue (cheek/nose tendons, jaw_horizontal fine detail) are left for a
# future mapping pass. The neck (30/31) is driven too, but not from here:
# there is no head-pose blendshape, so it comes from the facial transformation
# matrix instead -- see neck_coefficients.
BLENDSHAPE_MAP: dict[int, list[tuple[str, float, float]]] = {
    # Eyebrows (0-3). browInnerUp lifts both inner brows together; browDown and
    # browOuterUp are per-side.
    #
    # The outer brows used to map browDown only, so they could travel downward
    # and never up -- half of why a raised eyebrow produced nothing. On a static
    # neutral face browOuterUpLeft/Right are in fact the two highest-reading of
    # all 52 blendshapes (0.361 and 0.147), so the signal was there and was being
    # discarded. They are the only per-side brow *raise* MediaPipe offers:
    # browInnerUp is a single shared coefficient, so the inner brows cannot move
    # asymmetrically at all and a one-sided raise has to come from these two.
    #
    # Channel 0's calibrated neutral sits exactly at its maxApplied, so it can
    # only move downward. Once mirrored, that is the raising direction, and
    # browInnerUp gets the channel's full travel -- the problem that looked like
    # it needed a limit change disappears with the sign.
    #
    # browDownRight is left mapped here with no room to move, so check_neutral
    # keeps reporting it rather than it being scaled to zero and disappearing.
    # It probably does not belong on this channel at all: 愤怒, the preset that
    # lowers the brows, leaves channel 0 at 0.00 and does the lowering with the
    # outer brow channels instead.
    0: [("browInnerUp", -1.0, 1.0), ("browDownRight", 1.0, 0.0)],  # eyebrow_right_inner
    # Channel 1 is mirrored, so a rising coefficient lowers the brow on the face
    # and browOuterUpRight has to push the other way -- towards 0, across the
    # channel's whole 0.625 of downward coefficient travel.
    1: [
        ("browDownRight", 0.375, 0.625),
        ("browOuterUpRight", -0.625, 0.0),
    ],  # eyebrow_right_outer
    # Channel 2 rests at coefficient 0.133, so brow-down has only 15% of the
    # travel that brow-up does. Not a mapping fault -- the neutral is calibrated
    # near the bottom of this channel's range.
    2: [
        ("browInnerUp", 0.867, 0.133),
        ("browDownLeft", -0.133, 0.0),
    ],  # eyebrow_left_inner
    # Channel 3 is not mirrored, so raising the brow raises the coefficient.
    3: [
        ("browDownLeft", -0.55, 0.55),
        ("browOuterUpLeft", 0.45, 0.0),
    ],  # eyebrow_left_outer
    # Eyes (8-13). 8/13 are each a *single shared* mechanism driving both
    # eyeballs (see config.py's MOTOR_MAP comments), so both eyes' gaze
    # blendshapes are averaged into one signed value around the neutral.
    # eye_horizontal: hardware-measured 2026-08-29 (see
    # docs/hardware/CHANNEL_VERIFICATION.md) -- straight ahead is applied 110
    # and everything above is a mechanical dead zone, so neutral == max
    # (coefficient 1.0) and gaze deviates ONE way only: lower applied moves the
    # gaze to the SUBJECT'S RIGHT (the observer's left). Only the look-right
    # pair is mapped; looking left is mechanically unreachable.
    8: [  # eye_horizontal (shared gaze X)
        ("eyeLookOutRight", -0.5, 1.0),
        ("eyeLookInLeft", -0.5, 0.0),
    ],
    13: [  # eye_vertical (shared gaze Y)
        ("eyeLookUpRight", 0.143, 0.714),
        ("eyeLookDownRight", -0.357, 0.0),
        ("eyeLookUpLeft", 0.143, 0.0),
        ("eyeLookDownLeft", -0.357, 0.0),
    ],
    # Eyelids (9-12): the blendshape is eye *closure*, so each weight points
    # from the resting-open coefficient towards that channel's closed end.
    # Which end that is comes from control.rs's BLINK_CLOSE_DIRECTIONS, checked
    # against real hardware: closed is the high end for the left lids 9/10 and
    # the low end for both right lids 11/12, whose servos are mounted mirrored
    # -- hence the negative weights. Channel 11 was confirmed via idle blink,
    # channel 12 by single-channel jogging (2026-08-29, see
    # docs/hardware/CHANNEL_VERIFICATION.md).
    # Channel 9 rests at 0.722 with only 0.278 of closing travel, so a full
    # left-lid blink will not look like much until that neutral is revisited.
    9: [("eyeBlinkLeft", 0.278, 0.722)],  # eye_left_upper
    10: [("eyeBlinkLeft", 0.578, 0.422)],  # eye_left_lower
    11: [("eyeBlinkRight", -0.677, 0.677)],  # eye_right_upper (mirrored mount)
    12: [("eyeBlinkRight", -0.722, 0.722)],  # eye_right_lower (mirrored mount)
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
        ("mouthSmileRight", -0.444, 0.444),
        ("mouthFrownRight", 0.556, 0.0),
    ],  # mouth_right_corner_upper
    18: [
        ("mouthFrownRight", -0.5, 0.5),
        ("mouthSmileRight", 0.5, 0.0),
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

_MAPPED_CHANNEL_IDS = tuple(sorted(BLENDSHAPE_MAP.keys()))

# --- Head pose -> neck ------------------------------------------------------
#
# MediaPipe has no head-pose blendshape, so the neck is driven from the facial
# transformation matrix instead of BLENDSHAPE_MAP.
#
# The mechanism sets a hard ceiling on what is even worth extracting: the neck
# is two servos, `neck_left` and `neck_right`, mounted on opposite sides. They
# reach exactly two of the head's three rotational degrees of freedom -- moving
# together to nod (pitch) and oppositely to tilt (roll). Yaw (shaking the head)
# would need a third servo on a vertical axis, so it is unreachable and is not
# mapped at all, however well it can be detected.
NECK_CHANNEL_IDS = (30, 31)
# Both channels run 75..105 degrees, but their measured level-head neutrals
# differ: ch30 rests at 90 (coefficient 0.5), ch31 at 95.5 (0.6833, measured on
# hardware 2026-08-29 -- see docs/hardware/CHANNEL_VERIFICATION.md). A pure nod
# or tilt must move both servos by the same DEGREES, so the shared budget is
# set in degrees by the tightest side across both channels:
# min(15, 15, 9.5, 20.5) = 9.5 deg total, half per mode. Full nod plus full
# tilt then lands ch31 exactly on its upper limit rather than relying on the
# clamp, the same rule the blendshape weights above follow. Regenerate these if
# the neck limits or neutrals move.
NECK_REST = (0.5, 20.5 / 30.0)
NECK_MODE_BUDGET = 4.75 / 30.0
# Frames averaged for the automatic baseline; ~1s at SEND_HZ.
NECK_BASELINE_FRAMES = 30
# Same idea for the blendshape rest baseline (see BlendshapeBaseline).
BLENDSHAPE_BASELINE_FRAMES = 30

# Everything this process can drive. Each gets its own One Euro Filter and its
# own preview row; channels outside this set always send null.
_DRIVEN_CHANNEL_IDS = tuple(sorted({*_MAPPED_CHANNEL_IDS, *NECK_CHANNEL_IDS}))


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def clamp_signed(value: float) -> float:
    return max(-1.0, min(1.0, value))


def neck_coefficients(
    pitch_deg: float,
    roll_deg: float,
    pitch_range_deg: float,
    roll_range_deg: float,
) -> tuple[float, float]:
    """Baseline-corrected head pitch/roll in degrees -> (ch30, ch31).

    `*_range_deg` is the head rotation that maps to that mode's full travel;
    beyond it the output saturates. The compression matters: a person's nod or
    tilt runs well past the mechanism's +-15 degrees, so feeding raw angles in
    would park both channels on their limits for all but the smallest movements.

    Which physical tilt a positive differential actually produces is unverified,
    the same caveat as the rest of this file's direction signs. face_visualizer
    draws the pair as dev(30) - dev(31), so the preview and the twin face agree
    with each other whether or not they agree with the hardware.
    """
    common = clamp_signed(pitch_deg / pitch_range_deg) * NECK_MODE_BUDGET
    differential = clamp_signed(roll_deg / roll_range_deg) * NECK_MODE_BUDGET
    return (
        clamp01(NECK_REST[0] + common + differential),
        clamp01(NECK_REST[1] + common - differential),
    )


def head_pose_degrees(matrix: Any, cv2: Any) -> tuple[float, float, float]:
    """(pitch, yaw, roll) in degrees from MediaPipe's 4x4 transformation matrix.

    The matrix maps a canonical face model onto the detected face, so its
    upper-left 3x3 is the head's rotation; RQDecomp3x3 gives that rotation's
    Euler angles directly in degrees.
    """
    import numpy as np

    rotation = np.asarray(matrix, dtype=float)[:3, :3]
    pitch, yaw, roll = cv2.RQDecomp3x3(rotation)[0]
    return float(pitch), float(yaw), float(roll)


class HeadPoseBaseline:
    """The head pose that counts as level, subtracted from every reading.

    Head pose carries a per-subject and per-placement offset -- a static print
    measured 4.35 degrees of pitch, not 0 -- so raw angles would hold the neck
    off-centre for as long as the driver ran. Averaging the opening frames is
    enough precision because the pose is stable to well under a tenth of a
    degree on a still subject; the operator just has to hold level while it
    fills. Until it does, `observe` returns None and the caller should leave the
    neck at neutral rather than drive it from an unknown zero.
    """

    def __init__(self, frames: int, fixed: tuple[float, float] | None = None) -> None:
        self._frames = frames
        self._pitch_sum = 0.0
        self._roll_sum = 0.0
        self._count = 0
        self._value = fixed
        self._fixed = fixed is not None

    @property
    def value(self) -> tuple[float, float] | None:
        return self._value

    def observe(self, pitch: float, roll: float) -> tuple[float, float] | None:
        """Feeds one reading in and returns the baseline, or None if not ready."""
        if self._fixed or self._value is not None:
            return self._value
        self._pitch_sum += pitch
        self._roll_sum += roll
        self._count += 1
        if self._count >= self._frames:
            self._value = (
                self._pitch_sum / self._count,
                self._roll_sum / self._count,
            )
        return self._value


class BlendshapeBaseline:
    """The blendshape vector that counts as a neutral face, rebased out of
    every reading.

    Every mapping entry's bias assumes a neutral face scores ~0 on its
    blendshape, but resting scores are per-subject: the neutral A4 print reads
    browOuterUpLeft 0.361 against browOuterUpRight 0.147 -- 2.5x apart on one
    still face -- so without this the outer brows sit visibly raised and
    lopsided the moment the driver takes over. Averaging the opening frames is
    enough precision because scores on a still subject wobble by only sigma
    0.02-0.04.

    `rebase` maps each score through (score - rest) / (1 - rest), clamped to
    [0, 1]. Three deliberate semantics: rest lands on 0, so every channel falls
    back to its calibrated-neutral bias; a full activation still reaches 1, so
    the subtraction costs no travel; and a reading below rest -- "more relaxed
    than the calibration pose" -- clips to 0, because the map assigns it no
    meaning.
    """

    def __init__(self, frames: int, fixed: dict[str, float] | None = None) -> None:
        self._frames = frames
        self._sums: dict[str, float] = {}
        self._count = 0
        self._value = dict(fixed) if fixed is not None else None
        self._names_checked = False

    @property
    def value(self) -> dict[str, float] | None:
        return self._value

    @property
    def progress(self) -> tuple[int, int]:
        return self._count, self._frames

    def _check_names(self, scores: dict[str, float]) -> None:
        # A loaded baseline naming different blendshapes than the model outputs
        # would silently rebase some scores and leave others raw; refuse loudly
        # instead. Checked against the first real frame because the model's
        # name set is only known once it has produced one.
        if self._names_checked:
            return
        missing = sorted(set(scores) - set(self._value or {}))
        unknown = sorted(set(self._value or {}) - set(scores))
        if missing or unknown:
            raise SystemExit(
                "blendshape baseline does not match the model's outputs -- "
                f"absent from baseline: {missing or 'none'}; "
                f"unknown to the model: {unknown or 'none'}"
            )
        self._names_checked = True

    def observe(self, scores: dict[str, float]) -> dict[str, float] | None:
        """Feeds one frame's scores in and returns the baseline, or None while
        still collecting."""
        if self._value is not None:
            self._check_names(scores)
            return self._value
        for name, score in scores.items():
            self._sums[name] = self._sums.get(name, 0.0) + score
        self._count += 1
        if self._count >= self._frames:
            self._value = {
                name: total / self._count for name, total in self._sums.items()
            }
        return self._value

    def rebase(self, scores: dict[str, float]) -> dict[str, float]:
        # Callers only get here after observe() returned a baseline.
        rest_vector = self._value or {}
        rebased: dict[str, float] = {}
        for name, score in scores.items():
            rest = rest_vector.get(name, 0.0)
            # A rest score at 1.0 leaves no travel to rescale into; the max()
            # guard degrades that pathological case to "anything above rest is
            # fully on" instead of dividing by zero.
            rebased[name] = clamp01((score - rest) / max(1.0 - rest, 1e-6))
        return rebased

    def top_entries(self, count: int = 5) -> list[tuple[str, float]]:
        ranked = sorted((self._value or {}).items(), key=lambda item: -item[1])
        return ranked[:count]


def load_blendshape_baseline(path: Path) -> dict[str, float]:
    """Reads a {blendshape_name: rest_score} JSON written by --baseline-save."""
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not all(
        isinstance(name, str) and isinstance(value, (int, float))
        for name, value in data.items()
    ):
        raise SystemExit(f"{path} is not a {{blendshape_name: rest_score}} object")
    return {name: float(value) for name, value in data.items()}


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
    """One OneEuroFilter per driven channel, all sharing the same tuning."""

    min_cutoff: float
    beta: float
    d_cutoff: float
    filters: dict[int, OneEuroFilter] = field(default_factory=dict)

    def apply(
        self, coefficients: list[float | None], timestamp: float
    ) -> list[float | None]:
        smoothed = list(coefficients)
        for channel_id in _DRIVEN_CHANNEL_IDS:
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


def neck_baseline_arg(value: str) -> tuple[float, float]:
    """Parses a `PITCH,ROLL` degree pair for --neck-baseline."""
    parts = value.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("expected PITCH,ROLL in degrees")
    try:
        return float(parts[0]), float(parts[1])
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"expected two numbers, got {value!r}"
        ) from None


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
        "--no-neck",
        action="store_true",
        help=(
            "Leave the neck channels undriven. The head pose is still decoded "
            "and reported; this only stops it reaching channels 30/31."
        ),
    )
    parser.add_argument(
        "--neck-pitch-range",
        type=float,
        default=20.0,
        help=(
            "Head pitch in degrees mapping to the neck's full nod travel "
            "(default 20.0). Raise it to make nodding less sensitive."
        ),
    )
    parser.add_argument(
        "--neck-roll-range",
        type=float,
        default=25.0,
        help=(
            "Head roll in degrees mapping to the neck's full tilt travel "
            "(default 25.0). Raise it to make tilting less sensitive."
        ),
    )
    parser.add_argument(
        "--neck-baseline",
        type=neck_baseline_arg,
        default=None,
        metavar="PITCH,ROLL",
        help=(
            "Head pose in degrees to treat as level, skipping the automatic "
            f"calibration. Without it the first {NECK_BASELINE_FRAMES} frames "
            "with a face are averaged, so hold the head level and still while "
            "the driver starts."
        ),
    )
    parser.add_argument(
        "--baseline-frames",
        type=int,
        default=BLENDSHAPE_BASELINE_FRAMES,
        help=(
            "Frames with a face averaged into the blendshape rest baseline "
            f"(default {BLENDSHAPE_BASELINE_FRAMES}); hold a neutral expression "
            "while it fills. Mapped channels rest at their calibrated neutral "
            "until it completes."
        ),
    )
    parser.add_argument(
        "--no-blendshape-baseline",
        action="store_true",
        help=(
            "Feed raw blendshape scores to the map without rest-baseline "
            "rebasing. Per-subject resting bias then lands on the channels: "
            "the outer brows sit off-neutral and lopsided on a face whose "
            "resting browOuterUp reads high."
        ),
    )
    parser.add_argument(
        "--baseline-save",
        type=Path,
        default=None,
        help=(
            "Write the captured rest baseline to this JSON so a later run on "
            "the same subject can --baseline-load it. Only written when the "
            "baseline is captured this run, not when it was loaded."
        ),
    )
    parser.add_argument(
        "--baseline-load",
        type=Path,
        default=None,
        help=(
            "Skip capture and use this rest baseline. Refused if its name set "
            "differs from what the model outputs."
        ),
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
    status: str | None = None,
) -> None:
    """Renders the camera frame (mirrored to a pygame surface), landmark
    dots, and a bar chart of the mapped output coefficients. Imports pygame
    lazily so --preview is the only code path requiring a display.

    `neutral` is the coefficient each channel should sit at for a neutral face,
    drawn as a tick on every bar. Without it the bars only show "some value came
    out"; with it you can see, per channel, whether a deliberate expression
    moves the right way, how much of the channel's travel it actually uses, and
    which other channels moved when they should not have. `status` is a
    transient banner (e.g. baseline-calibration progress) drawn over the frame.
    """
    import pygame

    height, width = frame_bgr.shape[:2]
    # OpenCV is BGR, row-major (H, W, 3); pygame wants (W, H, 3) RGB.
    rgb = frame_bgr[:, :, ::-1]
    cam_surface = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))
    surface.blit(cam_surface, (0, 0))

    if status:
        banner_font = pygame.font.SysFont("monospace", 16, bold=True)
        surface.blit(banner_font.render(status, True, (255, 210, 80)), (10, 8))

    if landmarks:
        for point in landmarks:
            x, y = int(point.x * width), int(point.y * height)
            pygame.draw.circle(surface, (0, 255, 0), (x, y), 1)

    bar_x0 = width + 10
    bar_w = 200
    font = pygame.font.SysFont("monospace", 12)
    for row, channel_id in enumerate(_DRIVEN_CHANNEL_IDS):
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

    for flag, value in (
        ("--neck-pitch-range", args.neck_pitch_range),
        ("--neck-roll-range", args.neck_roll_range),
    ):
        if value <= 0:
            raise SystemExit(f"{flag} must be positive, got {value}")
    if args.baseline_frames <= 0:
        raise SystemExit(
            f"--baseline-frames must be positive, got {args.baseline_frames}"
        )

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

    # The transformation matrix is the only head-pose source MediaPipe offers;
    # without it channels 30/31 have nothing to follow.
    options = face_landmarker_options_cls(
        base_options=base_options_cls(model_asset_path=str(args.model)),
        output_face_blendshapes=True,
        output_facial_transformation_matrixes=True,
        running_mode=running_mode.VIDEO,
        num_faces=1,
    )
    landmarker = face_landmarker_cls.create_from_options(options)

    read_frame, (cam_width, cam_height), close_camera = open_frame_source(args, cv2)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    filter_bank = FilterBank(
        min_cutoff=args.min_cutoff, beta=args.beta, d_cutoff=args.d_cutoff
    )

    neck_baseline = HeadPoseBaseline(NECK_BASELINE_FRAMES, args.neck_baseline)
    if not args.no_neck:
        if args.neck_baseline is None:
            print(
                f"neck: averaging the first {NECK_BASELINE_FRAMES} frames with a "
                f"face for the level baseline -- hold still and level"
            )
        else:
            pitch, roll = args.neck_baseline
            print(f"neck: baseline fixed at pitch {pitch:+.2f} roll {roll:+.2f} deg")

    blendshape_baseline: BlendshapeBaseline | None = None
    if not args.no_blendshape_baseline:
        loaded = (
            load_blendshape_baseline(args.baseline_load)
            if args.baseline_load is not None
            else None
        )
        blendshape_baseline = BlendshapeBaseline(args.baseline_frames, loaded)
        if loaded is not None:
            print(f"blendshapes: rest baseline loaded from {args.baseline_load}")
        else:
            print(
                f"blendshapes: averaging the first {args.baseline_frames} frames "
                "with a face for the rest baseline -- hold a neutral expression"
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

            calibrating: str | None = None
            if blendshape_baseline is not None and scores:
                had_baseline = blendshape_baseline.value is not None
                baseline = blendshape_baseline.observe(scores)
                if baseline is None:
                    done, total = blendshape_baseline.progress
                    calibrating = f"hold neutral -- rest baseline {done}/{total}"
                    print(f"\r{calibrating}", end="", flush=True)
                    # No trustworthy zero yet, so rest every mapped channel at
                    # its calibrated neutral -- same rule as the neck below.
                    scores = {}
                else:
                    if not had_baseline:
                        peaks = ", ".join(
                            f"{name} {value:.3f}"
                            for name, value in blendshape_baseline.top_entries()
                        )
                        print(
                            f"\nblendshapes: rest baseline captured; highest: {peaks}"
                        )
                        if args.baseline_save is not None:
                            args.baseline_save.write_text(
                                json.dumps(baseline, indent=2, sort_keys=True) + "\n",
                                encoding="utf-8",
                            )
                            print(
                                f"blendshapes: baseline written to {args.baseline_save}"
                            )
                    scores = blendshape_baseline.rebase(scores)

            raw_coefficients = apply_blendshape_map(scores)

            if not args.no_neck:
                # No face, or no baseline yet, means no trustworthy zero, so
                # rest at neutral rather than driving off an unknown reference --
                # which is also what the blendshape channels do on empty scores.
                neck = NECK_REST
                if result.facial_transformation_matrixes:
                    pitch, _yaw, roll = head_pose_degrees(
                        result.facial_transformation_matrixes[0], cv2
                    )
                    had_baseline = neck_baseline.value is not None
                    baseline = neck_baseline.observe(pitch, roll)
                    if baseline is not None:
                        if not had_baseline:
                            print(
                                f"neck: baseline captured at pitch "
                                f"{baseline[0]:+.2f} roll {baseline[1]:+.2f} deg"
                            )
                        neck = neck_coefficients(
                            pitch - baseline[0],
                            roll - baseline[1],
                            args.neck_pitch_range,
                            args.neck_roll_range,
                        )
                left, right = NECK_CHANNEL_IDS
                raw_coefficients[left], raw_coefficients[right] = neck

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
                    preview_surface,
                    frame_bgr,
                    landmarks,
                    smoothed,
                    neutral_reference,
                    status=calibrating,
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
