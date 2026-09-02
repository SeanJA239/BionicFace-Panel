"""Standalone 2D face visualizer: a drop-in replacement for the Raspberry Pi
executor for development without hardware.

Listens on a UDP port for the exact wire protocol the Rust ControlService
sends (`{"frameId", "timestampNs", "source", "angles"}`, 32 floats), and
renders a layered line-art face (hardware review 2026-08-29, see the photo in
docs/hardware/face_frontal.jpg): a static skull with fixed UPPER teeth, eyes
whose gaze channels move the pupils (not dots), a rigid jaw carrying the LOWER
teeth, and an independent lip ring drawn in front of the teeth -- so "lips
parted over a closed jaw" shows closed teeth behind open lips instead of
looking like an open jaw. Anchor dots are drawn only for channels that are
real surface actuation points (DOT_CHANNELS); their rest positions come from
docs/hardware/face_anchors.json once the jog sweep fills channel_mapping, and
fall back to the hand-written constants until then.

Channel placement and grouping follow the CURRENT channel table in
raspi/config.py / motor_config.json (the old topology.ts ids 8-13 and 24-27
meant different things and are NOT reused). Direction conventions --
which end of a channel's range reads as "brow raised", "lid closed",
"corner up" -- follow tools/mediapipe_driver.py's documented assumptions
and are still unconfirmed except for the mirroring below.

Usage:
    python3 tools/face_visualizer.py [--host 0.0.0.0] [--port 6000] [--config PATH]

Keys: N toggles the anchor dots / channel ids overlay, ESC quits.

Per-channel limits (min/max/neutral applied degrees) are read from
`src-tauri/config/motor_config.json` so this script never carries a second,
divergent copy of MOTOR_LIMITS. Run `python3 raspi/export_config_json.py`
first if that file doesn't exist yet.

Frames arriving over UDP are already rate-limited/interpolated by the Rust
side, so this script renders each received frame directly with no local
smoothing -- it just holds the last frame's angles until the next one
arrives.

Note: screen displacement uses a bipolar deviation around each channel's
neutral (-1 at minApplied, 0 at neutral, +1 at maxApplied), computed locally
from the wire-protocol applied angles plus the config limits. This mirrors
control.rs's norm-space *semantics* for display purposes only -- the wire
protocol still carries plain applied degrees.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import socket
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import pygame
from face_profile import INSET_HEIGHT, INSET_WIDTH, profile_geometry

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = ROOT / "src-tauri" / "config" / "motor_config.json"
MOTOR_COUNT = 32

SCREEN_SIZE = (900, 720)
FACE_CENTER = (450, 340)
SIGNAL_LOST_TIMEOUT_S = 1.0

BACKGROUND = (13, 18, 28)
LINE_COLOR = (215, 226, 240)
OUTLINE_COLOR = (150, 164, 186)
EYE_WHITE = (238, 242, 248)
PUPIL_COLOR = (18, 22, 30)
LIP_COLOR = (216, 120, 126)
MOUTH_INNER = (36, 18, 24)
TOOTH_COLOR = (224, 226, 219)
TOOTH_EDGE = (148, 152, 148)
DISABLED_NODE_COLOR = (105, 112, 124)
HUD_COLOR = (200, 220, 230)
LOST_COLOR = (230, 60, 60)

# Anchor dot palette per channel group, carried over from the original
# frontend topology view.
# The subject's-right servos are mounted mirrored relative to the left ones, so
# a symmetric expression arrives with opposite norm signs on the two sides.
# Read off the hardware-authored emotion presets: across the five that should be
# left/right symmetric (喜悦/悲伤/愤怒/惊讶/恐惧 -- 困惑 and wink are deliberately
# asymmetric), paired channels carry opposite signs in 26 of 29 cases where both
# sides move. Channel 11 is independently confirmed by control.rs's idle-blink
# direction table, and no pair has its mirroring already encoded in
# minApplied/maxApplied -- every range is increasing.
#
# Mirroring is a property of the whole right-side bank, not of individual
# channels, so this set starts from every right-side channel that has a left-side
# mirror partner. Deciding it per channel by voting on preset signs is what
# produced an earlier half-corrected set (0, 1, 11, 17, 18) that rendered
# symmetric presets with the brows and mouth corners fixed while the cheeks,
# nose, lower lids and lips stayed flipped.
#
# Channel 22 (lower_lip_right) is the one right-side partner deliberately left
# OUT. 悲伤 and 愤怒 share an identical mouth block for channels 17-22, so they are
# one authored observation, not two -- and once deduplicated the lower-lip pair is
# a 1-1 tie. 恐惧 breaks it: 22=+1.00 with 21=+0.74, both large and same-signed,
# and fear pulls the lower lip down on both sides, so negating 22 renders that
# preset with the two halves of the lower lip moving apart. Every other pair here
# is unanimous.
#
# Deliberately excluded, for structural reasons rather than lack of evidence:
# midline channels with no mirror partner (8, 13, 15, 23, 24); the jaw, whose
# open axis enters through abs() and whose 26/27 pair is coupled in Rust; and the
# neck, which enters as the difference dev(30) - dev(31), a form that already
# handles a mirrored pair (tilt is antisymmetric either way, so the presets
# cannot tell us about neck mounting).
MIRRORED_CHANNELS = frozenset({0, 1, 6, 7, 11, 12, 16, 17, 18})

# Canonical subject-right <-> subject-left pairing of the paired facial features.
# The mirrored set above is a subset of its right-hand members; the render scale
# below shares one reference across each pair.
MIRROR_PAIRS: tuple[tuple[int, int], ...] = (
    (0, 2),
    (1, 3),
    (6, 5),
    (7, 4),
    (11, 9),
    (12, 10),
    (16, 14),
    (17, 19),
    (18, 20),
    (22, 21),
)


def render_scales(channels: list[Channel]) -> list[float]:
    """Degrees of travel each channel's drawn displacement is divided by.

    control.rs normalises each side of neutral by that side's own travel. Drawing
    in those units makes a channel with 5 deg of travel produce the same excursion
    as one with 50, so a physically symmetric pose renders lopsided. 悲伤 is the clear case: channel 18 at norm -1.00 and channel 20 at
    norm +0.50 are both 20 deg of real motion, and once 18's mirrored mount is
    accounted for they are the same face motion -- yet they differ 2x in norm,
    because 18 has 20 deg of upward travel where 20 has 40. The corners came out
    at +0.10 and -0.25, which is the slant in the rendered mouth.

    Dividing by degrees against a reference shared across the mirror pair makes
    equal physical motion draw equal displacement. Unpaired channels (midline,
    jaw, neck) keep their own largest travel, so their amplitude is unchanged.
    """
    own = [
        max(c.neutral_applied - c.min_applied, c.max_applied - c.neutral_applied)
        for c in channels
    ]
    scales = list(own)
    for right, left in MIRROR_PAIRS:
        shared = max(own[right], own[left])
        scales[right] = scales[left] = shared
    return [s if s > 1e-6 else 1.0 for s in scales]


ANCHORS_PATH = ROOT / "docs" / "hardware" / "face_anchors.json"

# Channels that are actual surface actuation points and therefore get an anchor
# dot: brow/cheek/nose tendons, eyelids, and the lip ring. Everything else has
# a dedicated representation instead -- 8/13 move the pupils, 24/25 move the
# rigid jaw, 26/27 are depth-axis (HUD numbers only, a frontal 2D view cannot
# show them), 28/29 are disabled tongue channels, 30/31 tilt the whole head.
DOT_CHANNELS = frozenset(range(8)) | frozenset(range(9, 13)) | frozenset(range(14, 24))

# Teeth geometry in face-local units. The upper row is part of the skull
# (fixed); the lower row rides the jaw. At rest the rows meet at OCCLUSION_Y;
# jaw open drops the lower band, exposing the dark mouth interior between them.
TEETH_HALF_WIDTH = 78.0
UPPER_TEETH_TOP = 46.0
OCCLUSION_Y = 60.0
LOWER_TEETH_BOTTOM = 74.0
TOOTH_PITCH = 13.0
TEETH_SAMPLE_STEP = 4.0


GROUP_COLORS = {
    "brow": (249, 115, 22),
    "tendon": (239, 68, 68),
    "eye": (34, 197, 94),
    "mouth": (56, 189, 248),
    "jaw": (167, 139, 250),
    "neck": (250, 204, 21),
}

CHANNEL_GROUPS: dict[int, str] = (
    {i: "brow" for i in range(4)}
    | {i: "tendon" for i in range(4, 8)}
    | {i: "eye" for i in range(8, 14)}
    | {i: "mouth" for i in range(14, 24)}
    | {i: "jaw" for i in range(24, 30)}
    | {i: "neck" for i in range(30, 32)}
)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass
class Channel:
    name: str
    min_applied: float
    max_applied: float
    neutral_applied: float
    enabled: bool


def load_channels(config_path: Path) -> list[Channel]:
    raw = json.loads(config_path.read_text(encoding="utf-8"))
    slots: list[Channel | None] = [None] * MOTOR_COUNT
    for entry in raw["channels"]:
        slots[entry["id"]] = Channel(
            name=entry["name"],
            min_applied=float(entry["minApplied"]),
            max_applied=float(entry["maxApplied"]),
            neutral_applied=float(entry["neutralApplied"]),
            enabled=bool(entry["enabled"]),
        )
    if any(slot is None for slot in slots):
        raise RuntimeError(f"{config_path} is missing one or more channel ids")
    return slots  # type: ignore[return-value]


@dataclass
class LatestFrame:
    angles: list[float]
    frame_id: int | None = None
    source: str | None = None
    received_at: float = field(default_factory=time.monotonic)


class FrameReceiver:
    """Non-blocking UDP receiver that keeps only the newest valid frame,
    mirroring raspi/servo_server.py's drain-to-latest behavior."""

    def __init__(self, host: str, port: int, neutral_angles: list[float]) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.socket.bind((host, port))
        self.latest = LatestFrame(angles=list(neutral_angles), received_at=0.0)

    def poll(self) -> None:
        newest_payload: dict[str, Any] | None = None
        while True:
            try:
                packet, _addr = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                payload = json.loads(packet.decode("utf-8"))
                if len(payload.get("angles", [])) == MOTOR_COUNT:
                    newest_payload = payload
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue

        if newest_payload is not None:
            self.latest = LatestFrame(
                angles=[float(a) for a in newest_payload["angles"]],
                frame_id=newest_payload.get("frameId"),
                source=newest_payload.get("source"),
                received_at=time.monotonic(),
            )

    def seconds_since_last_frame(self) -> float:
        if self.latest.received_at == 0.0:
            return math.inf
        return time.monotonic() - self.latest.received_at


def rotate(x: float, y: float, angle_rad: float) -> tuple[float, float]:
    cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
    return x * cos_a - y * sin_a, x * sin_a + y * cos_a


Point = tuple[float, float]
Transform = Callable[[float, float], tuple[int, int]]


def catmull_rom(points: list[Point], samples: int = 12) -> list[Point]:
    """Interpolates a smooth curve through all control points (endpoints
    included) so the face lines bend, not kink, around a displaced anchor."""
    if len(points) < 3:
        return list(points)
    pts = [points[0], *points, points[-1]]
    out: list[Point] = []
    for i in range(len(pts) - 3):
        p0, p1, p2, p3 = pts[i : i + 4]
        for j in range(samples):
            t = j / samples
            t2, t3 = t * t, t * t * t
            out.append(
                (
                    0.5
                    * (
                        2 * p1[0]
                        + (-p0[0] + p2[0]) * t
                        + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2
                        + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3
                    ),
                    0.5
                    * (
                        2 * p1[1]
                        + (-p0[1] + p2[1]) * t
                        + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2
                        + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3
                    ),
                )
            )
    out.append(points[-1])
    return out


def load_photo_anchors(
    path: Path, eye_left_local: Point, eye_right_local: Point
) -> dict[int, Point]:
    """Face-local rest positions measured on the hardware photo, per channel.

    docs/hardware/face_anchors.json carries the dots marked on the skinless
    frontal photo (normalized image coordinates) plus a channel_mapping filled
    in by the jog sweep. Photo coordinates map into face-local space through a
    similarity transform anchored on the two pupils. The photo and the twin
    face are both observer-view (+x = screen right = subject's left), so no
    mirror flip is needed; the photo is assumed upright, so no roll term.

    Returns {} when the file or its channel_mapping is absent (the mapping
    ships as a TODO string until the sweep fills it), keeping every channel on
    the hand-written fallback constants unchanged.
    """
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    mapping = raw.get("channel_mapping")
    candidates = raw.get("red_candidates")
    pupils = raw.get("pupils")
    if not isinstance(mapping, dict) or not candidates or not pupils:
        return {}

    width, height = raw["image_size"]
    pts = [(p[0] * width, p[1] * height) for p in pupils]
    if len(pts) < 2:
        return {}
    # The dot detector can pick up stray blue marks; the true pupils are the
    # pair sitting at nearly the same height.
    pair = min(
        ((a, b) for i, a in enumerate(pts) for b in pts[i + 1 :]),
        key=lambda ab: abs(ab[0][1] - ab[1][1]),
    )
    photo_left, photo_right = sorted(pair, key=lambda pt: pt[0])
    span = math.hypot(photo_right[0] - photo_left[0], photo_right[1] - photo_left[1])
    if span < 1e-6:
        return {}
    scale = (eye_right_local[0] - eye_left_local[0]) / span
    mid_photo = (
        (photo_left[0] + photo_right[0]) / 2.0,
        (photo_left[1] + photo_right[1]) / 2.0,
    )
    mid_local = (
        (eye_left_local[0] + eye_right_local[0]) / 2.0,
        (eye_left_local[1] + eye_right_local[1]) / 2.0,
    )

    anchors: dict[int, Point] = {}
    for channel_str, candidate in mapping.items():
        pos = candidates.get(str(candidate))
        if pos is None:
            continue
        px, py = pos[0] * width, pos[1] * height
        anchors[int(channel_str)] = (
            mid_local[0] + (px - mid_photo[0]) * scale,
            mid_local[1] + (py - mid_photo[1]) * scale,
        )
    return anchors


def curve_y_at(points: list[Point], x: float) -> float | None:
    """Linear-interpolated y of a sampled curve at x; None outside its span.

    The lip curves run right corner (-x) to left corner (+x); Catmull-Rom can
    wiggle near the ends, so the first crossing wins.
    """
    for (x0, y0), (x1, y1) in itertools.pairwise(points):
        if (x0 - x) * (x1 - x) <= 0.0 and abs(x1 - x0) > 1e-9:
            return y0 + (x - x0) / (x1 - x0) * (y1 - y0)
    return None


def teeth_strips(
    band_top: float,
    band_bottom: float,
    upper_lip: list[Point],
    lower_lip: list[Point],
    x_shift: float = 0.0,
) -> tuple[list[list[Point]], list[tuple[Point, Point]]]:
    """Visible tooth-band polygons, clipped per x-sample to the lip opening.

    Clipping is geometric rather than raster (pygame clips only to rects and
    SVG would need clipPath defs, and this way both renderers share one
    algorithm): at each sample x the band is intersected with the
    [upper lip, lower lip] interval, and contiguous samples become filled
    strips. `x_shift` rides the lower band on the jaw's lateral shift.
    Returns (strips, tooth-separator segments), all in face-local coords.
    """
    strips: list[list[Point]] = []
    separators: list[tuple[Point, Point]] = []
    top_run: list[Point] = []
    bottom_run: list[Point] = []

    def visible_interval(fx: float) -> tuple[float, float] | None:
        yu = curve_y_at(upper_lip, fx)
        yl = curve_y_at(lower_lip, fx)
        if yu is None or yl is None:
            return None
        top, bottom = max(band_top, yu), min(band_bottom, yl)
        return (top, bottom) if top < bottom - 0.5 else None

    def close_run() -> None:
        if len(top_run) >= 2:
            strips.append(top_run[:] + bottom_run[::-1])
        top_run.clear()
        bottom_run.clear()

    steps = int(2 * TEETH_HALF_WIDTH / TEETH_SAMPLE_STEP)
    for i in range(steps + 1):
        fx = -TEETH_HALF_WIDTH + i * TEETH_SAMPLE_STEP + x_shift
        interval = visible_interval(fx)
        if interval is None:
            close_run()
        else:
            top_run.append((fx, interval[0]))
            bottom_run.append((fx, interval[1]))
    close_run()

    boundaries = int(2 * TEETH_HALF_WIDTH / TOOTH_PITCH)
    for k in range(1, boundaries + 1):
        fx = -TEETH_HALF_WIDTH + k * TOOTH_PITCH + x_shift
        interval = visible_interval(fx)
        if interval is not None:
            separators.append(((fx, interval[0]), (fx, interval[1])))
    return strips, separators


class FaceRenderer:
    """Layered line-art face in face-local coordinates: origin at FACE_CENTER,
    +x = screen right = subject's LEFT (channel names are subject-relative,
    mirrored like the original topology view), +y = down.

    Layers back-to-front: static skull (outline, nose, UPPER teeth), eyes
    (pupils carry gaze channels 8/13), rigid jaw (LOWER teeth; open from 25,
    lateral from 24; 26/27 are depth-axis and only appear as HUD numbers),
    the lip ring (14-23) drawn in front of the teeth, and the neck tilt.

    Direction conventions per group (assumed, see module docstring):
    +deviation = brow raised, lid closed, upper corner up / lower corner
    down, upper lip up / lower lip down, tendon pulled up. Channel 25's
    open direction is unconfirmed, so jaw open uses |deviation|.
    """

    MAX_TILT_DEG = 12.0
    JAW_OPEN_RANGE_PX = 60.0
    EYE_GAZE_RANGE_PX = 12.0
    JAW_SHIFT_RANGE_PX = 14.0
    CORNER_VERTICAL_RANGE_PX = 22.0
    CORNER_HORIZONTAL_RANGE_PX = 18.0

    EYE_HALF_WIDTH = 34.0
    EYE_APERTURE_UPPER = 15.0
    EYE_APERTURE_LOWER = 11.0
    EYE_CENTER_X = 75.0
    EYE_CENTER_Y = -120.0

    def __init__(self, channels: list[Channel]) -> None:
        self.channels = channels
        self.render_scales = render_scales(channels)
        self.photo_anchors = load_photo_anchors(
            ANCHORS_PATH,
            (-self.EYE_CENTER_X, self.EYE_CENTER_Y),
            (self.EYE_CENTER_X, self.EYE_CENTER_Y),
        )
        self.show_nodes = True
        pygame.font.init()
        self.font = pygame.font.SysFont("monospace", 18)
        self.font_small = pygame.font.SysFont("monospace", 13, bold=True)
        self.font_big = pygame.font.SysFont("monospace", 36, bold=True)

    def rest(self, channel_id: int, default: Point) -> Point:
        """Rest position of a channel's feature point: the photo-measured
        anchor when the jog sweep has mapped it, the hand-written constant
        otherwise. Motion offsets are applied on top either way."""
        return self.photo_anchors.get(channel_id, default)

    def dev(self, channel_id: int, angles: list[float]) -> float:
        """Displacement from neutral in units of the channel's render scale,
        with the mirrored channels already flipped.

        Scaled by degrees rather than by Channel.deviation's per-side norm, so
        that paired channels with unequal travel draw equal physical motion as
        equal displacement -- see render_scales. Negating here, at the one place
        displacements enter the renderer, lets every part below keep reading
        "+ = brow raised / lid closed / corner up" regardless of which way that
        channel's servo is mounted.
        """
        channel = self.channels[channel_id]
        value = clamp(
            (angles[channel_id] - channel.neutral_applied)
            / self.render_scales[channel_id],
            -1.0,
            1.0,
        )
        return -value if channel_id in MIRRORED_CHANNELS else value

    def tilt_radians(self, angles: list[float]) -> float:
        delta = self.dev(30, angles) - self.dev(31, angles)
        return math.radians(
            clamp(delta * self.MAX_TILT_DEG, -self.MAX_TILT_DEG, self.MAX_TILT_DEG)
        )

    def jaw_open_px(self, angles: list[float]) -> float:
        # Channel 25 (jaw_right_upper, KS3518) is the primary jaw-open axis;
        # which direction is "open" on the real hardware is unconfirmed, so
        # distance from neutral in either direction counts as open. TODO:
        # hardware calibration may reveal this should be one-sided.
        return abs(self.dev(25, angles)) * self.JAW_OPEN_RANGE_PX

    def depth_readout(self, angles: list[float]) -> str:
        """Jaw numbers for the HUD. 26/27 move the jaw along the depth axis,
        which a frontal 2D view cannot show -- deliberately not drawn as a
        (fake) shape change; the numbers are the whole representation."""
        return (
            f"jaw open {abs(self.dev(25, angles)):.2f}"
            f"  shift {self.dev(24, angles):+.2f}"
            f"  depth 26:{self.dev(26, angles):+.2f} 27:{self.dev(27, angles):+.2f}"
        )

    def draw(self, surface: pygame.Surface, angles: list[float]) -> None:
        tilt = self.tilt_radians(angles)
        jaw_open = self.jaw_open_px(angles)
        jaw_x = self.dev(24, angles) * self.JAW_SHIFT_RANGE_PX

        def to_screen(x: float, y: float) -> tuple[int, int]:
            rx, ry = rotate(x, y, tilt)
            return int(FACE_CENTER[0] + rx), int(FACE_CENTER[1] + ry)

        def fixed(x: float, y: float) -> tuple[int, int]:
            return int(FACE_CENTER[0] + x), int(FACE_CENTER[1] + y)

        # Screen positions of every channel's anchor dot, filled in by the
        # feature that renders it so dots always sit on the deformed lines.
        nodes: dict[int, tuple[int, int]] = {}

        self._draw_neck(surface, angles, fixed, nodes)
        self._draw_head(surface, angles, jaw_open, jaw_x, to_screen, nodes)
        self._draw_brows(surface, angles, to_screen, nodes)
        self._draw_eyes(surface, angles, to_screen, nodes)
        self._draw_nose(surface, angles, to_screen, nodes)
        self._draw_cheeks(surface, angles, to_screen, nodes)
        self._draw_mouth(surface, angles, jaw_open, jaw_x, to_screen, nodes)
        self._draw_profile_inset(surface, angles)

        if self.show_nodes:
            self._draw_nodes(surface, nodes)

    def _curve(
        self, surface, points: list[Point], to_screen: Transform, color, width
    ) -> None:
        pts = [to_screen(x, y) for x, y in catmull_rom(points)]
        pygame.draw.lines(surface, color, False, pts, width)

    def _draw_neck(self, surface, angles, fixed: Transform, nodes) -> None:
        # The neck pair stays in screen space (the head tilts on top of it).
        # 30/31 get no dots: their whole representation is the head tilt.
        del angles, nodes
        for side in (1, -1):
            pygame.draw.line(
                surface, OUTLINE_COLOR, fixed(side * 72, 200), fixed(side * 95, 292), 4
            )

    def _draw_head(
        self, surface, angles, jaw_open, jaw_x, to_screen: Transform, nodes
    ) -> None:
        # Upper head: elliptical arc (a=175, b=235 around (0,-10)) ending at
        # (±160, 80), where the jaw outline takes over.
        arc: list[Point] = []
        t0, t1 = 0.393, -(math.pi + 0.393)
        for i in range(49):
            t = t0 + (t1 - t0) * i / 48
            arc.append((175.0 * math.cos(t), -10.0 + 235.0 * math.sin(t)))
        pygame.draw.lines(
            surface, OUTLINE_COLOR, False, [to_screen(x, y) for x, y in arc], 3
        )

        # Jaw outline moves as a rigid body: open (25) drops it, 24 shears it
        # sideways. 26/27 are depth-axis and deliberately absent here (see
        # depth_readout); the jaw group gets no anchor dots -- the outline and
        # the lower teeth ARE its representation.
        del angles, nodes
        chin = [
            (-160.0, 80.0),
            (-95.0 + jaw_x, 170.0 + 0.7 * jaw_open),
            (jaw_x, 195.0 + jaw_open),
            (95.0 + jaw_x, 170.0 + 0.7 * jaw_open),
            (160.0, 80.0),
        ]
        self._curve(surface, chin, to_screen, OUTLINE_COLOR, 3)

    def _draw_brows(self, surface, angles, to_screen: Transform, nodes) -> None:
        # Subject-right brow (0/1) on screen-left. Outer ends sit higher,
        # matching the original topology layout.
        for side, inner_ch, outer_ch in ((-1, 0, 1), (1, 2, 3)):
            rix, riy = self.rest(inner_ch, (side * 70.0, -168.0))
            rox, roy = self.rest(outer_ch, (side * 130.0, -180.0))
            ix, iy = rix, riy - 26.0 * self.dev(inner_ch, angles)
            ox, oy = rox, roy - 22.0 * self.dev(outer_ch, angles)
            mid = ((ix + ox) / 2.0, (iy + oy) / 2.0 - 5.0)
            self._curve(surface, [(ix, iy), mid, (ox, oy)], to_screen, LINE_COLOR, 5)
            nodes[inner_ch] = to_screen(ix, iy)
            nodes[outer_ch] = to_screen(ox, oy)

    def _draw_eyes(self, surface, angles, to_screen: Transform, nodes) -> None:
        gaze_x = self.dev(8, angles) * self.EYE_GAZE_RANGE_PX
        gaze_y = -self.dev(13, angles) * self.EYE_GAZE_RANGE_PX  # + = look up

        # side=-1 -> screen-left eye = subject's RIGHT (channels 11/12).
        for side, upper_ch, lower_ch in ((-1, 11, 12), (1, 9, 10)):
            cx, cy = side * self.EYE_CENTER_X, self.EYE_CENTER_Y
            hw = self.EYE_HALF_WIDTH
            ap_u = clamp(
                self.EYE_APERTURE_UPPER * (1.0 - self.dev(upper_ch, angles)), 0.0, 24.0
            )
            ap_l = clamp(
                self.EYE_APERTURE_LOWER * (1.0 - self.dev(lower_ch, angles)), 0.0, 18.0
            )

            upper_edge: list[Point] = []
            lower_edge: list[Point] = []
            for i in range(17):
                s = -1.0 + i / 8.0
                bulge = 1.0 - s * s
                upper_edge.append((cx + s * hw, cy - ap_u * bulge))
                lower_edge.append((cx + s * hw, cy + ap_l * bulge))

            if ap_u + ap_l > 4.0:
                aperture = [to_screen(x, y) for x, y in upper_edge + lower_edge[::-1]]
                pygame.draw.polygon(surface, EYE_WHITE, aperture)
                px = cx + clamp(gaze_x, -(hw - 12.0), hw - 12.0)
                py = cy + clamp(gaze_y, -ap_u * 0.5, ap_l * 0.5)
                radius = min(10.0, (ap_u + ap_l) * 0.45)
                pygame.draw.circle(surface, PUPIL_COLOR, to_screen(px, py), int(radius))
                pygame.draw.polygon(surface, LINE_COLOR, aperture, 2)
            else:
                pygame.draw.line(
                    surface,
                    LINE_COLOR,
                    to_screen(cx - hw, cy),
                    to_screen(cx + hw, cy),
                    3,
                )

            ru = self.rest(upper_ch, (cx, cy - self.EYE_APERTURE_UPPER))
            rl = self.rest(lower_ch, (cx, cy + self.EYE_APERTURE_LOWER))
            nodes[upper_ch] = to_screen(ru[0], ru[1] + (self.EYE_APERTURE_UPPER - ap_u))
            nodes[lower_ch] = to_screen(rl[0], rl[1] - (self.EYE_APERTURE_LOWER - ap_l))

        # 8/13 get no dots: they are rotations of the shared eye mechanism, not
        # surface points -- the pupils are their whole representation.

    def _draw_nose(self, surface, angles, to_screen: Transform, nodes) -> None:
        pygame.draw.line(
            surface, OUTLINE_COLOR, to_screen(0, -104), to_screen(0, -46), 2
        )
        # Nose tendons scrunch their side of the nose base upward.
        lift_l = -8.0 * self.dev(5, angles)  # subject left = +x
        lift_r = -8.0 * self.dev(6, angles)
        base = [(-24.0, -40.0 + lift_r), (0.0, -30.0), (24.0, -40.0 + lift_l)]
        self._curve(surface, base, to_screen, LINE_COLOR, 3)
        r5 = self.rest(5, (92.0, -80.0))
        r6 = self.rest(6, (-92.0, -80.0))
        nodes[5] = to_screen(r5[0], r5[1] - 14.0 * self.dev(5, angles))
        nodes[6] = to_screen(r6[0], r6[1] - 14.0 * self.dev(6, angles))

    def _draw_cheeks(self, surface, angles, to_screen: Transform, nodes) -> None:
        # Cheek tendon pulls its arc up and outward (smile apple / squint).
        for side, ch in ((1, 4), (-1, 7)):
            d = self.dev(ch, angles)
            rx, ry = self.rest(ch, (side * 150.0, -45.0))
            ax, ay = rx + side * 6.0 * d, ry - 18.0 * d
            self._curve(
                surface,
                [(side * 118.0, -95.0), (ax, ay), (side * 138.0, 28.0)],
                to_screen,
                OUTLINE_COLOR,
                3,
            )
            nodes[ch] = to_screen(ax, ay)

    def _draw_mouth(
        self, surface, angles, jaw_open, jaw_x, to_screen: Transform, nodes
    ) -> None:
        # The lip ring is its own layer, but it still rides the jaw a little:
        # with skin on, the lower lip sits on the jaw. The teeth do not follow
        # the lip channels at all -- that separation is what makes "lips parted
        # over a closed jaw" (closed teeth behind open lips) distinguishable
        # from an open jaw (a dark gap between the tooth rows).
        up_lift = -0.12 * jaw_open  # upper lip eases up slightly as jaw drops
        low_drop = 0.85 * jaw_open

        def pt(
            ch: int,
            x: float,
            y: float,
            move_y: float,
            shift_x: float = 0.0,
            follow_y: float = 0.0,
        ) -> Point:
            rx, ry = self.rest(ch, (x, y))
            p = (rx + shift_x, ry + follow_y + move_y * self.dev(ch, angles))
            nodes[ch] = to_screen(*p)
            return p

        # Each mouth corner is ONE physical point driven by two motors through a
        # linkage (only the motor pivots move; the corner is a coupler point),
        # and it is where the upper and lower lip curves have to meet. Drawing
        # 17/18 as two separate endpoints splayed the outline open at the
        # corners.
        #
        # The two channels' documented positive directions are geometrically
        # opposite -- +upper raises the corner, +lower pulls the lip down -- so
        # their DIFFERENCE is the up/down common mode and their SUM the in/out
        # differential. Checked against the presets: the difference puts 喜悦's
        # corners up and 悲伤/愤怒/恐惧's down, left and right within 0.05 of each
        # other; the sum gets every one of those backwards. The horizontal
        # direction sign is still an assumption, unconfirmed on hardware --
        # same caveat as jaw_open's direction.
        def corner(upper_ch: int, lower_ch: int, side: float) -> Point:
            up = self.dev(upper_ch, angles)
            low = self.dev(lower_ch, angles)
            lift = (up - low) / 2
            outward = (up + low) / 2
            # The pair shares one physical corner, so either channel's mapped
            # anchor (upper wins) positions the rest point.
            rx, ry = self.rest(upper_ch, self.rest(lower_ch, (side * 106.0, 56.0)))
            p = (
                rx + side * outward * self.CORNER_HORIZONTAL_RANGE_PX,
                ry + (up_lift + low_drop) / 2 - lift * self.CORNER_VERTICAL_RANGE_PX,
            )
            nodes[upper_ch] = to_screen(*p)
            nodes[lower_ch] = to_screen(*p)
            return p

        right_corner = corner(17, 18, -1.0)
        left_corner = corner(19, 20, 1.0)

        # Lips rest nearly closed (thin lens); jaw open and the individual
        # lip channels separate them. + = lip up on the upper, pulled down on
        # the lower, which also follows the jaw sideways (channel 24).
        upper = [
            right_corner,
            pt(16, -52.0, 46.0, -14.0, follow_y=up_lift),
            pt(15, 0.0, 44.0, -12.0, follow_y=up_lift),
            pt(14, 52.0, 46.0, -14.0, follow_y=up_lift),
            left_corner,
        ]
        lower = [
            right_corner,
            pt(22, -52.0, 58.0, 16.0, jaw_x, follow_y=low_drop),
            pt(23, 0.0, 62.0, 18.0, jaw_x, follow_y=low_drop),
            pt(21, 52.0, 58.0, 16.0, jaw_x, follow_y=low_drop),
            left_corner,
        ]

        upper_pts = catmull_rom(upper)
        lower_pts = catmull_rom(lower)
        interior = [to_screen(x, y) for x, y in upper_pts + lower_pts[::-1]]
        pygame.draw.polygon(surface, MOUTH_INNER, interior)

        # Teeth, clipped to the lip opening. Upper row is skull-fixed; the
        # lower row rides the rigid jaw (full open drop + lateral shift).
        upper_strips, upper_seps = teeth_strips(
            UPPER_TEETH_TOP, OCCLUSION_Y, upper_pts, lower_pts
        )
        lower_strips, lower_seps = teeth_strips(
            OCCLUSION_Y + jaw_open,
            LOWER_TEETH_BOTTOM + jaw_open,
            upper_pts,
            lower_pts,
            x_shift=jaw_x,
        )
        for strip in upper_strips + lower_strips:
            screen_strip = [to_screen(x, y) for x, y in strip]
            pygame.draw.polygon(surface, TOOTH_COLOR, screen_strip)
            pygame.draw.polygon(surface, TOOTH_EDGE, screen_strip, 1)
        for (x0, y0), (x1, y1) in upper_seps + lower_seps:
            pygame.draw.line(
                surface, TOOTH_EDGE, to_screen(x0, y0), to_screen(x1, y1), 1
            )

        self._curve(surface, upper, to_screen, LIP_COLOR, 4)
        self._curve(surface, lower, to_screen, LIP_COLOR, 4)

    # Inset corner: top-left holds FPS, bottom-left the last-frame line and
    # the depth readout, bottom-right the SIGNAL LOST banner -- top-right is
    # the only quiet corner, below the frame/src line (y < 32).
    PROFILE_ORIGIN = (SCREEN_SIZE[0] - int(INSET_WIDTH) - 12, 36)

    def _draw_profile_inset(self, surface, angles) -> None:
        """Side view for the depth-axis motions; see face_profile.py."""
        geometry = profile_geometry(
            abs(self.dev(25, angles)),
            self.dev(26, angles),
            (self.dev(30, angles) + self.dev(31, angles)) / 2.0,
            self.PROFILE_ORIGIN,
        )
        ox, oy = self.PROFILE_ORIGIN
        pygame.draw.rect(
            surface,
            DISABLED_NODE_COLOR,
            pygame.Rect(ox, oy, int(INSET_WIDTH), int(INSET_HEIGHT)),
            1,
        )
        for a, b in geometry["neck"]:
            pygame.draw.line(surface, OUTLINE_COLOR, a, b, 2)
        pygame.draw.aalines(surface, OUTLINE_COLOR, False, geometry["skull"])
        pygame.draw.aalines(surface, OUTLINE_COLOR, False, geometry["jaw"])
        for key in ("upper_teeth", "lower_teeth"):
            pygame.draw.polygon(surface, TOOTH_COLOR, geometry[key])
            pygame.draw.polygon(surface, TOOTH_EDGE, geometry[key], 1)
        for a, b in geometry["tooth_separators"]:
            pygame.draw.line(surface, TOOTH_EDGE, a, b, 1)
        label = self.font_small.render("SIDE", True, HUD_COLOR)
        surface.blit(label, (ox + 5, oy + 3))

    def _draw_nodes(self, surface, nodes: dict[int, tuple[int, int]]) -> None:
        for channel_id, (x, y) in nodes.items():
            enabled = self.channels[channel_id].enabled
            color = (
                GROUP_COLORS[CHANNEL_GROUPS[channel_id]]
                if enabled
                else DISABLED_NODE_COLOR
            )
            pygame.draw.circle(surface, color, (x, y), 5)
            label = self.font_small.render(str(channel_id), True, color)
            surface.blit(label, (x + 7, y - 16))


def format_hud_time(seconds_ago: float) -> str:
    if math.isinf(seconds_ago):
        return "never"
    return f"{seconds_ago:.2f}s ago"


def draw_hud(
    surface: pygame.Surface, renderer: FaceRenderer, receiver: FrameReceiver, fps: float
) -> None:
    latest = receiver.latest
    since = receiver.seconds_since_last_frame()

    top_left = renderer.font.render(f"FPS: {fps:5.1f}", True, HUD_COLOR)
    surface.blit(top_left, (12, 8))

    top_right_text = f"frame={latest.frame_id if latest.frame_id is not None else '-'} src={latest.source or '-'}"
    top_right = renderer.font.render(top_right_text, True, HUD_COLOR)
    surface.blit(top_right, (SCREEN_SIZE[0] - top_right.get_width() - 12, 8))

    bottom_left = renderer.font.render(
        f"last frame: {format_hud_time(since)}   [N] anchors", True, HUD_COLOR
    )
    surface.blit(bottom_left, (12, SCREEN_SIZE[1] - 26))

    depth = renderer.font.render(renderer.depth_readout(latest.angles), True, HUD_COLOR)
    surface.blit(depth, (12, SCREEN_SIZE[1] - 50))

    if since > SIGNAL_LOST_TIMEOUT_S:
        lost = renderer.font_big.render("信号丢失 / SIGNAL LOST", True, LOST_COLOR)
        surface.blit(
            lost,
            (
                SCREEN_SIZE[0] - lost.get_width() - 12,
                SCREEN_SIZE[1] - lost.get_height() - 12,
            ),
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Standalone UDP-driven 2D face visualizer"
    )
    parser.add_argument(
        "--host", default="0.0.0.0", help="UDP bind host (default 0.0.0.0)"
    )
    parser.add_argument(
        "--port", type=int, default=6000, help="UDP bind port (default 6000)"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help="Path to motor_config.json (default src-tauri/config/motor_config.json)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    channels = load_channels(args.config)
    neutral_angles = [channel.neutral_applied for channel in channels]

    pygame.init()
    screen = pygame.display.set_mode(SCREEN_SIZE)
    pygame.display.set_caption(
        f"BionicFace Visualizer - listening on {args.host}:{args.port}"
    )
    clock = pygame.time.Clock()

    receiver = FrameReceiver(args.host, args.port, neutral_angles)
    renderer = FaceRenderer(channels)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_n:
                    renderer.show_nodes = not renderer.show_nodes

        receiver.poll()

        screen.fill(BACKGROUND)
        renderer.draw(screen, receiver.latest.angles)
        draw_hud(screen, renderer, receiver, clock.get_fps())
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()
