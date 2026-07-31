"""Standalone 2D face visualizer: a drop-in replacement for the Raspberry Pi
executor for development without hardware.

Listens on a UDP port for the exact wire protocol the Rust ControlService
sends (`{"frameId", "timestampNs", "source", "angles"}`, 32 floats), and
renders a line-art face in the style of the original frontend topology view
(commit b58a660's src/topology.ts): every servo is a colored anchor dot
placed at its anatomical position, and the facial lines (brows, lids, lips,
jaw outline...) are curves drawn through those anchors, so each channel's
motion is visible as a deformation of the face lines around its dot.

Channel placement and grouping follow the CURRENT channel table in
raspi/config.py / motor_config.json (the old topology.ts ids 8-13 and 24-27
meant different things and are NOT reused). Direction conventions --
which end of a channel's range reads as "brow raised", "lid closed",
"corner up" -- follow tools/mediapipe_driver.py's documented assumptions
and are unconfirmed until hardware calibration.

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
import json
import math
import socket
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import pygame

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
DISABLED_NODE_COLOR = (105, 112, 124)
HUD_COLOR = (200, 220, 230)
LOST_COLOR = (230, 60, 60)

# Anchor dot palette per channel group, carried over from the original
# frontend topology view.
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

    def norm01(self, applied: float) -> float:
        span = self.max_applied - self.min_applied
        if span <= 1e-6:
            return 0.5
        return clamp((applied - self.min_applied) / span, 0.0, 1.0)

    def neutral_norm01(self) -> float:
        return self.norm01(self.neutral_applied)

    def deviation(self, applied: float) -> float:
        """Bipolar position around neutral: -1 = minApplied, 0 = neutral,
        +1 = maxApplied, each side scaled by its own span."""
        n = self.norm01(applied)
        n0 = self.neutral_norm01()
        span = (1.0 - n0) if n >= n0 else n0
        if span <= 1e-6:
            return 0.0
        return clamp((n - n0) / span, -1.0, 1.0)


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


class FaceRenderer:
    """Line-art face in face-local coordinates: origin at FACE_CENTER,
    +x = screen right = subject's LEFT (channel names are subject-relative,
    mirrored like the original topology view), +y = down.

    Direction conventions per group (assumed, see module docstring):
    +deviation = brow raised, lid closed, upper corner up / lower corner
    down, upper lip up / lower lip down, tendon pulled up. Channel 25's
    open direction is unconfirmed, so jaw open uses |deviation|.
    """

    MAX_TILT_DEG = 12.0
    JAW_OPEN_RANGE_PX = 60.0
    EYE_GAZE_RANGE_PX = 12.0
    JAW_SHIFT_RANGE_PX = 14.0

    EYE_HALF_WIDTH = 34.0
    EYE_APERTURE_UPPER = 15.0
    EYE_APERTURE_LOWER = 11.0

    def __init__(self, channels: list[Channel]) -> None:
        self.channels = channels
        self.show_nodes = True
        pygame.font.init()
        self.font = pygame.font.SysFont("monospace", 18)
        self.font_small = pygame.font.SysFont("monospace", 13, bold=True)
        self.font_big = pygame.font.SysFont("monospace", 36, bold=True)

    def dev(self, channel_id: int, angles: list[float]) -> float:
        return self.channels[channel_id].deviation(angles[channel_id])

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

        if self.show_nodes:
            self._draw_nodes(surface, nodes)

    def _curve(
        self, surface, points: list[Point], to_screen: Transform, color, width
    ) -> None:
        pts = [to_screen(x, y) for x, y in catmull_rom(points)]
        pygame.draw.lines(surface, color, False, pts, width)

    def _draw_neck(self, surface, angles, fixed: Transform, nodes) -> None:
        # The neck pair stays in screen space (the head tilts on top of it).
        for side, ch in ((1, 30), (-1, 31)):
            pygame.draw.line(
                surface, OUTLINE_COLOR, fixed(side * 72, 200), fixed(side * 95, 292), 4
            )
            nodes[ch] = fixed(side * 84, 248 - 14.0 * self.dev(ch, angles))

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

        # Jaw outline: cheeks -> jaw corner channels 26/27 -> chin. Opening
        # drops it (weights grow toward the chin) and 24 shears it sideways.
        d26 = self.dev(26, angles)
        d27 = self.dev(27, angles)
        chin = [
            (-160.0, 80.0),
            (-95.0 + jaw_x, 170.0 + 0.7 * jaw_open + 16.0 * d26),
            (jaw_x, 195.0 + jaw_open),
            (95.0 + jaw_x, 170.0 + 0.7 * jaw_open + 16.0 * d27),
            (160.0, 80.0),
        ]
        self._curve(surface, chin, to_screen, OUTLINE_COLOR, 3)

        nodes[26] = to_screen(*chin[1])
        nodes[27] = to_screen(*chin[3])
        nodes[24] = to_screen(jaw_x, 168.0 + 0.9 * jaw_open)
        nodes[25] = to_screen(-85.0, 145.0 + jaw_open)
        # Tongue channels (usually disabled): parked inside the jaw region.
        nodes[28] = to_screen(-20.0, 150.0 + 0.8 * jaw_open)
        nodes[29] = to_screen(20.0, 162.0 + 0.8 * jaw_open)

    def _draw_brows(self, surface, angles, to_screen: Transform, nodes) -> None:
        # Subject-right brow (0/1) on screen-left. Outer ends sit higher,
        # matching the original topology layout.
        for side, inner_ch, outer_ch in ((-1, 0, 1), (1, 2, 3)):
            ix, iy = side * 70.0, -168.0 - 26.0 * self.dev(inner_ch, angles)
            ox, oy = side * 130.0, -180.0 - 22.0 * self.dev(outer_ch, angles)
            mid = ((ix + ox) / 2.0, (iy + oy) / 2.0 - 5.0)
            self._curve(surface, [(ix, iy), mid, (ox, oy)], to_screen, LINE_COLOR, 5)
            nodes[inner_ch] = to_screen(ix, iy)
            nodes[outer_ch] = to_screen(ox, oy)

    def _draw_eyes(self, surface, angles, to_screen: Transform, nodes) -> None:
        gaze_x = self.dev(8, angles) * self.EYE_GAZE_RANGE_PX
        gaze_y = -self.dev(13, angles) * self.EYE_GAZE_RANGE_PX  # + = look up

        # side=-1 -> screen-left eye = subject's RIGHT (channels 11/12).
        for side, upper_ch, lower_ch in ((-1, 11, 12), (1, 9, 10)):
            cx, cy = side * 75.0, -120.0
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

            nodes[upper_ch] = to_screen(cx, cy - ap_u)
            nodes[lower_ch] = to_screen(cx, cy + ap_l)

        # Shared gaze mechanisms get their own dots between the eyes, moving
        # along the axis they steer.
        nodes[8] = to_screen(gaze_x, -132.0)
        nodes[13] = to_screen(0.0, -108.0 + gaze_y)

    def _draw_nose(self, surface, angles, to_screen: Transform, nodes) -> None:
        pygame.draw.line(
            surface, OUTLINE_COLOR, to_screen(0, -104), to_screen(0, -46), 2
        )
        # Nose tendons scrunch their side of the nose base upward.
        lift_l = -8.0 * self.dev(5, angles)  # subject left = +x
        lift_r = -8.0 * self.dev(6, angles)
        base = [(-24.0, -40.0 + lift_r), (0.0, -30.0), (24.0, -40.0 + lift_l)]
        self._curve(surface, base, to_screen, LINE_COLOR, 3)
        nodes[5] = to_screen(92.0, -80.0 - 14.0 * self.dev(5, angles))
        nodes[6] = to_screen(-92.0, -80.0 - 14.0 * self.dev(6, angles))

    def _draw_cheeks(self, surface, angles, to_screen: Transform, nodes) -> None:
        # Cheek tendon pulls its arc up and outward (smile apple / squint).
        for side, ch in ((1, 4), (-1, 7)):
            d = self.dev(ch, angles)
            ax, ay = side * (150.0 + 6.0 * d), -45.0 - 18.0 * d
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
        up_lift = -0.12 * jaw_open  # upper lip eases up slightly as jaw drops
        low_drop = 0.85 * jaw_open

        def pt(
            ch: int, x: float, y: float, move_y: float, shift_x: float = 0.0
        ) -> Point:
            p = (x + shift_x, y + move_y * self.dev(ch, angles))
            nodes[ch] = to_screen(*p)
            return p

        # Lips rest nearly closed (thin lens); jaw open and the individual
        # lip/corner channels separate them.
        # Upper lip through both corner_upper channels; + = corner/lip up.
        upper = [
            pt(17, -105.0, 52.0 + up_lift, -24.0),
            pt(16, -52.0, 46.0 + up_lift, -14.0),
            pt(15, 0.0, 44.0 + up_lift, -12.0),
            pt(14, 52.0, 46.0 + up_lift, -14.0),
            pt(19, 105.0, 52.0 + up_lift, -24.0),
        ]
        # Lower lip through both corner_lower channels; + = pulled down.
        # The lower half also follows the jaw sideways (channel 24).
        lower = [
            pt(18, -108.0, 60.0 + low_drop, 20.0, jaw_x),
            pt(22, -52.0, 58.0 + low_drop, 16.0, jaw_x),
            pt(23, 0.0, 62.0 + low_drop, 18.0, jaw_x),
            pt(21, 52.0, 58.0 + low_drop, 16.0, jaw_x),
            pt(20, 108.0, 60.0 + low_drop, 20.0, jaw_x),
        ]

        interior = [
            to_screen(x, y) for x, y in catmull_rom(upper) + catmull_rom(lower[::-1])
        ]
        pygame.draw.polygon(surface, MOUTH_INNER, interior)
        self._curve(surface, upper, to_screen, LIP_COLOR, 4)
        self._curve(surface, lower, to_screen, LIP_COLOR, 4)

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
