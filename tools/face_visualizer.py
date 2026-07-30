"""Standalone 2D face visualizer: a drop-in replacement for the Raspberry Pi
executor for development without hardware.

Listens on a UDP port for the exact wire protocol the Rust ControlService
sends (`{"frameId", "timestampNs", "source", "angles"}`, 32 floats), and
renders a simple front-facing cartoon face whose parts track those 32
channels. The upstream host app needs zero changes: point its UDP endpoint
at this process's host:port instead of the Raspberry Pi's.

Usage:
    python3 tools/face_visualizer.py [--host 0.0.0.0] [--port 6000] [--config PATH]

Per-channel limits (min/max/neutral applied degrees) are read from
`src-tauri/config/motor_config.json` so this script never carries a second,
divergent copy of MOTOR_LIMITS. Run `python3 raspi/export_config_json.py`
first if that file doesn't exist yet.

Frames arriving over UDP are already rate-limited/interpolated by the Rust
side, so this script renders each received frame directly with no local
smoothing -- it just holds the last frame's angles until the next one
arrives.

Note: this uses a single flat [0, 1] `(angle - min) / (max - min)` mapping
per channel purely for screen placement. That is unrelated to (and simpler
than) control.rs's bipolar, neutral-anchored norm space used internally for
presets/coupling -- this script only ever sees the final wire-protocol
angles, not that internal representation.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import pygame

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = ROOT / "src-tauri" / "config" / "motor_config.json"
MOTOR_COUNT = 32

SCREEN_SIZE = (900, 720)
FACE_CENTER = (450, 380)
SIGNAL_LOST_TIMEOUT_S = 1.0

BACKGROUND = (24, 26, 32)
FACE_COLOR = (235, 205, 180)
LINE_COLOR = (40, 30, 25)
EYE_WHITE = (250, 250, 250)
PUPIL_COLOR = (20, 20, 25)
LID_COLOR = FACE_COLOR
MOUTH_COLOR = (150, 50, 60)
HUD_COLOR = (200, 220, 230)
LOST_COLOR = (230, 60, 60)


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
        """Flat [0, 1] position within this channel's limits, 0.5 == neutral only
        incidentally (only exactly true if neutral is the midpoint)."""
        span = self.max_applied - self.min_applied
        if span <= 1e-6:
            return 0.5
        return clamp((applied - self.min_applied) / span, 0.0, 1.0)

    def neutral_norm01(self) -> float:
        return self.norm01(self.neutral_applied)


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


class FaceRenderer:
    # Neck tilt: total left/right lean, scaled by how far 30/31 sit from
    # their own neutral. Sign/scale are a visual approximation, not a
    # physical calibration.
    MAX_TILT_DEG = 12.0

    BROW_RANGE_PX = 32.0
    EYE_GAZE_RANGE_PX = 10.0
    MOUTH_HALF_WIDTH = 130.0
    MOUTH_LIP_RANGE_PX = 22.0
    JAW_OPEN_RANGE_PX = 60.0

    def __init__(self, channels: list[Channel]) -> None:
        self.channels = channels
        pygame.font.init()
        self.font = pygame.font.SysFont("monospace", 18)
        self.font_big = pygame.font.SysFont("monospace", 36, bold=True)

    def norm(self, channel_id: int, angles: list[float]) -> float:
        return self.channels[channel_id].norm01(angles[channel_id])

    def neutral_norm(self, channel_id: int) -> float:
        return self.channels[channel_id].neutral_norm01()

    def tilt_radians(self, angles: list[float]) -> float:
        left = self.norm(30, angles) - self.neutral_norm(30)
        right = self.norm(31, angles) - self.neutral_norm(31)
        tilt_deg = clamp((left - right) * self.MAX_TILT_DEG, -self.MAX_TILT_DEG, self.MAX_TILT_DEG)
        return math.radians(tilt_deg)

    def jaw_open_px(self, angles: list[float]) -> float:
        # Primary jaw axis is channel 25; direction of "open" vs "closed" on
        # the real hardware is unconfirmed, so this uses distance from
        # neutral in either direction as an open amount. TODO: hardware
        # calibration may reveal this should be one-sided.
        deviation = abs(self.norm(25, angles) - self.neutral_norm(25))
        return clamp(deviation * 2.0, 0.0, 1.0) * self.JAW_OPEN_RANGE_PX

    def draw(self, surface: pygame.Surface, angles: list[float]) -> None:
        tilt = self.tilt_radians(angles)
        jaw_open = self.jaw_open_px(angles)

        def to_screen(x: float, y: float) -> tuple[int, int]:
            rx, ry = rotate(x, y, tilt)
            return int(FACE_CENTER[0] + rx), int(FACE_CENTER[1] + ry)

        face_rect = self._face_bounds(240, 300)
        pygame.draw.ellipse(surface, FACE_COLOR, face_rect, 0)
        pygame.draw.ellipse(surface, LINE_COLOR, face_rect, 3)

        self._draw_eyebrows(surface, angles, to_screen)
        self._draw_eye(surface, angles, side=1, to_screen=to_screen)
        self._draw_eye(surface, angles, side=-1, to_screen=to_screen)
        self._draw_mouth(surface, angles, jaw_open, to_screen)

    def _face_bounds(self, w: float, h: float) -> pygame.Rect:
        # The face outline stays axis-aligned (an ellipse can't be rotated
        # in place without rendering to an intermediate surface); only the
        # features drawn via to_screen() actually tilt. Good enough for
        # "does the animation read correctly" rather than a physical render.
        return pygame.Rect(FACE_CENTER[0] - w / 2, FACE_CENTER[1] - h / 2, w, h)

    def _draw_eyebrows(self, surface, angles, to_screen) -> None:
        # Screen-left holds the character's anatomical right brow (0, 1),
        # mirrored, as is conventional for a front-facing face.
        for side, inner_ch, outer_ch in ((-1, 0, 1), (1, 2, 3)):
            inner_y = -140 + (self.norm(inner_ch, angles) - self.neutral_norm(inner_ch)) * -self.BROW_RANGE_PX
            outer_y = -132 + (self.norm(outer_ch, angles) - self.neutral_norm(outer_ch)) * -self.BROW_RANGE_PX
            inner_x = side * 45
            outer_x = side * 115
            pygame.draw.line(
                surface, LINE_COLOR, to_screen(inner_x, inner_y), to_screen(outer_x, outer_y), 6
            )

    def _draw_eye(self, surface, angles, side: int, to_screen) -> None:
        # side=-1 -> screen-left eye (anatomical right: channels 11/12),
        # side=+1 -> screen-right eye (anatomical left: channels 9/10).
        if side == -1:
            upper_ch, lower_ch = 11, 12
        else:
            upper_ch, lower_ch = 9, 10

        cx, cy = side * 80, -40
        radius = 34

        gaze_x = (self.norm(8, angles) - self.neutral_norm(8)) * self.EYE_GAZE_RANGE_PX
        gaze_y = (self.norm(13, angles) - self.neutral_norm(13)) * self.EYE_GAZE_RANGE_PX

        eye_center = to_screen(cx, cy)
        pygame.draw.circle(surface, EYE_WHITE, eye_center, radius)
        pygame.draw.circle(surface, LINE_COLOR, eye_center, radius, 2)

        pupil_center = to_screen(cx + gaze_x, cy + gaze_y)
        pygame.draw.circle(surface, PUPIL_COLOR, pupil_center, radius // 2)

        upper_cover = self.norm(upper_ch, angles)
        lower_cover = self.norm(lower_ch, angles)

        eye_box = pygame.Rect(eye_center[0] - radius, eye_center[1] - radius, radius * 2, radius * 2)
        upper_lid_h = int(upper_cover * radius * 2)
        lower_lid_h = int(lower_cover * radius * 2)
        if upper_lid_h > 0:
            pygame.draw.rect(surface, LID_COLOR, (eye_box.x, eye_box.y, eye_box.width, upper_lid_h))
        if lower_lid_h > 0:
            pygame.draw.rect(
                surface,
                LID_COLOR,
                (eye_box.x, eye_box.bottom - lower_lid_h, eye_box.width, lower_lid_h),
            )
        pygame.draw.circle(surface, LINE_COLOR, eye_center, radius, 2)

    def _draw_mouth(self, surface, angles, jaw_open: float, to_screen) -> None:
        mw = self.MOUTH_HALF_WIDTH
        rng = self.MOUTH_LIP_RANGE_PX
        base_y = 130

        def lip_y(channel_id: int, baseline: float, jaw_share: float) -> float:
            offset = (self.norm(channel_id, angles) - self.neutral_norm(channel_id)) * rng
            return baseline + offset + jaw_open * jaw_share

        # Upper edge, left corner -> right corner. Upper lip opens upward a
        # little as the jaw drops; lower edge opens downward a lot more.
        upper_points = [
            (-mw, lip_y(19, base_y, -0.15)),  # mouth_left_corner_upper
            (-mw * 0.45, lip_y(14, base_y, -0.15)),  # upper_lip_left
            (0, lip_y(15, base_y, -0.15)),  # upper_lip_mid
            (mw * 0.45, lip_y(16, base_y, -0.15)),  # upper_lip_right
            (mw, lip_y(17, base_y, -0.15)),  # mouth_right_corner_upper
        ]
        lower_points = [
            (mw, lip_y(18, base_y + 10, 0.85)),  # mouth_right_corner_lower
            (mw * 0.45, lip_y(22, base_y + 10, 0.85)),  # lower_lip_right
            (0, lip_y(23, base_y + 10, 0.85)),  # lower_lip_mid_tendon
            (-mw * 0.45, lip_y(21, base_y + 10, 0.85)),  # lower_lip_left
            (-mw, lip_y(20, base_y + 10, 0.85)),  # mouth_left_corner_lower
        ]
        loop = upper_points + lower_points
        screen_points = [to_screen(x, y) for x, y in loop]
        pygame.draw.polygon(surface, MOUTH_COLOR, screen_points)
        pygame.draw.polygon(surface, LINE_COLOR, screen_points, 3)


def format_hud_time(seconds_ago: float) -> str:
    if math.isinf(seconds_ago):
        return "never"
    return f"{seconds_ago:.2f}s ago"


def draw_hud(surface: pygame.Surface, renderer: FaceRenderer, receiver: FrameReceiver, fps: float) -> None:
    latest = receiver.latest
    since = receiver.seconds_since_last_frame()

    top_left = renderer.font.render(f"FPS: {fps:5.1f}", True, HUD_COLOR)
    surface.blit(top_left, (12, 8))

    top_right_text = f"frame={latest.frame_id if latest.frame_id is not None else '-'} src={latest.source or '-'}"
    top_right = renderer.font.render(top_right_text, True, HUD_COLOR)
    surface.blit(top_right, (SCREEN_SIZE[0] - top_right.get_width() - 12, 8))

    bottom_left = renderer.font.render(f"last frame: {format_hud_time(since)}", True, HUD_COLOR)
    surface.blit(bottom_left, (12, SCREEN_SIZE[1] - 26))

    if since > SIGNAL_LOST_TIMEOUT_S:
        lost = renderer.font_big.render("信号丢失 / SIGNAL LOST", True, LOST_COLOR)
        surface.blit(lost, (SCREEN_SIZE[0] - lost.get_width() - 12, SCREEN_SIZE[1] - lost.get_height() - 12))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Standalone UDP-driven 2D face visualizer")
    parser.add_argument("--host", default="0.0.0.0", help="UDP bind host (default 0.0.0.0)")
    parser.add_argument("--port", type=int, default=6000, help="UDP bind port (default 6000)")
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
    pygame.display.set_caption(f"BionicFace Visualizer - listening on {args.host}:{args.port}")
    clock = pygame.time.Clock()

    receiver = FrameReceiver(args.host, args.port, neutral_angles)
    renderer = FaceRenderer(channels)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False

        receiver.poll()

        screen.fill(BACKGROUND)
        renderer.draw(screen, receiver.latest.angles)
        draw_hud(screen, renderer, receiver, clock.get_fps())
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()
