"""Click-to-place anchor annotation over a photo of the real face.

The twin-face renderers place every channel's anchor dot from hand-written
pixel constants that were inherited from the old topology view and never
measured against the machine ("meeting problem 1"). This produces the ground
truth to replace them: a frontal photo of the skinless face plus one clicked
point per channel, saved as image-normalized coordinates.

Meant to be driven alongside tools/jog_channel.py: jog a channel, watch which
part of the face moves, click that spot here, move on. One sweep fills both
the verification ledger and this file.

Usage:
    python tools/annotate_anchors.py docs/hardware/face_frontal.jpg
    keys: click place + advance | n/p skip/back | u undo | digits+Enter jump
          | q save & quit
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = REPO_ROOT / "src-tauri" / "config" / "motor_config.json"
DEFAULT_OUTPUT = REPO_ROOT / "docs" / "hardware" / "face_anchors.json"

FONT = cv2.FONT_HERSHEY_SIMPLEX
PLACED = (110, 220, 140)
CURRENT = (80, 190, 240)
GREY = (170, 170, 170)


def load_channels(config_path: Path) -> dict[int, str]:
    raw = json.loads(config_path.read_text(encoding="utf-8"))
    return {c["id"]: c["name"] for c in raw["channels"]}


class Annotator:
    def __init__(
        self, image, channels: dict[int, str], existing: dict[int, list[float]]
    ):
        self.image = image
        self.channels = channels
        self.order = sorted(channels)
        self.anchors: dict[int, list[float]] = dict(existing)
        self.index = 0
        self.history: list[int] = []
        # Start on the first channel without an anchor yet.
        for pos, channel_id in enumerate(self.order):
            if channel_id not in self.anchors:
                self.index = pos
                break

    @property
    def current(self) -> int:
        return self.order[self.index]

    def place(self, x: int, y: int) -> None:
        height, width = self.image.shape[:2]
        self.anchors[self.current] = [round(x / width, 4), round(y / height, 4)]
        self.history.append(self.current)
        self.advance(1)

    def advance(self, step: int) -> None:
        self.index = (self.index + step) % len(self.order)

    def undo(self) -> None:
        if not self.history:
            return
        channel_id = self.history.pop()
        self.anchors.pop(channel_id, None)
        self.index = self.order.index(channel_id)

    def render(self, pending_digits: str):
        canvas = self.image.copy()
        height, width = canvas.shape[:2]
        for channel_id, (nx, ny) in self.anchors.items():
            point = (int(nx * width), int(ny * height))
            cv2.circle(canvas, point, 5, PLACED, -1)
            cv2.putText(
                canvas,
                str(channel_id),
                (point[0] + 7, point[1] - 5),
                FONT,
                0.5,
                PLACED,
                2,
            )
        head = f"ch{self.current} {self.channels[self.current]}"
        cv2.putText(canvas, head, (10, 28), FONT, 0.8, CURRENT, 2)
        state = f"{len(self.anchors)}/{len(self.order)} placed"
        if self.current in self.anchors:
            state += "  (re-click to move this one)"
        cv2.putText(canvas, state, (10, 54), FONT, 0.55, GREY, 1)
        if pending_digits:
            cv2.putText(
                canvas, f"goto: {pending_digits}_", (10, 80), FONT, 0.55, CURRENT, 2
            )
        cv2.putText(
            canvas,
            "click place | n/p skip/back | u undo | digits+Enter goto | q save",
            (10, height - 12),
            FONT,
            0.5,
            GREY,
            1,
        )
        return canvas


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("photo", type=Path)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)

    image = cv2.imread(str(args.photo))
    if image is None:
        print(f"error: cannot read image {args.photo}", file=sys.stderr)
        return 1
    channels = load_channels(args.config)

    existing: dict[int, list[float]] = {}
    if args.output.exists():
        saved = json.loads(args.output.read_text(encoding="utf-8"))
        existing = {int(k): v for k, v in saved.get("anchors", {}).items()}
        print(f"resuming: {len(existing)} anchors already in {args.output}")

    annotator = Annotator(image, channels, existing)
    clicks: list[tuple[int, int]] = []

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicks.append((x, y))

    window = "anchor annotation"
    cv2.namedWindow(window)
    cv2.setMouseCallback(window, on_mouse)

    pending = ""
    while True:
        while clicks:
            annotator.place(*clicks.pop(0))
        cv2.imshow(window, annotator.render(pending))
        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break
        if key in (ord("n"),):
            annotator.advance(1)
        elif key == ord("p"):
            annotator.advance(-1)
        elif key == ord("u"):
            annotator.undo()
        elif ord("0") <= key <= ord("9"):
            pending += chr(key)
        elif key in (13, 10) and pending:
            target = int(pending)
            pending = ""
            if target in channels:
                annotator.index = annotator.order.index(target)
        elif key == 27:
            pending = ""

    cv2.destroyAllWindows()
    height, width = image.shape[:2]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(
            {
                "image": str(args.photo),
                "image_size": [width, height],
                "coordinates": "fractions of image width/height, origin top-left",
                "anchors": {str(k): v for k, v in sorted(annotator.anchors.items())},
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"wrote {len(annotator.anchors)} anchors to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
