"""Minimal WHEELTEC C100/C70 capture module with locked imaging parameters.

Streaming goes through OpenCV; imaging controls go through V4L2 ioctls addressed
by numeric CID. The split is deliberate: OpenCV's V4L2 backend swallows
`VIDIOC_S_CTRL` failures -- `cap_v4l.cpp` says so in as many words ("The driver
may clamp the value or return ERANGE, ignored here") -- so `cap.set()` cannot be
used to *prove* a parameter was applied. Every control written here is read back
and compared; a mismatch raises instead of degrading silently.

CIDs are a stable kernel ABI, control *names* are not (`auto_exposure` used to be
`exposure_auto`), so the JSON keys here are our own vocabulary mapped to CIDs.

See docs/camera/PARAM_LOCK.md for the control table, the master/slave write
order and the boot-time restore flow, and docs/camera/CAPABILITIES.md for what
the vendor documentation does and does not pin down.

Usage:
    python tools/camera_capture.py list     [--device /dev/video0]
    python tools/camera_capture.py dump     [--config tools/camera_params.json]
    python tools/camera_capture.py lock     [--config tools/camera_params.json]
    python tools/camera_capture.py verify   [--config tools/camera_params.json]
    python tools/camera_capture.py selftest [--config tools/camera_params.json]

`verify` is the post-reboot check: it re-reads every control and reports
per-item differences against the JSON parameter set, exiting non-zero on any.
"""

from __future__ import annotations

import argparse
import ctypes
import errno
import fcntl
import itertools
import json
import os
import statistics
import sys
import time
from collections.abc import Iterator
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any

import cv2
import numpy as np

DEFAULT_CONFIG = Path(__file__).with_name("camera_params.json")
DEFAULT_DEVICE = "/dev/video0"
FPS_TOLERANCE = 0.5

# --- V4L2 ioctl layer -------------------------------------------------------
#
# Structures and ioctl numbers mirror include/uapi/linux/videodev2.h. uvcvideo
# only implements the ext-ctrl ops, but v4l2-ioctl.c forwards the legacy
# QUERYCTRL/G_CTRL/S_CTRL ioctls to them, so these simple structs are enough.


class _V4L2Control(ctypes.Structure):
    _fields_ = [("id", ctypes.c_uint32), ("value", ctypes.c_int32)]


class _V4L2QueryCtrl(ctypes.Structure):
    _fields_ = [
        ("id", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("name", ctypes.c_char * 32),
        ("minimum", ctypes.c_int32),
        ("maximum", ctypes.c_int32),
        ("step", ctypes.c_int32),
        ("default_value", ctypes.c_int32),
        ("flags", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32 * 2),
    ]


def _iowr(nr: int, size: int) -> int:
    """_IOWR('V', nr, size) -- asm-generic encoding, dir = READ|WRITE = 3."""
    return (3 << 30) | (size << 16) | (ord("V") << 8) | nr


VIDIOC_G_CTRL = _iowr(27, ctypes.sizeof(_V4L2Control))
VIDIOC_S_CTRL = _iowr(28, ctypes.sizeof(_V4L2Control))
VIDIOC_QUERYCTRL = _iowr(36, ctypes.sizeof(_V4L2QueryCtrl))

CTRL_FLAG_NEXT = 0x80000000

FLAG_DISABLED = 0x0001
FLAG_GRABBED = 0x0002
FLAG_READ_ONLY = 0x0004
FLAG_INACTIVE = 0x0010

TYPE_CTRL_CLASS = 6

_TYPE_NAMES = {
    1: "int",
    2: "bool",
    3: "menu",
    4: "button",
    5: "int64",
    TYPE_CTRL_CLASS: "class",
    7: "string",
    8: "bitmask",
    9: "intmenu",
}

_USER_BASE = 0x00980900
_CAMERA_BASE = 0x009A0900

# JSON key -> CID. Values from include/uapi/linux/v4l2-controls.h.
CONTROLS: dict[str, int] = {
    "brightness": _USER_BASE + 0,
    "contrast": _USER_BASE + 1,
    "saturation": _USER_BASE + 2,
    "hue": _USER_BASE + 3,
    "white_balance_automatic": _USER_BASE + 12,
    "gamma": _USER_BASE + 16,
    "gain": _USER_BASE + 19,
    "power_line_frequency": _USER_BASE + 24,
    "white_balance_temperature": _USER_BASE + 26,
    "sharpness": _USER_BASE + 27,
    "backlight_compensation": _USER_BASE + 28,
    "auto_exposure": _CAMERA_BASE + 1,
    "exposure_time_absolute": _CAMERA_BASE + 2,
    "exposure_dynamic_framerate": _CAMERA_BASE + 3,
    "focus_absolute": _CAMERA_BASE + 10,
    "focus_automatic_continuous": _CAMERA_BASE + 12,
    # Digital pan/tilt/zoom. These crop and re-frame the sensor output, so a
    # drifting zoom changes landmark geometry as surely as moving the camera
    # would -- they belong in the locked set on any device that exposes them.
    "pan_absolute": _CAMERA_BASE + 8,
    "tilt_absolute": _CAMERA_BASE + 9,
    "zoom_absolute": _CAMERA_BASE + 13,
}

_CID_TO_NAME = {cid: name for name, cid in CONTROLS.items()}

# uvcvideo refuses to write a slave control while its master is still in
# automatic mode (uvc_ctrl.c returns -EACCES), so the auto switches go first.
# exposure_dynamic_framerate is not a master; it leads because it decides
# whether the frame interval may drift, and the interval bounds the exposure.
_APPLY_FIRST = (
    "auto_exposure",
    "white_balance_automatic",
    "focus_automatic_continuous",
    "exposure_dynamic_framerate",
)


class CameraError(RuntimeError):
    """Device, configuration or format-negotiation failure."""


class ControlUnsupported(CameraError):
    """The device does not expose this control at all."""


class ParamLockError(CameraError):
    """A control did not read back as the value that was written."""


@dataclass(frozen=True)
class ControlInfo:
    """One control as reported by VIDIOC_QUERYCTRL."""

    name: str
    cid: int
    driver_name: str
    type: int
    minimum: int
    maximum: int
    step: int
    default: int
    flags: int

    @property
    def inactive(self) -> bool:
        """True while a master control still holds this one in automatic mode."""
        return bool(self.flags & FLAG_INACTIVE)

    @property
    def writable(self) -> bool:
        return not self.flags & (FLAG_DISABLED | FLAG_READ_ONLY | FLAG_GRABBED)

    @property
    def type_name(self) -> str:
        return _TYPE_NAMES.get(self.type, str(self.type))

    @property
    def range_text(self) -> str:
        return (
            f"{self.minimum}..{self.maximum} step {self.step or 1} def {self.default}"
        )

    @property
    def flags_text(self) -> str:
        labels = [
            label
            for bit, label in (
                (FLAG_DISABLED, "disabled"),
                (FLAG_GRABBED, "grabbed"),
                (FLAG_READ_ONLY, "read-only"),
                (FLAG_INACTIVE, "inactive"),
            )
            if self.flags & bit
        ]
        return ",".join(labels) or "-"

    def validate(self, value: int) -> None:
        """Reject values the driver would clamp, rather than clamping them here."""
        if not self.writable:
            raise CameraError(
                f"control {self.name!r} is not writable ({self.flags_text})"
            )
        if not self.minimum <= value <= self.maximum:
            raise CameraError(
                f"control {self.name!r} value {value} outside {self.range_text}"
            )
        step = self.step or 1
        if step > 1 and (value - self.minimum) % step:
            raise CameraError(
                f"control {self.name!r} value {value} is not on step {step} "
                f"from minimum {self.minimum}"
            )


class V4L2Device:
    """Control-plane access to a V4L2 node, independent of any streaming fd.

    Kept separate from the capture fd on purpose: controls can be read and
    written without streaming (that is what the post-reboot `verify` needs), and
    the vendor's own example does the same (cam_usb_set/set.cpp).
    """

    def __init__(self, path: str = DEFAULT_DEVICE) -> None:
        self.path = path
        self._fd: int | None = None

    # PYI034 wants `typing.Self`, which needs Python 3.11; this project targets 3.10.
    def __enter__(self) -> V4L2Device:  # noqa: PYI034
        self.open()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def open(self) -> None:
        if self._fd is not None:
            return
        try:
            self._fd = os.open(self.path, os.O_RDWR)
        except OSError as exc:
            raise CameraError(f"cannot open {self.path}: {exc.strerror}") from exc

    def close(self) -> None:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    @property
    def fd(self) -> int:
        if self._fd is None:
            raise CameraError(f"{self.path} is not open")
        return self._fd

    def query(self, name: str) -> ControlInfo:
        return self._info(self._queryctrl(_cid(name)), name)

    def iter_controls(self) -> Iterator[ControlInfo]:
        """Walk every control the device exposes, via V4L2_CTRL_FLAG_NEXT_CTRL.

        The device is the only authority on which controls exist and what their
        ranges are -- the vendor documentation lists none of them.
        """
        cid = 0
        while True:
            try:
                query = self._queryctrl(cid | CTRL_FLAG_NEXT)
            except ControlUnsupported:
                return
            if query.type != TYPE_CTRL_CLASS:
                yield self._info(query, _CID_TO_NAME.get(query.id, hex(query.id)))
            cid = query.id

    def get(self, name: str) -> int:
        return self.get_cid(_cid(name))

    def get_cid(self, cid: int) -> int:
        control = _V4L2Control(id=cid)
        try:
            fcntl.ioctl(self.fd, VIDIOC_G_CTRL, control)
        except OSError as exc:
            label = _CID_TO_NAME.get(cid, hex(cid))
            raise CameraError(f"VIDIOC_G_CTRL {label}: {exc.strerror}") from exc
        return control.value

    def set(self, name: str, value: int) -> None:
        self.query(name).validate(value)
        control = _V4L2Control(id=_cid(name), value=value)
        try:
            fcntl.ioctl(self.fd, VIDIOC_S_CTRL, control)
        except OSError as exc:
            hint = ""
            if exc.errno == errno.EACCES:
                hint = (
                    " (its master control is still automatic -- see PARAM_LOCK.md §4)"
                )
            raise CameraError(
                f"VIDIOC_S_CTRL {name}={value}: {exc.strerror}{hint}"
            ) from exc

    def _queryctrl(self, cid: int) -> _V4L2QueryCtrl:
        query = _V4L2QueryCtrl(id=cid)
        try:
            fcntl.ioctl(self.fd, VIDIOC_QUERYCTRL, query)
        except OSError as exc:
            if exc.errno == errno.EINVAL:
                label = _CID_TO_NAME.get(cid & ~CTRL_FLAG_NEXT, hex(cid))
                raise ControlUnsupported(
                    f"{self.path} has no control {label!r}"
                ) from exc
            raise CameraError(f"VIDIOC_QUERYCTRL {hex(cid)}: {exc.strerror}") from exc
        return query

    @staticmethod
    def _info(query: _V4L2QueryCtrl, name: str) -> ControlInfo:
        return ControlInfo(
            name=name,
            cid=query.id,
            driver_name=query.name.decode("ascii", "replace"),
            type=query.type,
            minimum=query.minimum,
            maximum=query.maximum,
            step=query.step,
            default=query.default_value,
            flags=query.flags,
        )


def _cid(name: str) -> int:
    try:
        return CONTROLS[name]
    except KeyError:
        raise CameraError(
            f"unknown control {name!r}; known: {', '.join(sorted(CONTROLS))}"
        ) from None


# --- Configuration ----------------------------------------------------------


@dataclass
class CaptureConfig:
    """One reproducible camera setup: stream format plus a locked control set.

    A `controls` value may be None, meaning "not yet measured on real hardware".
    Locking or verifying such a set fails loudly rather than inventing a number.
    """

    device: str = DEFAULT_DEVICE
    width: int = 640
    height: int = 480
    fps: float = 30.0
    fourcc: str = "MJPG"
    controls: dict[str, int | None] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if len(self.fourcc) != 4:
            raise CameraError(f"fourcc must be 4 characters, got {self.fourcc!r}")
        if self.width <= 0 or self.height <= 0 or self.fps <= 0:
            raise CameraError("width, height and fps must be positive")
        for name, value in self.controls.items():
            _cid(name)
            if value is not None and not isinstance(value, int):
                raise CameraError(
                    f"control {name!r} must be an int or null, got {value!r}"
                )

    @classmethod
    def load(cls, path: Path) -> CaptureConfig:
        try:
            raw: dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))
        except FileNotFoundError as exc:
            raise CameraError(f"no parameter set at {path}") from exc
        except json.JSONDecodeError as exc:
            raise CameraError(f"{path} is not valid JSON: {exc}") from exc
        unknown = sorted(set(raw) - {f.name for f in fields(cls)})
        if unknown:
            raise CameraError(f"{path}: unknown keys {unknown}")
        return cls(**raw)

    def save(self, path: Path) -> None:
        payload = {f.name: getattr(self, f.name) for f in fields(self)}
        path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    def locked_controls(self) -> dict[str, int]:
        """The control set, refusing any value that has not been measured yet."""
        missing = sorted(name for name, value in self.controls.items() if value is None)
        if missing:
            raise CameraError(
                f"controls not calibrated yet: {', '.join(missing)}; run "
                f"`camera_capture.py dump` on real hardware and fill them in"
            )
        if not self.controls:
            raise CameraError("the parameter set is empty; nothing to lock")
        return {
            name: value for name, value in self.controls.items() if value is not None
        }

    def apply_order(self) -> list[str]:
        """Masters (the auto switches) first; see _APPLY_FIRST."""
        names = list(self.locked_controls())
        return sorted(
            names,
            key=lambda name: (
                _APPLY_FIRST.index(name) if name in _APPLY_FIRST else len(_APPLY_FIRST)
            ),
        )


# --- Capture ----------------------------------------------------------------


@dataclass(frozen=True)
class Frame:
    """One captured frame.

    `timestamp` is `time.monotonic()` sampled the instant OpenCV handed the frame
    over -- arrival time in this process, not sensor exposure time. OpenCV does
    not surface the V4L2 buffer timestamp (which is CLOCK_MONOTONIC at hardware
    level), so exposure-accurate stamping would mean driving VIDIOC_DQBUF here.
    """

    index: int
    timestamp: float
    image: np.ndarray


class Camera:
    """open -> lock_params -> grab -> close, with every parameter read back."""

    def __init__(self, config: CaptureConfig) -> None:
        self.config = config
        self._cap: cv2.VideoCapture | None = None
        self._v4l2 = V4L2Device(config.device)
        self._frames = 0

    # PYI034 wants `typing.Self`, which needs Python 3.11; this project targets 3.10.
    def __enter__(self) -> Camera:  # noqa: PYI034
        self.open()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def open(self) -> None:
        """Open the device and negotiate the stream format.

        Format first, controls second: the exposure time is bounded by the
        negotiated frame interval (V4L2 ext-ctrls-camera documentation), so the
        format has to be settled before any exposure value is written.
        """
        if self._cap is not None:
            raise CameraError("camera is already open")
        cap = cv2.VideoCapture(self.config.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise CameraError(f"cannot open {self.config.device} with the V4L2 backend")
        self._cap = cap
        self._frames = 0
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*self.config.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        cap.set(cv2.CAP_PROP_FPS, self.config.fps)
        self._check_negotiated()
        self._v4l2.open()

    def negotiated_format(self) -> dict[str, Any]:
        cap = self._require_cap()
        fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        return {
            "fourcc": "".join(
                chr((fourcc >> shift) & 0xFF) for shift in (0, 8, 16, 24)
            ),
            "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "fps": cap.get(cv2.CAP_PROP_FPS),
        }

    def lock_params(self, config: CaptureConfig | None = None) -> dict[str, int]:
        """Apply a control set, then read every control back and compare.

        Returns the read-back values. Raises ParamLockError when the device did
        not take a value exactly -- silent clamping is the failure mode this
        whole module exists to catch.
        """
        config = config or self.config
        wanted = config.locked_controls()
        for name in config.apply_order():
            self._v4l2.set(name, wanted[name])
        readback = {name: self._v4l2.get(name) for name in wanted}
        mismatches = [
            f"{name}: wrote {wanted[name]}, read {readback[name]}"
            for name in wanted
            if readback[name] != wanted[name]
        ]
        if mismatches:
            raise ParamLockError(
                f"controls did not hold on {config.device}: {'; '.join(mismatches)}"
            )
        return readback

    def grab(self) -> Frame:
        cap = self._require_cap()
        ok, image = cap.read()
        timestamp = time.monotonic()
        if not ok or image is None:
            raise CameraError(
                f"frame {self._frames} read failed on {self.config.device}"
            )
        frame = Frame(index=self._frames, timestamp=timestamp, image=image)
        self._frames += 1
        return frame

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        self._v4l2.close()

    def _check_negotiated(self) -> None:
        actual = self.negotiated_format()
        differing = [
            f"{key}: asked {want!r}, got {actual[key]!r}"
            for key, want in (
                ("fourcc", self.config.fourcc),
                ("width", self.config.width),
                ("height", self.config.height),
            )
            if actual[key] != want
        ]
        if abs(actual["fps"] - self.config.fps) > FPS_TOLERANCE:
            differing.append(f"fps: asked {self.config.fps:g}, got {actual['fps']:g}")
        if differing:
            self.close()
            raise CameraError(
                f"{self.config.device} did not accept the requested format "
                f"({'; '.join(differing)}); `v4l2-ctl --list-formats-ext` shows the "
                f"supported modes"
            )

    def _require_cap(self) -> cv2.VideoCapture:
        if self._cap is None:
            raise CameraError("camera is not open")
        return self._cap


# --- Comparison and reporting ----------------------------------------------


@dataclass(frozen=True)
class ParamCheck:
    """One expected-vs-actual control comparison."""

    name: str
    expected: int
    actual: int | None  # None: the device does not expose this control
    info: ControlInfo | None

    @property
    def ok(self) -> bool:
        return self.actual == self.expected

    @property
    def note(self) -> str:
        if self.actual is None:
            return "control absent on this device"
        if self.info is not None and self.info.inactive:
            return "INACTIVE: its master control is still automatic"
        return ""


def compare(device: V4L2Device, expected: dict[str, int]) -> list[ParamCheck]:
    checks = []
    for name, value in expected.items():
        try:
            info = device.query(name)
        except ControlUnsupported:
            checks.append(ParamCheck(name, value, None, None))
            continue
        checks.append(ParamCheck(name, value, device.get(name), info))
    return checks


def report(checks: list[ParamCheck]) -> int:
    """Print one row per control; return the number of differences."""
    width = max((len(check.name) for check in checks), default=8)
    print(f"    {'control':<{width}}  {'expected':>9}  {'actual':>9}  range / flags")
    for check in checks:
        actual = "absent" if check.actual is None else str(check.actual)
        detail = (
            ""
            if check.info is None
            else f"{check.info.range_text} [{check.info.flags_text}]"
        )
        note = f"  <- {check.note}" if check.note else ""
        mark = "    " if check.ok else "DIFF"
        print(
            f"{mark} {check.name:<{width}}  {check.expected:>9}  {actual:>9}  {detail}{note}"
        )
    differences = sum(1 for check in checks if not check.ok)
    summary = "all match" if not differences else f"{differences} difference(s)"
    print(f"\n{len(checks)} controls checked, {summary}")
    return differences


@dataclass(frozen=True)
class RateStats:
    """Measured frame rate over a burst, from monotonic arrival timestamps."""

    frames: int
    elapsed: float
    fps: float
    interval_mean_ms: float
    interval_min_ms: float
    interval_max_ms: float
    interval_stdev_ms: float

    def describe(self, requested: float) -> str:
        return (
            f"{self.frames} frames in {self.elapsed:.3f}s -> {self.fps:.2f} fps "
            f"(requested {requested:g})\n"
            f"frame interval ms: mean {self.interval_mean_ms:.2f}, "
            f"min {self.interval_min_ms:.2f}, max {self.interval_max_ms:.2f}, "
            f"stdev {self.interval_stdev_ms:.2f}"
        )


def measure_rate(camera: Camera, count: int) -> RateStats:
    """Burst `count` frames back to back and derive the real rate from stamps."""
    if count < 2:
        raise CameraError("need at least 2 frames to measure a rate")
    stamps = [camera.grab().timestamp for _ in range(count)]
    intervals = [
        (later - earlier) * 1000.0 for earlier, later in itertools.pairwise(stamps)
    ]
    elapsed = stamps[-1] - stamps[0]
    return RateStats(
        frames=count,
        elapsed=elapsed,
        fps=(count - 1) / elapsed if elapsed > 0 else float("inf"),
        interval_mean_ms=statistics.fmean(intervals),
        interval_min_ms=min(intervals),
        interval_max_ms=max(intervals),
        interval_stdev_ms=statistics.stdev(intervals) if len(intervals) > 1 else 0.0,
    )


# --- CLI --------------------------------------------------------------------


def cmd_list(args: argparse.Namespace) -> int:
    with V4L2Device(args.device) as device:
        found = list(device.iter_controls())
        print(f"{args.device}: {len(found)} controls\n")
        width = max((len(info.name) for info in found), default=8)
        print(
            f"{'control':<{width}}  {'type':<7}  {'now':>7}  range / flags  driver name"
        )
        for info in found:
            print(
                f"{info.name:<{width}}  {info.type_name:<7}  "
                f"{device.get_cid(info.cid):>7}  "
                f"{info.range_text} [{info.flags_text}]  {info.driver_name!r}"
            )
        absent = sorted(set(CONTROLS) - {info.name for info in found})
        if absent:
            print(f"\nnot exposed by this device: {', '.join(absent)}")
    return 0


def cmd_dump(args: argparse.Namespace) -> int:
    config = (
        CaptureConfig.load(args.config) if args.config.exists() else CaptureConfig()
    )
    config.device = args.device
    with V4L2Device(args.device) as device:
        controls: dict[str, int | None] = {}
        for name in CONTROLS:
            try:
                device.query(name)
            except ControlUnsupported:
                continue
            controls[name] = device.get(name)
    config.controls = controls
    config.save(args.config)
    print(f"wrote {len(controls)} current control values to {args.config}")
    print("review the auto_* switches before locking (docs/camera/PARAM_LOCK.md §6)")
    return 0


def cmd_lock(args: argparse.Namespace) -> int:
    config = _load(args)
    with Camera(config) as camera:
        readback = camera.lock_params()
        print(f"{config.device}: negotiated {camera.negotiated_format()}")
        print(f"locked {len(readback)} controls, all read back as written")
    return 0


def cmd_verify(args: argparse.Namespace) -> int:
    config = _load(args)
    expected = config.locked_controls()
    with V4L2Device(config.device) as device:
        checks = compare(device, expected)
    print(
        f"{config.device}: verifying {len(expected)} controls against {args.config}\n"
    )
    return 1 if report(checks) else 0


def cmd_selftest(args: argparse.Namespace) -> int:
    config = _load(args)
    expected = config.locked_controls()

    print(f"== 1. open and lock ({config.device}) ==")
    camera = Camera(config)
    camera.open()
    try:
        camera.lock_params()
        print(f"negotiated {camera.negotiated_format()}")
        print(f"locked {len(expected)} controls\n")
        print(f"== 2. burst {args.frames} frames ==")
        print(measure_rate(camera, args.frames).describe(config.fps) + "\n")
    finally:
        camera.close()

    print("== 3. reopen the device and read every parameter back ==")
    with V4L2Device(config.device) as device:
        checks = compare(device, expected)
    differences = report(checks)
    print(
        "\nnote: step 3 reopens the device without power-cycling it. For the real"
        "\nboot-time check run `verify` after a reboot or a replug -- control values"
        "\nlive in the camera and return to firmware defaults when it loses power."
    )
    return 1 if differences else 0


def _load(args: argparse.Namespace) -> CaptureConfig:
    config = CaptureConfig.load(args.config)
    if args.device is not None:
        config.device = args.device
    return config


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    commands = (
        ("list", cmd_list, "list every control the device exposes", DEFAULT_DEVICE),
        (
            "dump",
            cmd_dump,
            "write the device's current control values into the config",
            DEFAULT_DEVICE,
        ),
        (
            "lock",
            cmd_lock,
            "apply the config's control set and verify the read-back",
            None,
        ),
        (
            "verify",
            cmd_verify,
            "compare the device against the config, item by item",
            None,
        ),
        (
            "selftest",
            cmd_selftest,
            "lock, measure the real frame rate, re-read params",
            None,
        ),
    )
    for name, handler, help_text, device_default in commands:
        sp = sub.add_parser(name, help=help_text)
        sp.set_defaults(handler=handler)
        sp.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
        sp.add_argument(
            "--device",
            default=device_default,
            help="V4L2 node; defaults to the config's `device` where applicable",
        )
        if name == "selftest":
            sp.add_argument("--frames", type=int, default=100)

    args = parser.parse_args(argv)
    try:
        return int(args.handler(args))
    except CameraError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
