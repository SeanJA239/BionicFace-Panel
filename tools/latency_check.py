"""End-to-end camera -> FaceLandmarker latency and frame-rate measurement.

Board 2 step 1 of the capture-pipeline handbook: prove the camera -> locked
params -> FaceLandmarker link works, and measure what it actually costs.

Timing is split into the two stages that can each be the bottleneck: waiting on
a frame from the camera, and running inference on it. Both are reported as
p50/p95 rather than means, because a mean hides the stalls that matter for a
30Hz control loop.

Parameters are locked before the first measured frame -- an unlocked camera
re-runs auto-exposure as the scene changes, which moves both the frame interval
(longer exposure, fewer frames) and the inference cost.

Usage:
    python tools/latency_check.py [--config tools/camera_params.json]
                                  [--duration 30] [--warmup 2]
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from camera_capture import Camera, CameraError, CaptureConfig
from facemesh import DEFAULT_MODEL, LandmarkExtractor

# Below this share of frames the run says nothing about timing, only about framing.
UNUSABLE_DETECTION_RATE = 0.5
GOOD_DETECTION_RATE = 0.9


@dataclass(frozen=True)
class StageTimings:
    """Per-stage wall-clock costs for one measurement run, in milliseconds."""

    name: str
    samples: list[float]

    @property
    def p50(self) -> float:
        return float(np.percentile(self.samples, 50))

    @property
    def p95(self) -> float:
        return float(np.percentile(self.samples, 95))

    @property
    def mean(self) -> float:
        return statistics.fmean(self.samples)

    @property
    def worst(self) -> float:
        return max(self.samples)


def measure(
    camera: Camera, extractor: LandmarkExtractor, duration: float, warmup: float
) -> tuple[list[StageTimings], list[float], int]:
    """Runs the link for `duration` seconds after a `warmup` settling period.

    Returns (per-stage timings, frame arrival timestamps, detection count). The
    warmup exists because the first detect() call builds the mediapipe graph and
    costs an order of magnitude more than steady state.
    """
    grab_ms: list[float] = []
    infer_ms: list[float] = []
    total_ms: list[float] = []
    arrivals: list[float] = []
    detections = 0

    deadline = time.monotonic() + warmup + duration
    measure_from = time.monotonic() + warmup
    while time.monotonic() < deadline:
        start = time.perf_counter()
        frame = camera.grab()
        grabbed = time.perf_counter()
        landmarks = extractor.detect(frame.image)
        finished = time.perf_counter()
        if frame.timestamp < measure_from:
            continue
        grab_ms.append((grabbed - start) * 1000.0)
        infer_ms.append((finished - grabbed) * 1000.0)
        total_ms.append((finished - start) * 1000.0)
        arrivals.append(frame.timestamp)
        if landmarks is not None:
            detections += 1

    if not total_ms:
        raise CameraError(
            f"no frames measured; duration {duration}s is too short relative to "
            f"warmup {warmup}s"
        )
    stages = [
        StageTimings("grab", grab_ms),
        StageTimings("inference", infer_ms),
        StageTimings("end-to-end", total_ms),
    ]
    return stages, arrivals, detections


def report(
    stages: list[StageTimings],
    arrivals: list[float],
    detections: int,
    requested_fps: float,
) -> int:
    frames = len(arrivals)
    elapsed = arrivals[-1] - arrivals[0] if frames > 1 else 0.0
    achieved = (frames - 1) / elapsed if elapsed > 0 else float("nan")
    rate = detections / frames if frames else 0.0

    print(f"{'stage':<12}  {'p50':>8}  {'p95':>8}  {'mean':>8}  {'max':>8}   (ms)")
    for stage in stages:
        print(
            f"{stage.name:<12}  {stage.p50:>8.2f}  {stage.p95:>8.2f}  "
            f"{stage.mean:>8.2f}  {stage.worst:>8.2f}"
        )
    print(
        f"\n{frames} frames over {elapsed:.2f}s -> {achieved:.2f} fps achieved "
        f"(camera negotiated {requested_fps:g})"
    )
    print(f"face detected in {detections}/{frames} frames ({rate * 100:.1f}%)")

    # Compare *inference* against the frame budget, not end-to-end. grab()
    # blocks until the next frame is ready, so end-to-end is frame_interval +
    # inference by construction and would trip the check on any healthy setup.
    # What decides whether processing keeps up is inference alone.
    budget_ms = 1000.0 / requested_fps if requested_fps > 0 else float("inf")
    inference = next(stage for stage in stages if stage.name == "inference")
    if inference.p95 > budget_ms:
        print(
            f"\nwarning: inference p95 {inference.p95:.2f}ms exceeds the "
            f"{budget_ms:.2f}ms budget for {requested_fps:g} fps -- processing cannot "
            f"keep up with the stream, frames will queue or drop"
        )
    if detections == 0:
        print(
            "\nno face was detected in any frame. Check framing and lighting before "
            "trusting any timing above; see board 2 step 3 on camera placement.",
            file=sys.stderr,
        )
        return 1
    if rate < UNUSABLE_DETECTION_RATE:
        print(
            f"\ndetection rate {rate * 100:.1f}% is too low to measure anything on: "
            f"the face is probably small, clipped by the frame edge or badly lit. "
            f"Fix framing (board 2 step 3) before recording a noise floor.",
            file=sys.stderr,
        )
        return 1
    if rate < GOOD_DETECTION_RATE:
        print(
            f"\nwarning: detection rate {rate * 100:.1f}% -- usable but not solid; "
            f"expect near-100% once the face fills 60-80% of the frame"
        )
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=Path("tools/camera_params.json"))
    parser.add_argument("--device", default=None, help="overrides the config's device")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--duration", type=float, default=30.0, help="measured seconds")
    parser.add_argument("--warmup", type=float, default=2.0, help="discarded seconds")
    args = parser.parse_args(argv)

    try:
        config = CaptureConfig.load(args.config)
        if args.device is not None:
            config.device = args.device
        with Camera(config) as camera:
            locked = camera.lock_params()
            negotiated = camera.negotiated_format()
            print(f"{config.device}: {negotiated}, {len(locked)} controls locked")
            print(f"measuring {args.duration:g}s after {args.warmup:g}s warmup\n")
            with LandmarkExtractor(args.model) as extractor:
                stages, arrivals, detections = measure(
                    camera, extractor, args.duration, args.warmup
                )
        return report(stages, arrivals, detections, negotiated["fps"])
    except (CameraError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
