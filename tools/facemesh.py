"""Shared FaceLandmarker plumbing for the camera-verification tools.

`tools/latency_check.py` and `tools/noise_floor.py` both need the same three
things: a resolved model path, a FaceLandmarker running in IMAGE mode, and the
canonical face-region index sets. They live here so neither tool re-implements
them.

IMAGE mode is deliberate. VIDEO/LIVE_STREAM mode applies temporal filtering
across frames, which would smooth away exactly the per-frame jitter that
`noise_floor.py` exists to measure -- it would report a flatteringly low noise
floor that no longer describes what a single-shot capture actually gives you.
Every detect() call here is independent.

Region index sets are derived at runtime from mediapipe's own
`FaceLandmarksConnections` tables rather than hardcoded, so they cannot drift
from whatever model file is loaded.
"""

from __future__ import annotations

import types
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from collections.abc import Mapping

DEFAULT_MODEL = Path(__file__).resolve().with_name("face_landmarker.task")

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/face_landmarker/"
    "face_landmarker/float16/latest/face_landmarker.task"
)

# Region name -> the FaceLandmarksConnections constants unioned to form it.
_REGION_SOURCES: dict[str, tuple[str, ...]] = {
    "eyebrows": ("FACE_LANDMARKS_LEFT_EYEBROW", "FACE_LANDMARKS_RIGHT_EYEBROW"),
    "eyes": ("FACE_LANDMARKS_LEFT_EYE", "FACE_LANDMARKS_RIGHT_EYE"),
    "lips": ("FACE_LANDMARKS_LIPS",),
    "nose": ("FACE_LANDMARKS_NOSE",),
    "face_oval": ("FACE_LANDMARKS_FACE_OVAL",),
    "iris": ("FACE_LANDMARKS_LEFT_IRIS", "FACE_LANDMARKS_RIGHT_IRIS"),
}

# The 468-point face mesh, i.e. every landmark the tesselation covers. The
# model also emits iris points (469..477); those are tracked as their own
# region because they move with gaze rather than with the face surface.
_MESH_SOURCE = "FACE_LANDMARKS_TESSELATION"

_regions_cache: Mapping[str, frozenset[int]] | None = None
_mesh_cache: frozenset[int] | None = None


def _connection_indices(source_names: tuple[str, ...]) -> frozenset[int]:
    from mediapipe.tasks.python.vision.face_landmarker import FaceLandmarksConnections

    indices: set[int] = set()
    for name in source_names:
        for connection in getattr(FaceLandmarksConnections, name):
            indices.add(connection.start)
            indices.add(connection.end)
    return frozenset(indices)


def regions() -> Mapping[str, frozenset[int]]:
    """Canonical face-region landmark index sets, keyed by region name."""
    global _regions_cache
    if _regions_cache is None:
        _regions_cache = types.MappingProxyType(
            {
                name: _connection_indices(sources)
                for name, sources in _REGION_SOURCES.items()
            }
        )
    return _regions_cache


def mesh_indices() -> frozenset[int]:
    """The 468 landmarks covered by the face tesselation."""
    global _mesh_cache
    if _mesh_cache is None:
        _mesh_cache = _connection_indices((_MESH_SOURCE,))
    return _mesh_cache


class LandmarkExtractor:
    """FaceLandmarker in IMAGE mode: BGR frame -> (N, 2) pixel landmarks."""

    def __init__(self, model: Path = DEFAULT_MODEL, num_faces: int = 1) -> None:
        if not model.exists():
            raise FileNotFoundError(
                f"FaceLandmarker model not found: {model}\n"
                f"Download it with:\n  curl -L -o {model} {MODEL_URL}"
            )
        # Heavy imports stay inside the constructor, matching the style in
        # mediapipe_driver.py, so this module imports on a machine without
        # mediapipe/opencv installed.
        import cv2
        import mediapipe as mp

        self._cv2 = cv2
        self._mp = mp
        self._landmarker = mp.tasks.vision.FaceLandmarker.create_from_options(
            mp.tasks.vision.FaceLandmarkerOptions(
                base_options=mp.tasks.BaseOptions(model_asset_path=str(model)),
                running_mode=mp.tasks.vision.RunningMode.IMAGE,
                num_faces=num_faces,
                output_face_blendshapes=False,
                output_facial_transformation_matrixes=False,
            )
        )

    def __enter__(self) -> LandmarkExtractor:  # noqa: PYI034  # typing.Self needs 3.11
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def detect(self, frame_bgr: np.ndarray) -> np.ndarray | None:
        """Returns (N, 2) float32 pixel coordinates, or None if no face."""
        height, width = frame_bgr.shape[:2]
        rgb = self._cv2.cvtColor(frame_bgr, self._cv2.COLOR_BGR2RGB)
        result = self._landmarker.detect(
            self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb)
        )
        if not result.face_landmarks:
            return None
        face = result.face_landmarks[0]
        points = np.empty((len(face), 2), dtype=np.float32)
        for i, landmark in enumerate(face):
            points[i, 0] = landmark.x * width
            points[i, 1] = landmark.y * height
        return points

    def close(self) -> None:
        """Closes the graph explicitly.

        Leaving it to __del__ raises a TypeError from mediapipe's own teardown
        once interpreter shutdown has cleared its module globals.
        """
        self._landmarker.close()
