"""Side-view inset geometry for the twin-face renderers.

The frontal view can only carry second-order cues for depth-axis motion, so
the motions that are perpendicular (or mostly perpendicular) to the image
plane get a small profile inset where they are in-plane and directly
drawable:

  * jaw open (ch25)      -- hinge rotation at the ear; also shown frontally,
                            repeated here on purpose so the same rigid body
                            reads consistently across the two views
  * jaw protrusion (ch26; ch27 is the coupled follower) -- horizontal slide,
                            previously a HUD number only and this inset's
                            main reason to exist
  * head pitch (nod)     -- ch30/31 common mode, the frontal view's tilt uses
                            only their differential so pitch had no visual at
                            all until now

Pure geometry: callers pass normalized deviations and get named point lists
in screen coordinates. Kept in its own module so `face_visualizer.py` stays
under the size limit; `src/faceProfile.ts` is the line-for-line TSX twin and
the two must be edited together, same as the main renderers.

Direction caveats, same standing as the main view's unverified signs:
protrusion sign (does lower applied on ch26 move the jaw forward or back?)
and pitch sign (is positive common mode nod-down?) are both unconfirmed on
hardware; flip `PROFILE_PROTRUDE_RANGE_PX` / `PROFILE_PITCH_RANGE_RAD` signs
when the jog sweep settles them.
"""

from __future__ import annotations

import math

Point = tuple[float, float]

INSET_WIDTH = 150.0
INSET_HEIGHT = 190.0
# Neck pivot inside the inset box; all pitch rotation happens around it.
_PIVOT = (66.0, 156.0)

# Amplitude constants, hand-written like the main view's *_RANGE_PX family --
# replace with measured linkage ratios when those exist.
PROFILE_JAW_OPEN_RANGE_RAD = 0.42
PROFILE_PROTRUDE_RANGE_PX = 12.0
PROFILE_PITCH_RANGE_RAD = 0.30

# Jaw hinge sits at the ear, relative to the pivot.
_JAW_HINGE = (-18.0, -78.0)

# Skull outline, pivot-relative, nose pointing +x (observer's right): up the
# back of the head, over the crown, down the face to the maxilla underside.
_SKULL: list[Point] = [
    (-30.0, -6.0),
    (-47.0, -55.0),
    (-40.0, -100.0),
    (-14.0, -133.0),
    (16.0, -140.0),
    (38.0, -122.0),
    (46.0, -100.0),
    (42.0, -88.0),
    (48.0, -80.0),
    (62.0, -66.0),
    (46.0, -60.0),
    (48.0, -50.0),
    (44.0, -42.0),
    (12.0, -42.0),
]

# Tooth rows meet at y=-34 when the jaw is closed. The upper row hangs from
# the maxilla and only pitches; the lower row rides the jaw.
_UPPER_TEETH: list[Point] = [(12.0, -42.0), (42.0, -42.0), (42.0, -34.0), (12.0, -34.0)]
_LOWER_TEETH: list[Point] = [(12.0, -34.0), (40.0, -34.0), (40.0, -26.0), (12.0, -26.0)]
_TOOTH_SEPARATORS_X = (20.0, 28.0, 36.0)

# Jaw outline: hinge, down around the jaw angle, forward to the chin, up to
# where the lower teeth mount.
_JAW: list[Point] = [
    (-18.0, -78.0),
    (-16.0, -40.0),
    (-6.0, -22.0),
    (30.0, -18.0),
    (46.0, -28.0),
    (42.0, -40.0),
]

# Neck stays fixed: the head pitches on top of it, matching the main view's
# treatment of tilt.
_NECK: list[tuple[Point, Point]] = [
    ((-24.0, -2.0), (-32.0, 28.0)),
    ((10.0, -2.0), (16.0, 28.0)),
]


def _rotate(point: Point, angle_rad: float) -> Point:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return (point[0] * c - point[1] * s, point[0] * s + point[1] * c)


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def profile_geometry(
    open01: float,
    protrude01: float,
    pitch01: float,
    origin: Point,
) -> dict[str, list]:
    """Placed inset geometry for one frame.

    open01 in [0,1]; protrude01/pitch01 in [-1,1] (ch26's one-sided travel
    means protrude01 is non-positive in practice). `origin` is the inset
    box's top-left corner in screen coordinates.

    Returns polylines ("skull", "jaw"), tooth quads ("upper_teeth",
    "lower_teeth"), and segment lists ("tooth_separators", "neck").
    """
    open_rad = _clamp(open01, 0.0, 1.0) * PROFILE_JAW_OPEN_RANGE_RAD
    protrude_px = _clamp(protrude01, -1.0, 1.0) * PROFILE_PROTRUDE_RANGE_PX
    pitch_rad = _clamp(pitch01, -1.0, 1.0) * PROFILE_PITCH_RANGE_RAD

    def place_head(point: Point) -> Point:
        x, y = _rotate(point, pitch_rad)
        return (origin[0] + _PIVOT[0] + x, origin[1] + _PIVOT[1] + y)

    def place_jaw(point: Point) -> Point:
        local = (point[0] - _JAW_HINGE[0], point[1] - _JAW_HINGE[1])
        x, y = _rotate(local, open_rad)
        return place_head((x + _JAW_HINGE[0] + protrude_px, y + _JAW_HINGE[1]))

    def place_neck(point: Point) -> Point:
        return (origin[0] + _PIVOT[0] + point[0], origin[1] + _PIVOT[1] + point[1])

    def separators(quad: list[Point], place) -> list[tuple[Point, Point]]:
        top_y, bottom_y = quad[0][1], quad[2][1]
        return [(place((x, top_y)), place((x, bottom_y))) for x in _TOOTH_SEPARATORS_X]

    return {
        "skull": [place_head(p) for p in _SKULL],
        "upper_teeth": [place_head(p) for p in _UPPER_TEETH],
        "jaw": [place_jaw(p) for p in _JAW],
        "lower_teeth": [place_jaw(p) for p in _LOWER_TEETH],
        "tooth_separators": separators(_UPPER_TEETH, place_head)
        + separators(_LOWER_TEETH, place_jaw),
        "neck": [(place_neck(a), place_neck(b)) for a, b in _NECK],
    }
