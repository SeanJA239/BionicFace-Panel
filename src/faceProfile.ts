// Side-view inset geometry for the twin face. Line-for-line twin of
// tools/face_profile.py -- the two must be edited together, same as the main
// renderers. See that file's docstring for the design rationale (depth-axis
// motions get a profile view where they are in-plane) and the direction
// caveats (protrusion and pitch signs are unconfirmed on hardware).

export type ProfilePoint = readonly [number, number];

export type ProfileGeometry = {
  skull: ProfilePoint[];
  upperTeeth: ProfilePoint[];
  jaw: ProfilePoint[];
  lowerTeeth: ProfilePoint[];
  toothSeparators: Array<readonly [ProfilePoint, ProfilePoint]>;
  neck: Array<readonly [ProfilePoint, ProfilePoint]>;
};

export const INSET_WIDTH = 150;
export const INSET_HEIGHT = 190;
// Neck pivot inside the inset box; all pitch rotation happens around it.
const PIVOT: ProfilePoint = [66, 156];

// Amplitude constants, hand-written like the main view's *_RANGE_PX family --
// replace with measured linkage ratios when those exist.
const PROFILE_JAW_OPEN_RANGE_RAD = 0.42;
const PROFILE_PROTRUDE_RANGE_PX = 12.0;
const PROFILE_PITCH_RANGE_RAD = 0.3;

// Jaw hinge sits at the ear, relative to the pivot.
const JAW_HINGE: ProfilePoint = [-18, -78];

// Skull outline, pivot-relative, nose pointing +x (observer's right): up the
// back of the head, over the crown, down the face to the maxilla underside.
const SKULL: ProfilePoint[] = [
  [-30, -6],
  [-47, -55],
  [-40, -100],
  [-14, -133],
  [16, -140],
  [38, -122],
  [46, -100],
  [42, -88],
  [48, -80],
  [62, -66],
  [46, -60],
  [48, -50],
  [44, -42],
  [12, -42],
];

// Tooth rows meet at y=-34 when the jaw is closed. The upper row hangs from
// the maxilla and only pitches; the lower row rides the jaw.
const UPPER_TEETH: ProfilePoint[] = [
  [12, -42],
  [42, -42],
  [42, -34],
  [12, -34],
];
const LOWER_TEETH: ProfilePoint[] = [
  [12, -34],
  [40, -34],
  [40, -26],
  [12, -26],
];
const TOOTH_SEPARATORS_X = [20, 28, 36];

// Jaw outline: hinge, down around the jaw angle, forward to the chin, up to
// where the lower teeth mount.
const JAW: ProfilePoint[] = [
  [-18, -78],
  [-16, -40],
  [-6, -22],
  [30, -18],
  [46, -28],
  [42, -40],
];

// Neck stays fixed: the head pitches on top of it, matching the main view's
// treatment of tilt.
const NECK: Array<readonly [ProfilePoint, ProfilePoint]> = [
  [
    [-24, -2],
    [-32, 28],
  ],
  [
    [10, -2],
    [16, 28],
  ],
];

/** Catmull-Rom through every control point, endpoints duplicated -- the same
 * scheme the main renderers use, duplicated here so the twin modules stay
 * self-contained (importing it back from the renderer would be a cycle).
 * Teeth quads and separators stay angular on purpose: they are teeth. */
function smooth(points: ProfilePoint[], samples = 8): ProfilePoint[] {
  if (points.length < 3) return [...points];
  const pts = [points[0], ...points, points[points.length - 1]];
  const out: ProfilePoint[] = [points[0]];
  for (let i = 0; i < pts.length - 3; i++) {
    const [p0, p1, p2, p3] = [pts[i], pts[i + 1], pts[i + 2], pts[i + 3]];
    for (let step = 1; step <= samples; step++) {
      const s = step / samples;
      const s2 = s * s;
      const s3 = s2 * s;
      out.push([
        0.5 *
          (2 * p1[0] +
            (-p0[0] + p2[0]) * s +
            (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * s2 +
            (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * s3),
        0.5 *
          (2 * p1[1] +
            (-p0[1] + p2[1]) * s +
            (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * s2 +
            (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * s3),
      ]);
    }
  }
  return out;
}

function rotate(point: ProfilePoint, angleRad: number): ProfilePoint {
  const c = Math.cos(angleRad);
  const s = Math.sin(angleRad);
  return [point[0] * c - point[1] * s, point[0] * s + point[1] * c];
}

function clamp(value: number, lower: number, upper: number): number {
  return Math.max(lower, Math.min(upper, value));
}

/** Placed inset geometry for one frame. open01 in [0,1]; protrude01/pitch01
 * in [-1,1] (ch26's one-sided travel means protrude01 is non-positive in
 * practice). `origin` is the inset box's top-left corner in view
 * coordinates. */
export function profileGeometry(
  open01: number,
  protrude01: number,
  pitch01: number,
  origin: ProfilePoint,
): ProfileGeometry {
  const openRad = clamp(open01, 0, 1) * PROFILE_JAW_OPEN_RANGE_RAD;
  const protrudePx = clamp(protrude01, -1, 1) * PROFILE_PROTRUDE_RANGE_PX;
  const pitchRad = clamp(pitch01, -1, 1) * PROFILE_PITCH_RANGE_RAD;

  const placeHead = (point: ProfilePoint): ProfilePoint => {
    const [x, y] = rotate(point, pitchRad);
    return [origin[0] + PIVOT[0] + x, origin[1] + PIVOT[1] + y];
  };
  const placeJaw = (point: ProfilePoint): ProfilePoint => {
    const local: ProfilePoint = [point[0] - JAW_HINGE[0], point[1] - JAW_HINGE[1]];
    const [x, y] = rotate(local, openRad);
    return placeHead([x + JAW_HINGE[0] + protrudePx, y + JAW_HINGE[1]]);
  };
  const placeNeck = (point: ProfilePoint): ProfilePoint => [
    origin[0] + PIVOT[0] + point[0],
    origin[1] + PIVOT[1] + point[1],
  ];

  const separators = (
    quad: ProfilePoint[],
    place: (p: ProfilePoint) => ProfilePoint,
  ): Array<readonly [ProfilePoint, ProfilePoint]> =>
    TOOTH_SEPARATORS_X.map((x) => [place([x, quad[0][1]]), place([x, quad[2][1]])] as const);

  return {
    skull: smooth(SKULL.map(placeHead)),
    upperTeeth: UPPER_TEETH.map(placeHead),
    jaw: smooth(JAW.map(placeJaw)),
    lowerTeeth: LOWER_TEETH.map(placeJaw),
    toothSeparators: [...separators(UPPER_TEETH, placeHead), ...separators(LOWER_TEETH, placeJaw)],
    neck: NECK.map(([a, b]) => [placeNeck(a), placeNeck(b)] as const),
  };
}
