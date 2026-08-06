import { memo } from "react";
import type { MotorChannel } from "./tauri";

// Line-art anchor-dot face, matching tools/face_visualizer.py's FaceRenderer
// (commit c99f338): every servo is a colored anchor point placed at its
// anatomical position, and the facial lines (brows, jaw outline, cheeks,
// nose) are smooth curves drawn through those anchors -- no filled face
// shape, so a channel's motion reads as a deformation of the lines
// themselves, not a texture swap. Proportions (round eye sockets, a
// prominent central nose ridge, wide-set eyes) are tuned against a photo of
// the actual printed skull + servo rig, not guessed from nothing.
//
// This only has the wire-protocol `applied` degrees + each channel's own
// min/max/neutral (like the Python renderer does) -- no access to Rust's
// internal norm state -- so it reconstructs the same bipolar deviation
// locally via Channel.deviation(), mirroring face_visualizer.py exactly.
// Direction conventions (brow raised, lid closed, corner up, jaw open) are
// the same unconfirmed assumptions as that Python renderer.

const VIEW_WIDTH = 900;
const VIEW_HEIGHT = 720;
const FACE_CENTER: readonly [number, number] = [450, 380];

const LINE_COLOR = "#d7e2f0";
const OUTLINE_COLOR = "#96a4ba";
const EYE_WHITE = "#eef2f8";
const PUPIL_COLOR = "#12121e";
const LIP_COLOR = "#d8787e";
const MOUTH_INNER = "#241218";

type ChannelGroup = "brow" | "tendon" | "eye" | "mouth" | "jaw" | "neck";

const GROUP_COLOR: Record<ChannelGroup, string> = {
  brow: "#f97316",
  tendon: "#ef4444",
  eye: "#22c55e",
  mouth: "#38bdf8",
  jaw: "#a78bfa",
  neck: "#facc15",
};

function channelGroup(channel: number): ChannelGroup {
  if (channel < 4) return "brow";
  if (channel < 8) return "tendon";
  if (channel < 14) return "eye";
  if (channel < 24) return "mouth";
  if (channel < 30) return "jaw";
  return "neck";
}

const MAX_TILT_DEG = 12.0;
const JAW_OPEN_RANGE_PX = 60.0;
const JAW_SHIFT_RANGE_PX = 14.0;
const EYE_GAZE_RANGE_PX = 12.0;
// Eyes tuned larger/rounder than a stylized cartoon face -- the real rig's
// eye sockets are big, round, and set well apart (see reference photo).
const EYE_HALF_WIDTH = 40.0;
const EYE_APERTURE_UPPER = 19.0;
const EYE_APERTURE_LOWER = 15.0;
const CORNER_VERTICAL_RANGE_PX = 22.0;
const CORNER_HORIZONTAL_RANGE_PX = 18.0;

function clamp(value: number, lower: number, upper: number): number {
  return Math.max(lower, Math.min(upper, value));
}

// Flat [0, 1] position within a channel's own limits -- screen-placement
// bookkeeping only, unrelated to control.rs's bipolar norm space (this
// component never sees that; it only has wire-protocol applied degrees).
function norm01(channel: MotorChannel, applied: number): number {
  const span = channel.maxApplied - channel.minApplied;
  if (span <= 1e-6) return 0.5;
  return clamp((applied - channel.minApplied) / span, 0, 1);
}

/** Bipolar position around neutral: -1 = minApplied, 0 = neutral,
 * +1 = maxApplied, each side scaled by its own span. */
function deviation(channel: MotorChannel, applied: number): number {
  const n = norm01(channel, applied);
  const n0 = norm01(channel, channel.neutralApplied);
  const span = n >= n0 ? 1.0 - n0 : n0;
  if (span <= 1e-6) return 0;
  return clamp((n - n0) / span, -1, 1);
}

type Point = readonly [number, number];

/** Interpolates a smooth curve through all control points (endpoints
 * included) so the face lines bend, not kink, around a displaced anchor. */
function catmullRom(points: Point[], samples = 12): Point[] {
  if (points.length < 3) return points;
  const pts = [points[0], ...points, points[points.length - 1]];
  const out: Point[] = [];
  for (let i = 0; i < pts.length - 3; i++) {
    const [p0, p1, p2, p3] = [pts[i], pts[i + 1], pts[i + 2], pts[i + 3]];
    for (let j = 0; j < samples; j++) {
      const t = j / samples;
      const t2 = t * t;
      const t3 = t2 * t;
      out.push([
        0.5 *
          (2 * p1[0] +
            (-p0[0] + p2[0]) * t +
            (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2 +
            (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3),
        0.5 *
          (2 * p1[1] +
            (-p0[1] + p2[1]) * t +
            (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2 +
            (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3),
      ]);
    }
  }
  out.push(points[points.length - 1]);
  return out;
}

function rotate(x: number, y: number, angleRad: number): Point {
  const cos = Math.cos(angleRad);
  const sin = Math.sin(angleRad);
  return [x * cos - y * sin, x * sin + y * cos];
}

function pointsToSvg(points: Point[]): string {
  return points.map(([x, y]) => `${x.toFixed(1)},${y.toFixed(1)}`).join(" ");
}

export type FacePose = {
  headArc: Point[];
  chinCurve: Point[];
  neckLines: Array<{ x1: number; y1: number; x2: number; y2: number }>;
  browCurves: Point[][];
  noseBridge: [Point, Point];
  noseBase: Point[];
  cheekCurves: Point[][];
  eyes: Array<
    | { open: true; aperture: Point[]; pupil: Point; radius: number }
    | { open: false; line: [Point, Point] }
  >;
  mouthInterior: Point[];
  upperLipCurve: Point[];
  lowerLipCurve: Point[];
  nodes: Record<number, Point>;
  // Channels 26/27 (jaw_right_lower, jaw_left) drive front-back jaw
  // protrusion -- a depth-axis motion a straight-on 2D view can't actually
  // show. Rather than fake a shape deformation that isn't real, these get
  // a dedicated abstract stand-in (dot radius) instead of participating in
  // `nodes`/the jaw outline. Revisit once a 3/4-view render exists.
  protrusionNodes: Array<{ channel: number; point: Point; scale: number }>;
};

/** Pure geometry computation, kept separate from the SVG markup below so it
 * can be unit-tested without a browser or React renderer. Ported from
 * tools/face_visualizer.py's FaceRenderer -- same anchor layout, same
 * curves, same channel groups -- just fed `currentApplied` directly instead
 * of reconstructing it from a UDP wire frame. */
export function computeFacePose(channels: MotorChannel[], applied: number[]): FacePose | null {
  if (channels.length < 32 || applied.length < 32) {
    return null;
  }

  const nodes: Record<number, Point> = {};
  const dev = (id: number) => deviation(channels[id], applied[id]);

  const tiltRad = tiltRadiansFromDelta(dev(30) - dev(31));

  const toScreen = (x: number, y: number): Point => {
    const [rx, ry] = rotate(x, y, tiltRad);
    return [FACE_CENTER[0] + rx, FACE_CENTER[1] + ry];
  };
  const fixed = (x: number, y: number): Point => [FACE_CENTER[0] + x, FACE_CENTER[1] + y];

  // Channel 25 (jaw_right_upper) is the primary jaw-open axis; direction of
  // "open" vs "closed" is unconfirmed on real hardware, so distance from
  // neutral in either direction counts as open.
  const jawOpen = Math.abs(dev(25)) * JAW_OPEN_RANGE_PX;
  const jawX = dev(24) * JAW_SHIFT_RANGE_PX;

  // --- Neck (screen space, not tilted -- the head tilts on top of it) ---
  const neckLines: FacePose["neckLines"] = [];
  for (const [side, channel] of [
    [1, 30],
    [-1, 31],
  ] as const) {
    const [x1, y1] = fixed(side * 72, 200);
    const [x2, y2] = fixed(side * 95, 292);
    neckLines.push({ x1, y1, x2, y2 });
    nodes[channel] = fixed(side * 84, 248 - 14.0 * dev(channel));
  }

  // --- Head outline: forehead arc + jaw outline through 26/27 ---
  const headArcLocal: Point[] = [];
  const t0 = 0.393;
  const t1 = -(Math.PI + 0.393);
  for (let i = 0; i < 49; i++) {
    const t = t0 + (t1 - t0) * (i / 48);
    headArcLocal.push([175.0 * Math.cos(t), -10.0 + 235.0 * Math.sin(t)]);
  }
  const headArc = headArcLocal.map(([x, y]) => toScreen(x, y));

  // Chin outline is driven only by channel 25 (open amount) and channel 24
  // (left-right shear) -- 26/27 are front-back (protrusion), a depth-axis
  // motion that doesn't deform this frontal silhouette; see protrusionNodes.
  const chin: Point[] = [
    [-160.0, 80.0],
    [-95.0 + jawX, 170.0 + 0.7 * jawOpen],
    [jawX, 195.0 + jawOpen],
    [95.0 + jawX, 170.0 + 0.7 * jawOpen],
    [160.0, 80.0],
  ];
  const chinCurve = catmullRom(chin).map(([x, y]) => toScreen(x, y));
  nodes[24] = toScreen(jawX, 168.0 + 0.9 * jawOpen);
  nodes[25] = toScreen(-85.0, 145.0 + jawOpen);
  const protrusionNodes: FacePose["protrusionNodes"] = [
    { channel: 26, point: toScreen(...chin[1]), scale: 1 + dev(26) * 0.6 },
    { channel: 27, point: toScreen(...chin[3]), scale: 1 + dev(27) * 0.6 },
  ];
  // Tongue channels (28/29) aren't rendered at all -- no visual anchor,
  // per hardware review (they don't drive anything a 2D face preview shows).

  // --- Brows: subject-right (0/1) on screen-left, subject-left (2/3) on
  // screen-right, outer ends sitting higher. ---
  const browCurves = ([[-1, 0, 1] as const, [1, 2, 3] as const]).map(([side, innerCh, outerCh]) => {
    const ix = side * 70.0;
    const iy = -168.0 - 26.0 * dev(innerCh);
    const ox = side * 130.0;
    const oy = -180.0 - 22.0 * dev(outerCh);
    const mid: Point = [(ix + ox) / 2.0, (iy + oy) / 2.0 - 5.0];
    nodes[innerCh] = toScreen(ix, iy);
    nodes[outerCh] = toScreen(ox, oy);
    return catmullRom([[ix, iy], mid, [ox, oy]]).map(([x, y]) => toScreen(x, y));
  });

  // --- Eyes: side=-1 -> screen-left eye = subject's right (11/12). ---
  const gazeX = dev(8) * EYE_GAZE_RANGE_PX;
  const gazeY = -dev(13) * EYE_GAZE_RANGE_PX; // + = look up

  const eyes: FacePose["eyes"] = ([[-1, 11, 12] as const, [1, 9, 10] as const]).map(([side, upperCh, lowerCh]) => {
    const cx = side * 80.0;
    const cy = -110.0;
    const hw = EYE_HALF_WIDTH;
    const apU = clamp(EYE_APERTURE_UPPER * (1.0 - dev(upperCh)), 0.0, 30.0);
    const apL = clamp(EYE_APERTURE_LOWER * (1.0 - dev(lowerCh)), 0.0, 22.0);

    const upperEdge: Point[] = [];
    const lowerEdge: Point[] = [];
    for (let i = 0; i < 17; i++) {
      const s = -1.0 + i / 8.0;
      const bulge = 1.0 - s * s;
      upperEdge.push([cx + s * hw, cy - apU * bulge]);
      lowerEdge.push([cx + s * hw, cy + apL * bulge]);
    }

    nodes[upperCh] = toScreen(cx, cy - apU);
    nodes[lowerCh] = toScreen(cx, cy + apL);

    if (apU + apL > 4.0) {
      const aperture = [...upperEdge, ...lowerEdge.slice().reverse()].map(([x, y]) => toScreen(x, y));
      const px = cx + clamp(gazeX, -(hw - 14.0), hw - 14.0);
      const py = cy + clamp(gazeY, -apU * 0.5, apL * 0.5);
      const radius = Math.min(11.0, (apU + apL) * 0.42);
      return { open: true as const, aperture, pupil: toScreen(px, py), radius };
    }
    return { open: false as const, line: [toScreen(cx - hw, cy), toScreen(cx + hw, cy)] as [Point, Point] };
  });

  nodes[8] = toScreen(gazeX, -142.0);
  nodes[13] = toScreen(0.0, -118.0 + gazeY);

  // --- Nose: prominent central ridge (reference photo shows a raised,
  // fairly pointed bridge, not a flat one) down to a rounded tip base. ---
  const noseBridge: [Point, Point] = [toScreen(0, -108), toScreen(0, -42)];
  const liftL = -8.0 * dev(5); // subject left = +x
  const liftR = -8.0 * dev(6);
  const noseBase = catmullRom([
    [-26.0, -38.0 + liftR],
    [0.0, -26.0],
    [26.0, -38.0 + liftL],
  ]).map(([x, y]) => toScreen(x, y));
  nodes[5] = toScreen(96.0, -80.0 - 14.0 * dev(5));
  nodes[6] = toScreen(-96.0, -80.0 - 14.0 * dev(6));

  // --- Cheeks: tendon pulls its arc up and outward (smile apple / squint);
  // anchor sits roughly where the real rig's cheek tendon guide is. ---
  const cheekCurves = ([[1, 4] as const, [-1, 7] as const]).map(([side, channel]) => {
    const d = dev(channel);
    const ax = side * (150.0 + 6.0 * d);
    const ay = -40.0 - 18.0 * d;
    nodes[channel] = toScreen(ax, ay);
    return catmullRom([
      [side * 122.0, -95.0],
      [ax, ay],
      [side * 140.0, 30.0],
    ]).map(([x, y]) => toScreen(x, y));
  });

  // --- Mouth ---
  const upLift = -0.12 * jawOpen;
  const lowDrop = 0.85 * jawOpen;
  const pt = (channel: number, x: number, y: number, moveY: number, shiftX = 0): Point => {
    const p: Point = [x + shiftX, y + moveY * dev(channel)];
    nodes[channel] = toScreen(...p);
    return p;
  };

  // Each mouth corner is one physical point driven by TWO motors through a
  // linkage (only the two motor pivots move; the corner is a coupler
  // point): same-direction rotation moves the corner up/down, opposite
  // rotation moves it in/out. Horizontal direction sign is an assumption,
  // unconfirmed on hardware -- same caveat as jaw_open's direction above.
  const mouthCorner = (upperCh: number, lowerCh: number, side: -1 | 1): Point => {
    const vertical = ((dev(upperCh) + dev(lowerCh)) / 2) * CORNER_VERTICAL_RANGE_PX;
    const horizontal = (dev(upperCh) - dev(lowerCh)) * CORNER_HORIZONTAL_RANGE_PX;
    const p: Point = [side * 106.0 + side * horizontal, 56.0 + (upLift + lowDrop) / 2 + vertical];
    nodes[upperCh] = toScreen(...p);
    nodes[lowerCh] = toScreen(...p);
    return p;
  };

  const rightCorner = mouthCorner(17, 18, -1);
  const leftCorner = mouthCorner(19, 20, 1);

  const upperLip: Point[] = [
    rightCorner,
    pt(16, -52.0, 46.0 + upLift, -14.0),
    pt(15, 0.0, 44.0 + upLift, -12.0),
    pt(14, 52.0, 46.0 + upLift, -14.0),
    leftCorner,
  ];
  const lowerLip: Point[] = [
    rightCorner,
    pt(22, -52.0, 58.0 + lowDrop, 16.0, jawX),
    pt(23, 0.0, 62.0 + lowDrop, 18.0, jawX),
    pt(21, 52.0, 58.0 + lowDrop, 16.0, jawX),
    leftCorner,
  ];

  const mouthInterior = [...catmullRom(upperLip), ...catmullRom(lowerLip.slice().reverse())].map(([x, y]) =>
    toScreen(x, y),
  );
  const upperLipCurve = catmullRom(upperLip).map(([x, y]) => toScreen(x, y));
  const lowerLipCurve = catmullRom(lowerLip).map(([x, y]) => toScreen(x, y));

  return {
    headArc,
    chinCurve,
    neckLines,
    browCurves,
    noseBridge,
    noseBase,
    cheekCurves,
    eyes,
    mouthInterior,
    upperLipCurve,
    lowerLipCurve,
    nodes,
    protrusionNodes,
  };
}

function tiltRadiansFromDelta(rawDelta: number): number {
  const deg = clamp(rawDelta * MAX_TILT_DEG, -MAX_TILT_DEG, MAX_TILT_DEG);
  return (deg * Math.PI) / 180;
}

type FacePreviewProps = {
  channels: MotorChannel[];
  applied: number[];
};

export const FacePreview = memo(function FacePreview({ channels, applied }: FacePreviewProps) {
  const pose = computeFacePose(channels, applied);
  if (!pose) {
    return <svg className="face-preview" viewBox={`0 0 ${VIEW_WIDTH} ${VIEW_HEIGHT}`} />;
  }

  return (
    <svg className="face-preview" viewBox={`0 0 ${VIEW_WIDTH} ${VIEW_HEIGHT}`}>
      {pose.neckLines.map((line, i) => (
        <line key={`neck-${i}`} {...line} stroke={OUTLINE_COLOR} strokeWidth={4} />
      ))}

      <polyline points={pointsToSvg(pose.headArc)} fill="none" stroke={OUTLINE_COLOR} strokeWidth={3} />
      <polyline points={pointsToSvg(pose.chinCurve)} fill="none" stroke={OUTLINE_COLOR} strokeWidth={3} />

      {pose.browCurves.map((curve, i) => (
        <polyline key={`brow-${i}`} points={pointsToSvg(curve)} fill="none" stroke={LINE_COLOR} strokeWidth={5} />
      ))}

      <line
        x1={pose.noseBridge[0][0]}
        y1={pose.noseBridge[0][1]}
        x2={pose.noseBridge[1][0]}
        y2={pose.noseBridge[1][1]}
        stroke={OUTLINE_COLOR}
        strokeWidth={2.5}
      />
      <polyline points={pointsToSvg(pose.noseBase)} fill="none" stroke={LINE_COLOR} strokeWidth={3} />

      {pose.cheekCurves.map((curve, i) => (
        <polyline key={`cheek-${i}`} points={pointsToSvg(curve)} fill="none" stroke={OUTLINE_COLOR} strokeWidth={3} />
      ))}

      {pose.eyes.map((eye, i) =>
        eye.open ? (
          <g key={`eye-${i}`}>
            <polygon points={pointsToSvg(eye.aperture)} fill={EYE_WHITE} stroke={LINE_COLOR} strokeWidth={2} />
            <circle cx={eye.pupil[0]} cy={eye.pupil[1]} r={eye.radius} fill={PUPIL_COLOR} />
          </g>
        ) : (
          <line
            key={`eye-${i}`}
            x1={eye.line[0][0]}
            y1={eye.line[0][1]}
            x2={eye.line[1][0]}
            y2={eye.line[1][1]}
            stroke={LINE_COLOR}
            strokeWidth={3}
          />
        ),
      )}

      <polygon points={pointsToSvg(pose.mouthInterior)} fill={MOUTH_INNER} />
      <polyline points={pointsToSvg(pose.upperLipCurve)} fill="none" stroke={LIP_COLOR} strokeWidth={4} />
      <polyline points={pointsToSvg(pose.lowerLipCurve)} fill="none" stroke={LIP_COLOR} strokeWidth={4} />

      {Object.entries(pose.nodes).map(([channelStr, [x, y]]) => {
        const channel = Number(channelStr);
        return <circle key={channel} cx={x} cy={y} r={4.5} fill={GROUP_COLOR[channelGroup(channel)]} />;
      })}

      {/* Abstract stand-in for front-back jaw protrusion (26/27): a 2D
          frontal view can't show depth, so these read via dot size instead
          of a (fake) shape change. Revisit if/when a 3/4-view render exists. */}
      {pose.protrusionNodes.map(({ channel, point: [x, y], scale }) => (
        <circle
          key={channel}
          cx={x}
          cy={y}
          r={4.5 * Math.max(0.4, scale)}
          fill={GROUP_COLOR.jaw}
          opacity={0.85}
        />
      ))}
    </svg>
  );
});
