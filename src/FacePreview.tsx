import { memo } from "react";
import { INSET_HEIGHT, INSET_WIDTH, profileGeometry, type ProfileGeometry } from "./faceProfile";
import photoAnchorsFile from "../docs/hardware/face_anchors.json";
import type { MotorChannel } from "./tauri";

// Layered line-art face, matching tools/face_visualizer.py's FaceRenderer
// (hardware review 2026-08-29, photo: docs/hardware/face_frontal.jpg):
// a static skull with fixed UPPER teeth, eyes whose gaze channels (8/13) move
// the pupils rather than dots, a rigid jaw carrying the LOWER teeth (open from
// 25, lateral from 24; 26/27 are depth-axis and appear only as a numeric
// readout), and an independent lip ring drawn in front of the teeth -- so
// "lips parted over a closed jaw" shows closed teeth behind open lips instead
// of looking like an open jaw. Anchor dots are drawn only for channels that
// are real surface actuation points (DOT_CHANNELS); their rest positions come
// from docs/hardware/face_anchors.json once the jog sweep fills
// channel_mapping, and fall back to the hand-written constants until then.
//
// This only has the wire-protocol `applied` degrees + each channel's own
// min/max/neutral (like the Python renderer does) -- no access to Rust's
// internal norm state -- so it derives its own displacement from neutral,
// scaled by degrees of travel; see renderScales.
// Direction conventions (brow raised, lid closed, corner up, jaw open) are
// the same assumptions as that Python renderer, and still unconfirmed except
// for the mirroring below.

const VIEW_WIDTH = 900;
const VIEW_HEIGHT = 720;
const FACE_CENTER: readonly [number, number] = [450, 380];

// The subject's-right servos are mounted mirrored relative to the left ones, so
// a symmetric expression arrives with opposite norm signs on the two sides.
// Read off the hardware-authored emotion presets: across the five that should
// be left/right symmetric (喜悦/悲伤/愤怒/惊讶/恐惧 -- 困惑 and wink are
// deliberately asymmetric), paired channels carry opposite signs in 26 of 29
// cases where both sides move. Channel 11 is independently confirmed by
// control.rs's idle-blink direction table, and no pair has its mirroring
// already encoded in minApplied/maxApplied -- every range is increasing.
//
// Mirroring is a property of the whole right-side bank, not of individual
// channels, so this set starts from every right-side channel that has a
// left-side mirror partner. Deciding it per channel by voting on preset signs
// is what produced an earlier half-corrected set (0, 1, 11, 17, 18) that
// rendered symmetric presets with the brows and mouth corners fixed while the
// cheeks, nose, lower lids and lips stayed flipped.
//
// Channel 22 (lower_lip_right) is the one right-side partner deliberately left
// OUT. 悲伤 and 愤怒 share an identical mouth block for channels 17-22, so they
// are one authored observation, not two -- and once deduplicated the lower-lip
// pair is a 1-1 tie. 恐惧 breaks it: 22=+1.00 with 21=+0.74, both large and
// same-signed, and fear pulls the lower lip down on both sides, so negating 22
// renders that preset with the two halves of the lower lip moving apart.
// Every other pair here is unanimous.
//
// Deliberately excluded, for structural reasons rather than lack of evidence:
// midline channels with no mirror partner (8, 13, 15, 23, 24); the jaw, whose
// open axis enters through Math.abs and whose 26/27 pair is coupled in Rust;
// and the neck, which enters as the difference dev(30) - dev(31), a form that
// already handles a mirrored pair (tilt is antisymmetric either way, so the
// presets cannot tell us about neck mounting).
const MIRRORED_CHANNELS: ReadonlySet<number> = new Set([
  0, 1, 6, 7, 11, 12, 16, 17, 18,
]);

/** Canonical subject-right <-> subject-left pairing of the paired facial
 * features. MIRRORED_CHANNELS is a subset of its right-hand members, and
 * renderScales shares one reference across each pair. */
const MIRROR_PAIRS: ReadonlyArray<readonly [number, number]> = [
  [0, 2],
  [1, 3],
  [6, 5],
  [7, 4],
  [11, 9],
  [12, 10],
  [16, 14],
  [17, 19],
  [18, 20],
  [22, 21],
];

/** Degrees of travel each channel's drawn displacement is divided by.
 *
 * control.rs normalises each side of neutral by that side's own travel. Drawing
 * in those units makes a channel with 5deg of travel produce the same excursion
 * as one with 50, so a physically symmetric pose renders lopsided.
 * 悲伤 is the clear case: channel 18 at norm -1.00 and channel 20 at norm +0.50
 * are both 20deg of real motion, and once 18's mirrored mount is accounted for
 * they are the same face motion -- yet they differ 2x in norm, because 18 has
 * 20deg of upward travel where 20 has 40. That drew the two lower-mouth corners
 * 10px apart; against a shared reference they land on the same line.
 *
 * Unpaired channels (midline, jaw, neck) keep their own largest travel, so
 * their amplitude is unchanged. */
function renderScales(channels: MotorChannel[]): number[] {
  const own = channels.map((c) =>
    Math.max(c.neutralApplied - c.minApplied, c.maxApplied - c.neutralApplied),
  );
  const scales = [...own];
  for (const [right, left] of MIRROR_PAIRS) {
    const shared = Math.max(own[right], own[left]);
    scales[right] = shared;
    scales[left] = shared;
  }
  return scales.map((s) => (s > 1e-6 ? s : 1));
}

const LINE_COLOR = "#d7e2f0";
const OUTLINE_COLOR = "#96a4ba";
const EYE_WHITE = "#eef2f8";
const PUPIL_COLOR = "#12121e";
const LIP_COLOR = "#d8787e";
const MOUTH_INNER = "#241218";
const TOOTH_COLOR = "#e0e2db";
const TOOTH_EDGE = "#949894";
const HUD_TEXT = "#c8dce6";
const INSET_FRAME = "#69707c";

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
const EYE_CENTER_X = 80.0;
const EYE_CENTER_Y = -110.0;
const CORNER_VERTICAL_RANGE_PX = 22.0;
const CORNER_HORIZONTAL_RANGE_PX = 18.0;

// Channels that are actual surface actuation points and therefore get an
// anchor dot: brow/cheek/nose tendons, eyelids, and the lip ring. Everything
// else has a dedicated representation -- 8/13 move the pupils, 24/25 move the
// rigid jaw, 26/27 are depth-axis (numeric readout only), 28/29 are disabled
// tongue channels, 30/31 tilt the whole head.
const DOT_CHANNELS: ReadonlySet<number> = new Set([
  ...Array.from({ length: 8 }, (_, i) => i),
  9, 10, 11, 12,
  ...Array.from({ length: 10 }, (_, i) => 14 + i),
]);

// Teeth geometry in face-local units. The upper row is part of the skull
// (fixed); the lower row rides the jaw. At rest the rows meet at OCCLUSION_Y;
// jaw open drops the lower band, exposing the dark mouth interior between.
const TEETH_HALF_WIDTH = 78.0;
const UPPER_TEETH_TOP = 46.0;
const OCCLUSION_Y = 60.0;
const LOWER_TEETH_BOTTOM = 74.0;
const TOOTH_PITCH = 13.0;
const TEETH_SAMPLE_STEP = 4.0;

function clamp(value: number, lower: number, upper: number): number {
  return Math.max(lower, Math.min(upper, value));
}

// Flat [0, 1] position within a channel's own limits -- screen-placement
// bookkeeping only, unrelated to control.rs's bipolar norm space (this
// component never sees that; it only has wire-protocol applied degrees).
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

type PhotoAnchorsFile = {
  image_size: [number, number];
  red_candidates: Record<string, [number, number]>;
  pupils: Array<[number, number]>;
  // Ships as a TODO string until the jog sweep fills it with
  // {channel_id: red_candidate_index}.
  channel_mapping: unknown;
};

/** Face-local rest positions measured on the hardware photo, per channel.
 *
 * Photo coordinates (normalized, origin top-left) map into face-local space
 * through a similarity transform anchored on the two pupils. Photo and twin
 * face are both observer-view (+x = screen right = subject's left), so no
 * mirror flip; the photo is assumed upright, so no roll term. Empty whenever
 * channel_mapping isn't a filled dict yet -- every channel then stays on the
 * hand-written fallback constants. Same algorithm as face_visualizer.py's
 * load_photo_anchors. */
function loadPhotoAnchors(
  raw: PhotoAnchorsFile,
  eyeLeftLocal: Point,
  eyeRightLocal: Point,
): Map<number, Point> {
  const anchors = new Map<number, Point>();
  const mapping = raw.channel_mapping;
  if (typeof mapping !== "object" || mapping === null || Array.isArray(mapping)) {
    return anchors;
  }
  const [width, height] = raw.image_size;
  const pts = raw.pupils.map(([nx, ny]): Point => [nx * width, ny * height]);
  if (pts.length < 2) return anchors;
  // The dot detector can pick up stray blue marks; the true pupils are the
  // pair sitting at nearly the same height.
  let best: [Point, Point] | null = null;
  for (let i = 0; i < pts.length; i++) {
    for (let j = i + 1; j < pts.length; j++) {
      if (!best || Math.abs(pts[i][1] - pts[j][1]) < Math.abs(best[0][1] - best[1][1])) {
        best = [pts[i], pts[j]];
      }
    }
  }
  if (!best) return anchors;
  const [photoLeft, photoRight] = best[0][0] <= best[1][0] ? best : [best[1], best[0]];
  const span = Math.hypot(photoRight[0] - photoLeft[0], photoRight[1] - photoLeft[1]);
  if (span < 1e-6) return anchors;
  const scale = (eyeRightLocal[0] - eyeLeftLocal[0]) / span;
  const midPhoto: Point = [
    (photoLeft[0] + photoRight[0]) / 2,
    (photoLeft[1] + photoRight[1]) / 2,
  ];
  const midLocal: Point = [
    (eyeLeftLocal[0] + eyeRightLocal[0]) / 2,
    (eyeLeftLocal[1] + eyeRightLocal[1]) / 2,
  ];
  for (const [channelStr, candidate] of Object.entries(mapping as Record<string, unknown>)) {
    const pos = raw.red_candidates[String(candidate)];
    if (!pos) continue;
    anchors.set(Number(channelStr), [
      midLocal[0] + (pos[0] * width - midPhoto[0]) * scale,
      midLocal[1] + (pos[1] * height - midPhoto[1]) * scale,
    ]);
  }
  return anchors;
}

const PHOTO_ANCHORS = loadPhotoAnchors(
  photoAnchorsFile as unknown as PhotoAnchorsFile,
  [-EYE_CENTER_X, EYE_CENTER_Y],
  [EYE_CENTER_X, EYE_CENTER_Y],
);

/** Linear-interpolated y of a sampled curve at x; null outside its span.
 * The lip curves run right corner (-x) to left corner (+x); Catmull-Rom can
 * wiggle near the ends, so the first crossing wins. */
function curveYAt(points: Point[], x: number): number | null {
  for (let i = 0; i + 1 < points.length; i++) {
    const [x0, y0] = points[i];
    const [x1, y1] = points[i + 1];
    if ((x0 - x) * (x1 - x) <= 0 && Math.abs(x1 - x0) > 1e-9) {
      return y0 + ((x - x0) / (x1 - x0)) * (y1 - y0);
    }
  }
  return null;
}

/** Visible tooth-band polygons, clipped per x-sample to the lip opening.
 * Geometric rather than raster clipping so pygame and SVG share one
 * algorithm; `xShift` rides the lower band on the jaw's lateral shift.
 * All coordinates face-local. */
function teethStrips(
  bandTop: number,
  bandBottom: number,
  upperLip: Point[],
  lowerLip: Point[],
  xShift = 0,
): { strips: Point[][]; separators: Array<[Point, Point]> } {
  const strips: Point[][] = [];
  const separators: Array<[Point, Point]> = [];

  const visibleInterval = (fx: number): [number, number] | null => {
    const yu = curveYAt(upperLip, fx);
    const yl = curveYAt(lowerLip, fx);
    if (yu === null || yl === null) return null;
    const top = Math.max(bandTop, yu);
    const bottom = Math.min(bandBottom, yl);
    return top < bottom - 0.5 ? [top, bottom] : null;
  };

  let topRun: Point[] = [];
  let bottomRun: Point[] = [];
  const closeRun = () => {
    if (topRun.length >= 2) strips.push([...topRun, ...bottomRun.slice().reverse()]);
    topRun = [];
    bottomRun = [];
  };
  const steps = Math.floor((2 * TEETH_HALF_WIDTH) / TEETH_SAMPLE_STEP);
  for (let i = 0; i <= steps; i++) {
    const fx = -TEETH_HALF_WIDTH + i * TEETH_SAMPLE_STEP + xShift;
    const interval = visibleInterval(fx);
    if (!interval) {
      closeRun();
    } else {
      topRun.push([fx, interval[0]]);
      bottomRun.push([fx, interval[1]]);
    }
  }
  closeRun();

  const boundaries = Math.floor((2 * TEETH_HALF_WIDTH) / TOOTH_PITCH);
  for (let k = 1; k <= boundaries; k++) {
    const fx = -TEETH_HALF_WIDTH + k * TOOTH_PITCH + xShift;
    const interval = visibleInterval(fx);
    if (interval) separators.push([[fx, interval[0]], [fx, interval[1]]]);
  }
  return { strips, separators };
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
  // Tooth bands (screen coords), clipped to the lip opening: upper row is
  // skull-fixed, lower row rides the rigid jaw.
  teethPolygons: Point[][];
  teethSeparators: Array<[Point, Point]>;
  upperLipCurve: Point[];
  lowerLipCurve: Point[];
  nodes: Record<number, Point>;
  // Channels 26/27 drive front-back jaw protrusion -- a depth-axis motion a
  // straight-on 2D view can't show. Deliberately not drawn as a (fake) shape
  // change; this readout is their whole representation.
  depthReadout: string;
  profile: ProfileGeometry;
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
  const scales = renderScales(channels);
  // Displacement from neutral in units of the channel's render scale, so paired
  // channels with unequal travel draw equal physical motion as equal
  // displacement. Mirrored channels are flipped here, at the one place
  // displacements enter the renderer, so everything below keeps reading
  // "+ = raised / closed / up".
  const dev = (id: number) => {
    const value = clamp((applied[id] - channels[id].neutralApplied) / scales[id], -1, 1);
    return MIRRORED_CHANNELS.has(id) ? -value : value;
  };
  // Rest position of a channel's feature point: the photo-measured anchor when
  // the jog sweep has mapped it, the hand-written constant otherwise. Motion
  // offsets are applied on top either way.
  const rest = (id: number, fallback: Point): Point => PHOTO_ANCHORS.get(id) ?? fallback;

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

  // Side-view inset for the depth-axis motions (see faceProfile.ts). Corner
  // choice: the only HUD text is the depth readout at bottom-left and the
  // face stays within x 270..630, so top-right is quiet.
  const profile = profileGeometry(
    Math.abs(dev(25)),
    dev(26),
    (dev(30) + dev(31)) / 2,
    [VIEW_WIDTH - INSET_WIDTH - 12, 12],
  );

  // --- Neck (screen space, not tilted -- the head tilts on top of it) ---
  const neckLines: FacePose["neckLines"] = [];
  // 30/31 get no dots: their whole representation is the head tilt.
  for (const side of [1, -1] as const) {
    const [x1, y1] = fixed(side * 72, 200);
    const [x2, y2] = fixed(side * 95, 292);
    neckLines.push({ x1, y1, x2, y2 });
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
  // motion that doesn't deform this frontal silhouette; see depthReadout.
  const chin: Point[] = [
    [-160.0, 80.0],
    [-95.0 + jawX, 170.0 + 0.7 * jawOpen],
    [jawX, 195.0 + jawOpen],
    [95.0 + jawX, 170.0 + 0.7 * jawOpen],
    [160.0, 80.0],
  ];
  const chinCurve = catmullRom(chin).map(([x, y]) => toScreen(x, y));
  // The jaw group (24-27) gets no anchor dots: the outline and the lower
  // teeth ARE its in-plane representation, and 26/27 are depth-axis (see
  // depthReadout). Tongue channels (28/29) are disabled and not rendered.
  const depthReadout =
    `jaw open ${Math.abs(dev(25)).toFixed(2)}  shift ${dev(24) >= 0 ? "+" : ""}${dev(24).toFixed(2)}` +
    `  depth 26:${dev(26) >= 0 ? "+" : ""}${dev(26).toFixed(2)} 27:${dev(27) >= 0 ? "+" : ""}${dev(27).toFixed(2)}`;

  // --- Brows: subject-right (0/1) on screen-left, subject-left (2/3) on
  // screen-right, outer ends sitting higher. ---
  const browCurves = ([[-1, 0, 1] as const, [1, 2, 3] as const]).map(([side, innerCh, outerCh]) => {
    const [rix, riy] = rest(innerCh, [side * 70.0, -168.0]);
    const [rox, roy] = rest(outerCh, [side * 130.0, -180.0]);
    const ix = rix;
    const iy = riy - 26.0 * dev(innerCh);
    const ox = rox;
    const oy = roy - 22.0 * dev(outerCh);
    const mid: Point = [(ix + ox) / 2.0, (iy + oy) / 2.0 - 5.0];
    nodes[innerCh] = toScreen(ix, iy);
    nodes[outerCh] = toScreen(ox, oy);
    return catmullRom([[ix, iy], mid, [ox, oy]]).map(([x, y]) => toScreen(x, y));
  });

  // --- Eyes: side=-1 -> screen-left eye = subject's right (11/12). ---
  const gazeX = dev(8) * EYE_GAZE_RANGE_PX;
  const gazeY = -dev(13) * EYE_GAZE_RANGE_PX; // + = look up

  const eyes: FacePose["eyes"] = ([[-1, 11, 12] as const, [1, 9, 10] as const]).map(([side, upperCh, lowerCh]) => {
    const cx = side * EYE_CENTER_X;
    const cy = EYE_CENTER_Y;
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

    const [rux, ruy] = rest(upperCh, [cx, cy - EYE_APERTURE_UPPER]);
    const [rlx, rly] = rest(lowerCh, [cx, cy + EYE_APERTURE_LOWER]);
    nodes[upperCh] = toScreen(rux, ruy + (EYE_APERTURE_UPPER - apU));
    nodes[lowerCh] = toScreen(rlx, rly - (EYE_APERTURE_LOWER - apL));

    if (apU + apL > 4.0) {
      const aperture = [...upperEdge, ...lowerEdge.slice().reverse()].map(([x, y]) => toScreen(x, y));
      const px = cx + clamp(gazeX, -(hw - 14.0), hw - 14.0);
      const py = cy + clamp(gazeY, -apU * 0.5, apL * 0.5);
      const radius = Math.min(11.0, (apU + apL) * 0.42);
      return { open: true as const, aperture, pupil: toScreen(px, py), radius };
    }
    return { open: false as const, line: [toScreen(cx - hw, cy), toScreen(cx + hw, cy)] as [Point, Point] };
  });

  // 8/13 get no dots: they are rotations of the shared eye mechanism, not
  // surface points -- the pupils are their whole representation.

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
  const [r5x, r5y] = rest(5, [96.0, -80.0]);
  const [r6x, r6y] = rest(6, [-96.0, -80.0]);
  nodes[5] = toScreen(r5x, r5y - 14.0 * dev(5));
  nodes[6] = toScreen(r6x, r6y - 14.0 * dev(6));

  // --- Cheeks: tendon pulls its arc up and outward (smile apple / squint);
  // anchor sits roughly where the real rig's cheek tendon guide is. ---
  const cheekCurves = ([[1, 4] as const, [-1, 7] as const]).map(([side, channel]) => {
    const d = dev(channel);
    const [rx, ry] = rest(channel, [side * 150.0, -40.0]);
    const ax = rx + side * 6.0 * d;
    const ay = ry - 18.0 * d;
    nodes[channel] = toScreen(ax, ay);
    return catmullRom([
      [side * 122.0, -95.0],
      [ax, ay],
      [side * 140.0, 30.0],
    ]).map(([x, y]) => toScreen(x, y));
  });

  // --- Mouth. The lip ring is its own layer but still rides the jaw a
  // little: with skin on, the lower lip sits on the jaw. The teeth do not
  // follow the lip channels at all -- that separation is what makes "lips
  // parted over a closed jaw" (closed teeth behind open lips) distinguishable
  // from an open jaw (a dark gap between the tooth rows). ---
  const upLift = -0.12 * jawOpen;
  const lowDrop = 0.85 * jawOpen;
  const pt = (channel: number, x: number, y: number, moveY: number, shiftX = 0, followY = 0): Point => {
    const [rx, ry] = rest(channel, [x, y]);
    const p: Point = [rx + shiftX, ry + followY + moveY * dev(channel)];
    nodes[channel] = toScreen(...p);
    return p;
  };

  // Each mouth corner is ONE physical point driven by two motors through a
  // linkage (only the motor pivots move; the corner is a coupler point).
  //
  // The two channels' documented positive directions are geometrically
  // opposite -- +upper raises the corner, +lower pulls the lip down -- so their
  // DIFFERENCE is the up/down common mode and their SUM the in/out
  // differential. Checked against the presets: the difference puts 喜悦's
  // corners up and 悲伤/愤怒/恐惧's down, left and right within 0.05 of each
  // other; the sum gets every one of those backwards. The horizontal direction
  // sign is still an assumption, unconfirmed on hardware -- same caveat as
  // jaw_open's direction above.
  const mouthCorner = (upperCh: number, lowerCh: number, side: -1 | 1): Point => {
    const lift = ((dev(upperCh) - dev(lowerCh)) / 2) * CORNER_VERTICAL_RANGE_PX;
    const outward = ((dev(upperCh) + dev(lowerCh)) / 2) * CORNER_HORIZONTAL_RANGE_PX;
    // The pair shares one physical corner, so either channel's mapped anchor
    // (upper wins) positions the rest point.
    const [rx, ry] = rest(upperCh, rest(lowerCh, [side * 106.0, 56.0]));
    const p: Point = [rx + side * outward, ry + (upLift + lowDrop) / 2 - lift];
    nodes[upperCh] = toScreen(...p);
    nodes[lowerCh] = toScreen(...p);
    return p;
  };

  const rightCorner = mouthCorner(17, 18, -1);
  const leftCorner = mouthCorner(19, 20, 1);

  const upperLip: Point[] = [
    rightCorner,
    pt(16, -52.0, 46.0, -14.0, 0, upLift),
    pt(15, 0.0, 44.0, -12.0, 0, upLift),
    pt(14, 52.0, 46.0, -14.0, 0, upLift),
    leftCorner,
  ];
  const lowerLip: Point[] = [
    rightCorner,
    pt(22, -52.0, 58.0, 16.0, jawX, lowDrop),
    pt(23, 0.0, 62.0, 18.0, jawX, lowDrop),
    pt(21, 52.0, 58.0, 16.0, jawX, lowDrop),
    leftCorner,
  ];

  const upperPts = catmullRom(upperLip);
  const lowerPts = catmullRom(lowerLip);
  const mouthInterior = [...upperPts, ...lowerPts.slice().reverse()].map(([x, y]) => toScreen(x, y));
  const upperLipCurve = upperPts.map(([x, y]) => toScreen(x, y));
  const lowerLipCurve = lowerPts.map(([x, y]) => toScreen(x, y));

  // Teeth, clipped to the lip opening. Upper row skull-fixed; lower row rides
  // the rigid jaw (full open drop + lateral shift).
  const upper = teethStrips(UPPER_TEETH_TOP, OCCLUSION_Y, upperPts, lowerPts);
  const lower = teethStrips(
    OCCLUSION_Y + jawOpen,
    LOWER_TEETH_BOTTOM + jawOpen,
    upperPts,
    lowerPts,
    jawX,
  );
  const teethPolygons = [...upper.strips, ...lower.strips].map((strip) =>
    strip.map(([x, y]) => toScreen(x, y)),
  );
  const teethSeparators = [...upper.separators, ...lower.separators].map(
    ([a, b]): [Point, Point] => [toScreen(...a), toScreen(...b)],
  );

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
    teethPolygons,
    teethSeparators,
    upperLipCurve,
    lowerLipCurve,
    nodes,
    depthReadout,
    profile,
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
      {pose.teethPolygons.map((strip, i) => (
        <polygon
          key={`teeth-${i}`}
          points={pointsToSvg(strip)}
          fill={TOOTH_COLOR}
          stroke={TOOTH_EDGE}
          strokeWidth={1}
        />
      ))}
      {pose.teethSeparators.map(([a, b], i) => (
        <line key={`tooth-sep-${i}`} x1={a[0]} y1={a[1]} x2={b[0]} y2={b[1]} stroke={TOOTH_EDGE} strokeWidth={1} />
      ))}
      <polyline points={pointsToSvg(pose.upperLipCurve)} fill="none" stroke={LIP_COLOR} strokeWidth={4} />
      <polyline points={pointsToSvg(pose.lowerLipCurve)} fill="none" stroke={LIP_COLOR} strokeWidth={4} />

      {Object.entries(pose.nodes).map(([channelStr, [x, y]]) => {
        const channel = Number(channelStr);
        if (!DOT_CHANNELS.has(channel)) return null;
        return <circle key={channel} cx={x} cy={y} r={4.5} fill={GROUP_COLOR[channelGroup(channel)]} />;
      })}

      <text x={12} y={VIEW_HEIGHT - 14} fill={HUD_TEXT} fontSize={15} fontFamily="monospace">
        {pose.depthReadout}
      </text>

      <g>
        <rect
          x={VIEW_WIDTH - INSET_WIDTH - 12}
          y={12}
          width={INSET_WIDTH}
          height={INSET_HEIGHT}
          fill="none"
          stroke={INSET_FRAME}
          strokeWidth={1}
        />
        <text
          x={VIEW_WIDTH - INSET_WIDTH - 7}
          y={26}
          fill={HUD_TEXT}
          fontSize={12}
          fontFamily="monospace"
          fontWeight="bold"
        >
          SIDE
        </text>
        {pose.profile.neck.map(([a, b], i) => (
          <line key={`p-neck-${i}`} x1={a[0]} y1={a[1]} x2={b[0]} y2={b[1]} stroke={OUTLINE_COLOR} strokeWidth={2} />
        ))}
        <polyline points={pointsToSvg(pose.profile.skull)} fill="none" stroke={OUTLINE_COLOR} strokeWidth={2} />
        <polyline points={pointsToSvg(pose.profile.jaw)} fill="none" stroke={OUTLINE_COLOR} strokeWidth={2} />
        <polygon points={pointsToSvg(pose.profile.upperTeeth)} fill={TOOTH_COLOR} stroke={TOOTH_EDGE} strokeWidth={1} />
        <polygon points={pointsToSvg(pose.profile.lowerTeeth)} fill={TOOTH_COLOR} stroke={TOOTH_EDGE} strokeWidth={1} />
        {pose.profile.toothSeparators.map(([a, b], i) => (
          <line key={`p-sep-${i}`} x1={a[0]} y1={a[1]} x2={b[0]} y2={b[1]} stroke={TOOTH_EDGE} strokeWidth={1} />
        ))}
      </g>
    </svg>
  );
});
