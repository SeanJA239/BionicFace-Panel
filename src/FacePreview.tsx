import { memo } from "react";
import type { MotorChannel } from "./tauri";

// Ported from tools/face_visualizer.py so the panel and the standalone
// pygame mock render the exact same pose from the exact same 32 applied
// angles + channel limits -- no second copy of the mapping "rules", just a
// second renderer of the same math. Keep the two in sync if either changes.

const FACE_CENTER: readonly [number, number] = [450, 380];
const VIEW_WIDTH = 900;
const VIEW_HEIGHT = 720;

const MAX_TILT_DEG = 12;
const BROW_RANGE_PX = 32;
const EYE_GAZE_RANGE_PX = 10;
const EYE_RADIUS = 34;
const MOUTH_HALF_WIDTH = 130;
const MOUTH_LIP_RANGE_PX = 22;
const MOUTH_BASE_Y = 130;
const JAW_OPEN_RANGE_PX = 60;

const FACE_COLOR = "#ebcdb4";
const LINE_COLOR = "#281e19";
const EYE_WHITE = "#fafafa";
const PUPIL_COLOR = "#141419";
const MOUTH_COLOR = "#96323c";

function clamp(value: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, value));
}

// Flat [0, 1] position within a channel's own limits -- unrelated to (and
// simpler than) control.rs's bipolar neutral-anchored norm space; this is
// screen-placement bookkeeping only, matching face_visualizer.py's Channel.norm01.
function norm01(channel: MotorChannel, applied: number): number {
  const span = channel.maxApplied - channel.minApplied;
  if (span <= 1e-6) return 0.5;
  return clamp((applied - channel.minApplied) / span, 0, 1);
}

function neutralNorm01(channel: MotorChannel): number {
  return norm01(channel, channel.neutralApplied);
}

function rotate(x: number, y: number, angleRad: number): [number, number] {
  const cos = Math.cos(angleRad);
  const sin = Math.sin(angleRad);
  return [x * cos - y * sin, x * sin + y * cos];
}

export type BrowGeometry = { key: -1 | 1; x1: number; y1: number; x2: number; y2: number };
export type EyeGeometry = {
  key: -1 | 1;
  centerX: number;
  centerY: number;
  pupilX: number;
  pupilY: number;
  upperHeight: number;
  lowerHeight: number;
};

export type FacePose = {
  tiltDeg: number;
  jawOpenPx: number;
  brows: BrowGeometry[];
  eyes: EyeGeometry[];
  mouthPoints: Array<[number, number]>;
};

/** Pure geometry computation, kept separate from the SVG markup below so it
 * can be unit-tested (and cross-checked against tools/face_visualizer.py's
 * per-preset tilt_deg/jaw_open_px) without a browser or React renderer. */
export function computeFacePose(channels: MotorChannel[], applied: number[]): FacePose | null {
  if (channels.length < 32 || applied.length < 32) {
    return null;
  }

  const norm = (id: number) => norm01(channels[id], applied[id]);
  const neutral = (id: number) => neutralNorm01(channels[id]);

  // Neck tilt (30/31): visual approximation, not a physical calibration.
  const tiltDeg = clamp((norm(30) - neutral(30) - (norm(31) - neutral(31))) * MAX_TILT_DEG, -MAX_TILT_DEG, MAX_TILT_DEG);
  const tiltRad = (tiltDeg * Math.PI) / 180;

  const toScreen = (x: number, y: number): [number, number] => {
    const [rx, ry] = rotate(x, y, tiltRad);
    return [FACE_CENTER[0] + rx, FACE_CENTER[1] + ry];
  };

  // Jaw open amount: direction of "open" vs "closed" is unconfirmed against
  // real hardware, so this uses distance from neutral in either direction.
  const jawDeviation = Math.abs(norm(25) - neutral(25));
  const jawOpenPx = clamp(jawDeviation * 2, 0, 1) * JAW_OPEN_RANGE_PX;

  // Screen-left holds the character's anatomical right brow (0/1), mirrored,
  // as is conventional for a front-facing face.
  const brows: BrowGeometry[] = ([-1, 1] as const).map((side) => {
    const [innerCh, outerCh] = side === -1 ? [0, 1] : [2, 3];
    const innerY = -140 + (norm(innerCh) - neutral(innerCh)) * -BROW_RANGE_PX;
    const outerY = -132 + (norm(outerCh) - neutral(outerCh)) * -BROW_RANGE_PX;
    const [x1, y1] = toScreen(side * 45, innerY);
    const [x2, y2] = toScreen(side * 115, outerY);
    return { key: side, x1, y1, x2, y2 };
  });

  const gazeX = (norm(8) - neutral(8)) * EYE_GAZE_RANGE_PX;
  const gazeY = (norm(13) - neutral(13)) * EYE_GAZE_RANGE_PX;
  const eyes: EyeGeometry[] = ([-1, 1] as const).map((side) => {
    // side=-1 -> screen-left eye (anatomical right: channels 11/12),
    // side=+1 -> screen-right eye (anatomical left: channels 9/10).
    const [upperCh, lowerCh] = side === -1 ? [11, 12] : [9, 10];
    const cx = side * 80;
    const cy = -40;
    const [centerX, centerY] = toScreen(cx, cy);
    const [pupilX, pupilY] = toScreen(cx + gazeX, cy + gazeY);
    return {
      key: side,
      centerX,
      centerY,
      pupilX,
      pupilY,
      upperHeight: norm(upperCh) * EYE_RADIUS * 2,
      lowerHeight: norm(lowerCh) * EYE_RADIUS * 2,
    };
  });

  const lipY = (channelId: number, baseline: number, jawShare: number) =>
    baseline + (norm(channelId) - neutral(channelId)) * MOUTH_LIP_RANGE_PX + jawOpenPx * jawShare;

  const mw = MOUTH_HALF_WIDTH;
  const mouthPoints: Array<[number, number]> = [
    [-mw, lipY(19, MOUTH_BASE_Y, -0.15)], // mouth_left_corner_upper
    [-mw * 0.45, lipY(14, MOUTH_BASE_Y, -0.15)], // upper_lip_left
    [0, lipY(15, MOUTH_BASE_Y, -0.15)], // upper_lip_mid
    [mw * 0.45, lipY(16, MOUTH_BASE_Y, -0.15)], // upper_lip_right
    [mw, lipY(17, MOUTH_BASE_Y, -0.15)], // mouth_right_corner_upper
    [mw, lipY(18, MOUTH_BASE_Y + 10, 0.85)], // mouth_right_corner_lower
    [mw * 0.45, lipY(22, MOUTH_BASE_Y + 10, 0.85)], // lower_lip_right
    [0, lipY(23, MOUTH_BASE_Y + 10, 0.85)], // lower_lip_mid_tendon
    [-mw * 0.45, lipY(21, MOUTH_BASE_Y + 10, 0.85)], // lower_lip_left
    [-mw, lipY(20, MOUTH_BASE_Y + 10, 0.85)], // mouth_left_corner_lower
  ].map(([x, y]) => toScreen(x, y));

  return { tiltDeg, jawOpenPx, brows, eyes, mouthPoints };
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

  const mouthPoints = pose.mouthPoints.map(([x, y]) => `${x},${y}`).join(" ");

  return (
    <svg className="face-preview" viewBox={`0 0 ${VIEW_WIDTH} ${VIEW_HEIGHT}`}>
      <ellipse cx={FACE_CENTER[0]} cy={FACE_CENTER[1]} rx={120} ry={150} fill={FACE_COLOR} stroke={LINE_COLOR} strokeWidth={3} />

      {pose.brows.map((brow) => (
        <line key={brow.key} x1={brow.x1} y1={brow.y1} x2={brow.x2} y2={brow.y2} stroke={LINE_COLOR} strokeWidth={6} strokeLinecap="round" />
      ))}

      {pose.eyes.map((eye) => (
        <g key={eye.key}>
          <circle cx={eye.centerX} cy={eye.centerY} r={EYE_RADIUS} fill={EYE_WHITE} />
          <circle cx={eye.pupilX} cy={eye.pupilY} r={EYE_RADIUS / 2} fill={PUPIL_COLOR} />
          <rect x={eye.centerX - EYE_RADIUS} y={eye.centerY - EYE_RADIUS} width={EYE_RADIUS * 2} height={eye.upperHeight} fill={FACE_COLOR} />
          <rect
            x={eye.centerX - EYE_RADIUS}
            y={eye.centerY + EYE_RADIUS - eye.lowerHeight}
            width={EYE_RADIUS * 2}
            height={eye.lowerHeight}
            fill={FACE_COLOR}
          />
          <circle cx={eye.centerX} cy={eye.centerY} r={EYE_RADIUS} fill="none" stroke={LINE_COLOR} strokeWidth={2} />
        </g>
      ))}

      <polygon points={mouthPoints} fill={MOUTH_COLOR} stroke={LINE_COLOR} strokeWidth={3} />
    </svg>
  );
});
