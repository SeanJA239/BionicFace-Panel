import type { MotorConfig } from "./tauri";

export type MotorNode = {
  id: number;
  name: string;
  x: number;
  y: number;
  color: string;
};

export const MOTOR_NODES: MotorNode[] = [
  { id: 0, name: "R Brow Inner", x: 250, y: 155, color: "#f97316" },
  { id: 1, name: "R Brow Outer", x: 190, y: 142, color: "#f97316" },
  { id: 2, name: "L Brow Inner", x: 390, y: 155, color: "#f97316" },
  { id: 3, name: "L Brow Outer", x: 450, y: 142, color: "#f97316" },
  { id: 4, name: "L Cheek Tendon", x: 470, y: 285, color: "#ef4444" },
  { id: 5, name: "L Nose Tendon", x: 412, y: 250, color: "#ef4444" },
  { id: 6, name: "R Nose Tendon", x: 228, y: 250, color: "#ef4444" },
  { id: 7, name: "R Cheek Tendon", x: 170, y: 285, color: "#ef4444" },
  { id: 8, name: "R Upper Lid", x: 225, y: 195, color: "#22c55e" },
  { id: 9, name: "R Lower Lid", x: 225, y: 225, color: "#22c55e" },
  { id: 10, name: "R Eye Horizontal", x: 190, y: 210, color: "#22c55e" },
  { id: 11, name: "R Eye Vertical", x: 260, y: 210, color: "#22c55e" },
  { id: 12, name: "L Upper Lid", x: 415, y: 195, color: "#22c55e" },
  { id: 13, name: "L Lower Lid", x: 415, y: 225, color: "#22c55e" },
  { id: 14, name: "Upper Lip Left", x: 385, y: 365, color: "#38bdf8" },
  { id: 15, name: "Upper Lip Mid", x: 320, y: 350, color: "#38bdf8" },
  { id: 16, name: "Upper Lip Right", x: 255, y: 365, color: "#38bdf8" },
  { id: 17, name: "R Mouth Corner Up", x: 215, y: 380, color: "#38bdf8" },
  { id: 18, name: "R Mouth Corner Low", x: 205, y: 410, color: "#38bdf8" },
  { id: 19, name: "L Mouth Corner Up", x: 425, y: 380, color: "#38bdf8" },
  { id: 20, name: "L Mouth Corner Low", x: 435, y: 410, color: "#38bdf8" },
  { id: 21, name: "Lower Lip Left", x: 370, y: 430, color: "#38bdf8" },
  { id: 22, name: "Lower Lip Right", x: 270, y: 430, color: "#38bdf8" },
  { id: 23, name: "Lower Lip Mid", x: 320, y: 445, color: "#38bdf8" },
  { id: 24, name: "Jaw Horizontal", x: 320, y: 500, color: "#a78bfa" },
  { id: 25, name: "Jaw R Upper", x: 235, y: 475, color: "#a78bfa" },
  { id: 26, name: "Jaw R Lower", x: 220, y: 520, color: "#a78bfa" },
  { id: 27, name: "Jaw L Upper", x: 405, y: 475, color: "#a78bfa" },
  { id: 28, name: "Tongue Upper", x: 300, y: 472, color: "#a78bfa" },
  { id: 29, name: "Tongue Lower", x: 340, y: 492, color: "#a78bfa" },
  { id: 30, name: "Neck Left", x: 272, y: 585, color: "#facc15" },
  { id: 31, name: "Neck Right", x: 368, y: 585, color: "#facc15" },
];

export const DEFAULT_MOTOR_CONFIG: MotorConfig[] = MOTOR_NODES.map((node) => ({
  id: node.id,
  name: node.name,
  min_angle: -45,
  max_angle: 45,
  zero_offset: 0,
  home_logical: 0,
}));

export const BLENDSHAPE_NAMES = [
  "browInnerUp",
  "browOuterUpLeft",
  "browOuterUpRight",
  "eyeBlinkLeft",
  "eyeBlinkRight",
  "cheekSquintLeft",
  "cheekSquintRight",
  "mouthSmileLeft",
  "mouthSmileRight",
  "jawOpen",
] as const;

const motorWeights: Record<number, Partial<Record<(typeof BLENDSHAPE_NAMES)[number], number>>> = {
  0: { browInnerUp: 24 },
  1: { browOuterUpRight: 22 },
  2: { browInnerUp: 24 },
  3: { browOuterUpLeft: 22 },
  4: { cheekSquintLeft: 20, mouthSmileLeft: 12 },
  7: { cheekSquintRight: 20, mouthSmileRight: 12 },
  8: { eyeBlinkRight: -28 },
  9: { eyeBlinkRight: 16 },
  12: { eyeBlinkLeft: -28 },
  13: { eyeBlinkLeft: 16 },
  17: { mouthSmileRight: 26 },
  18: { mouthSmileRight: 14, jawOpen: -8 },
  19: { mouthSmileLeft: 26 },
  20: { mouthSmileLeft: 14, jawOpen: -8 },
  21: { jawOpen: 18 },
  22: { jawOpen: 18 },
  23: { jawOpen: 22 },
  24: { jawOpen: 28 },
  25: { jawOpen: 24 },
  26: { jawOpen: 30 },
  27: { jawOpen: 24 },
  30: { jawOpen: 6 },
  31: { jawOpen: 6 },
};

export const BLENDSHAPE_MAPPING = MOTOR_NODES.map((node) =>
  BLENDSHAPE_NAMES.map((shape) => motorWeights[node.id]?.[shape] ?? 0),
);
