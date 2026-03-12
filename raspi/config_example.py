"""Example calibration schema for the Raspberry Pi servo server.

The runtime script loads the real config from ``../bionicFace/config.py`` by
default. This file is only a reference for the richer safety/calibration
structure that can be gradually added there.
"""

from __future__ import annotations

from typing import TypedDict


class MotorSafety(TypedDict, total=False):
    name: str
    min_angle: float
    max_angle: float
    zero_offset: float
    home_logical: float
    actuation_range: float


PCA9685_ADDRESSES = [
    0x40,
    0x41,
    0x42,
]

ZMQ_BIND = "tcp://0.0.0.0:5555"
PWM_FREQUENCY_HZ = 50
HOME_STEP_DELAY_SEC = 0.03
SERVO_RELEASE_ON_EXIT = False

MOTOR_MAP = {
    0: (0, 0),
    1: (0, 1),
    2: (0, 2),
    3: (0, 3),
    4: (2, 0),
    5: (2, 1),
    6: (2, 2),
    7: (2, 3),
    8: (0, 4),
    9: (0, 5),
    10: (0, 6),
    11: (0, 7),
    12: (0, 8),
    13: (0, 9),
    14: (1, 0),
    15: (1, 1),
    16: (1, 2),
    17: (1, 3),
    18: (1, 4),
    19: (1, 5),
    20: (1, 6),
    21: (1, 7),
    22: (1, 8),
    23: (1, 9),
    24: (1, 10),
    25: (1, 11),
    26: (1, 12),
    27: (1, 13),
    28: (1, 14),
    29: (1, 15),
    30: (2, 4),
    31: (2, 5),
}

# Optional calibration layer on top of MOTOR_MAP.
# If a motor is missing here, runtime defaults are:
# min_angle=0, max_angle=180, zero_offset=0, home_logical=90, actuation_range=180
MOTOR_SAFETY: dict[int, MotorSafety] = {
    0: {"name": "eyebrow_right_inner", "min_angle": 70, "max_angle": 118, "zero_offset": 0, "home_logical": 90},
    1: {"name": "eyebrow_right_outer", "min_angle": 68, "max_angle": 122, "zero_offset": 0, "home_logical": 90},
    2: {"name": "eyebrow_left_inner", "min_angle": 62, "max_angle": 112, "zero_offset": 0, "home_logical": 90},
    3: {"name": "eyebrow_left_outer", "min_angle": 64, "max_angle": 118, "zero_offset": 0, "home_logical": 90},
    14: {"name": "upper_lip_left", "min_angle": 52, "max_angle": 128, "zero_offset": 0, "home_logical": 90},
    15: {"name": "upper_lip_mid", "min_angle": 50, "max_angle": 126, "zero_offset": 0, "home_logical": 90},
}
