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

UDP_PORT = 6000
BOARD_ADDRESSES = [0x40, 0x41]

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

MOTOR_LIMITS = {
    0: (170, 210),
    1: (10, 40),
    2: (170, 210),
    3: (10, 40),
}

MOTOR_OFFSET = {
    0: 3,
    1: 3,
    2: 3,
    3: 3,
}
