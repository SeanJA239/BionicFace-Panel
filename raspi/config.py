# Canonical hardware calibration source for the current project architecture.
#
# PC side:
# - reads this file via `raspi/export_config_json.py`
# - owns clamp/offset/interpolation/UDP heartbeat
#
# Raspberry Pi side:
# - reads this file only for board address, motor map, and UDP port
# - acts as a dumb executor

BOARD_ADDRESSES = [0x40, 0x41]
UDP_PORT = 6000

MOTOR_NAMES = {
    0: "eyebrow_right_inner",
    1: "eyebrow_right_outer",
    2: "eyebrow_left_inner",
    3: "eyebrow_left_outer",
    4: "cheek_left_tendon",
    5: "nose_left_tendon",
    6: "nose_right_tendon",
    7: "cheek_right_tendon",
    8: "eye_horizontal",
    9: "eye_left_upper",
    10: "eye_left_lower",
    11: "eye_right_upper",
    12: "eye_right_lower",
    13: "eye_vertical",
    14: "upper_lip_left",
    15: "upper_lip_mid",
    16: "upper_lip_right",
    17: "mouth_right_corner_upper",
    18: "mouth_right_corner_lower",
    19: "mouth_left_corner_upper",
    20: "mouth_left_corner_lower",
    21: "lower_lip_left",
    22: "lower_lip_right",
    23: "lower_lip_mid_tendon",
    24: "jaw_horizontal",
    25: "jaw_right_upper",
    26: "jaw_right_lower",
    27: "jaw_left",
    28: "tongue_upper",
    29: "tongue_lower",
    30: "neck_left",
    31: "neck_right",
}

# Neck channels remain in the 32-channel protocol, but are temporarily excluded
# from data collection while the neck structure is under active redesign.
DISABLED_MOTORS = [30, 31]

MOTOR_MAP = {
    # eyebrow_rigid
    0: (0, 0),  # eyebrow_GUOHUAA0090:1 'eyebrow_right_inner'
    1: (0, 1),  # eyebrow_GUOHUAA0090:2 'eyebrow_right_outer'
    2: (0, 2),  # eyebrow_GUOHUAA0090:3 'eyebrow_left_inner'
    3: (0, 3),  # eyebrow_GUOHUAA0090:4 'eyebrow_left_outer'
    4: (0, 10),  # cheek_tendon_GUOHUAA0090:5 'cheek_left_tendon'
    5: (0, 11),  # cheek_tendon_GUOHUAA0090:6 'nose_left_tendon'
    6: (0, 12),  # cheek_tendon_GUOHUAA0090:7 'nose_right_tendon'
    7: (0, 13),  # cheek_tendon_GUOHUAA0090:8 'cheek_right_tendon'
    # eye_rigid
    8: (0, 4),  # eye_horizontal: 90-zero connected to eye_m_shaped_board: 1
    9: (0, 5),  # eye_left_upper
    10: (0, 6),  # eye_left_lower
    11: (0, 7),  # eye_right_upper
    12: (0, 8),  # eye_right_lower
    13: (0, 9),  # eye_vertical: 90-mid
    # mouth_rigid
    14: (1, 0),  # mouth_MG90S: 1 'upper_lip_left'
    15: (1, 1),  # mouth_MG90S: 11 'upper_lip_mid'
    16: (1, 2),  # mouth_MG90S: 10 'upper_lip_right'
    17: (1, 3),  # mouth_MG90S: 8 'mouth_right_corner_upper'
    18: (1, 4),  # mouth_MG90S: 9 'mouth_right_corner_lower'
    19: (1, 5),  # mouth_MG90S: 6 'mouth_left_corner_upper'
    20: (1, 6),  # mouth_MG90S: 7 'mouth_left_corner_lower'
    21: (1, 7),  # mouth_MG90S: 18 'lower_lip_left'
    22: (1, 8),  # mouth_MG90S: 17 'lower_lip_right'
    23: (1, 9),  # mouth_MG90S: 23 'lower_lip_mid_tendon'
    24: (1, 10),  # mouth_MG90S: 16 'jaw_horizontal'
    25: (1, 11),  # KS3518: 1 'jaw_right_upper'
    26: (1, 12),  # mouth_MG90S: 2 'jaw_right_lower'
    27: (1, 13),  # mouth_GUOHUAA0090: 1 'jaw_left'
    28: (1, 14),  # mouth_MG90S: 12 'tongue_upper'
    29: (1, 15),  # mouth_MG90S: 21 'tongue_lower'
    # neck_rigid
    30: (0, 14),  # neck_KS3518: 1 'neck_left'
    31: (0, 15),  # neck_KS3518: 2 'neck_right'
}

MOTOR_LIMITS = {
    0: (170, 210),  # 90-zero
    1: (10, 40),  # 90-zero
    2: (170, 210),  # 90-zero
    3: (10, 40),  # 90-zero
    4: (0, 180),  # 90-zero tendon
    5: (0, 180),  # 90-zero tendon
    6: (0, 180),  # 90-zero tendon
    7: (0, 180),  # 90-zero tendon
    8: (0, 180),  # 90-zero
    9: (60, 135),
    10: (60, 135),
    11: (60, 135),
    12: (60, 135),
    13: (90, 120),
    14: (75, 105),  # 90-mid
    15: (90, 120),  # 90-zero
    16: (75, 105),  # 90-mid
    17: (30, 50),  # 90-mid
    18: (90, 130),  # 90-mid
    19: (30, 50),  # 90-mid
    20: (90, 130),  # 90-mid
    21: (80, 90),  # 90-rand
    22: (90, 100),  # 90-rand
    23: (85, 95),  # 90-mid
    24: (60, 120),  # 90-zero-r
    25: (0, 180),
    26: (0, 180),
    27: (0, 180),
    28: (255, 285),  # 90-mid-r
    29: (75, 105),
    30: (0, 180),
    31: (0, 180),
}

MOTOR_OFFSET = {
    0: 3,
    1: 3,
    2: 3,
    3: 3,
    8: 5,
    13: 5,
    21: 3,
    22: 3,
    24: 10,
}
