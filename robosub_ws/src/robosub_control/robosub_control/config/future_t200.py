"""Saved T200 / Basic ESC R3 configuration for the future vehicle.

Mounts are a planning copy of the current M5 measurements. They have not
been measured on the future vehicle. Voltage, channels and polarity are
unverified setup defaults.
"""

import math

from robosub_control.config.thruster_layout import (
    THRUSTER_INDICES,
    THRUSTER_NAMES as THRUSTER_NAMES,
    THRUSTER_TOPIC_IDS as THRUSTER_TOPIC_IDS,
    THRUSTERS as THRUSTERS,
)


def onshape_to_body(position):
    """Rotate a position/vector 180 degrees about z; metres stay metres."""
    x, y, z = position
    return (-x, -y, z)


THRUSTER_POSITIONS_ONSHAPE_M = {
    "FL": (-0.352, -0.229, 0.033),
    "FR": (-0.352, 0.215, 0.033),
    "BR": (0.352, 0.215, 0.033),
    "BL": (0.352, -0.229, 0.033),
    "LD": (-0.012, -0.301, -0.014),
    "RD": (-0.012, 0.287, -0.014),
}
THRUSTER_YAW_CW_DEG = {
    "FL": 45.0,
    "FR": 315.0,
    "BR": 225.0,
    "BL": 135.0,
    "LD": 0.0,
    "RD": 0.0,
}
THRUSTER_PITCH_DOWN_DEG = {
    name: 90.0 if name in ("LD", "RD") else 0.0 for name in THRUSTER_NAMES
}
THRUSTER_POSITIONS_BODY_M = {
    name: onshape_to_body(position)
    for name, position in THRUSTER_POSITIONS_ONSHAPE_M.items()
}

# Software channel order; future physical wiring remains unverified.
THRUSTER_CHANNELS = dict(THRUSTER_INDICES)
THRUSTER_REVERSED = {name: False for name in THRUSTER_NAMES}
THRUSTER_CHANNELS_CONFIRMED = False
THRUSTER_DIRECTIONS_CONFIRMED = False

# Reference for the thrust model; replace with the actual voltage at the ESCs.
THRUSTER_SUPPLY_VOLTAGE_V = 16.0
THRUSTER_SUPPLY_VOLTAGE_CONFIRMED = False
ESC_CONTINUOUS_CURRENT_A = 30.0
ESC_CURRENT_MODEL_LIMIT_A = 30.0
ESC_PWM_MIN_US = 1100
ESC_PWM_NEUTRAL_US = 1500
ESC_PWM_MAX_US = 1900
ESC_PWM_DEADBAND_US = 25
ESC_MAX_UPDATE_RATE_HZ = 400.0
# Vendor requires a neutral signal for a few seconds; this is our hold policy.
ESC_NEUTRAL_INIT_S = 3.0
T200_PROPELLER_DIAMETER_M = 0.0762

# 1.0 requests the model's symmetric, single-axis available force/torque.
# Combined commands are scaled together if a thruster would saturate.
MIXER_AXIS_FRACTIONS = {
    "surge": 1.0,
    "sway": 1.0,
    "heave": 1.0,
    "roll": 1.0,
    "yaw": 1.0,
}

DVL_POSITION_ONSHAPE_M = (0.153, -0.007, -0.016)
DVL_POSITION_BODY_M = onshape_to_body(DVL_POSITION_ONSHAPE_M)
DVL_MOUNT_PITCH_DOWN_DEG = 90.0

# The custom boards replace the former Pixhawk at this measured location.
CONTROL_BOARD_POSITION_ONSHAPE_M = (0.0, -0.006, 0.049)
CONTROL_BOARD_POSITION_BODY_M = onshape_to_body(
    CONTROL_BOARD_POSITION_ONSHAPE_M
)
CAMERA_OFFSET_FROM_BOARD_ONSHAPE_M = (6.185760 * 0.0254, 0.0, -4.0 * 0.0254)
CAMERA_POSITION_ONSHAPE_M = tuple(
    a + b
    for a, b in zip(
        CONTROL_BOARD_POSITION_ONSHAPE_M,
        CAMERA_OFFSET_FROM_BOARD_ONSHAPE_M,
        strict=True,
    )
)
CAMERA_POSITION_BODY_M = onshape_to_body(CAMERA_POSITION_ONSHAPE_M)
CAMERA_RPY_BODY_RAD = (0.0, 0.0, 0.0)
CAMERA_ORIENTATION_CONFIRMED = False

# RPY for a mount axis uses right-hand ROS rotations, not optical image axes.
DVL_MOUNT_RPY_BODY_RAD = (0.0, math.pi / 2.0, 0.0)
SIM_DVL_OFFSET_M = DVL_POSITION_BODY_M

NAME = "future_t200"
ACTUATOR_KIND = "t200"
GEOMETRY_CONFIRMED = False
MAX_OUTPUT_RATE_HZ = ESC_MAX_UPDATE_RATE_HZ
TARGET_TOPIC = "/sub/thruster_forces"
INDIVIDUAL_TOPIC_PREFIX = "/sub/thrust"
PREVIEW_TOPIC = "/sub/esc/pwm_preview"
NEUTRAL_COMMAND = ESC_PWM_NEUTRAL_US
