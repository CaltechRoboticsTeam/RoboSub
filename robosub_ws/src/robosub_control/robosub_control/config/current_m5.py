"""Current VideoRay M5 vehicle; live USB transport is explicitly selected.

The user confirmed these mounts and the legacy MAVLink receiver firmware.
Polarity, supply voltage and power-to-force calibration
still require verification on this vehicle.
"""

import math

from robosub_control.config.thruster_layout import (
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

DVL_POSITION_ONSHAPE_M = (0.153, -0.007, -0.016)
DVL_POSITION_BODY_M = onshape_to_body(DVL_POSITION_ONSHAPE_M)
DVL_MOUNT_PITCH_DOWN_DEG = 90.0

# Measured Pixhawk location on the current M5 vehicle.
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

NAME = "current_m5"
ACTUATOR_KIND = "m5"
GEOMETRY_CONFIRMED = True
THRUSTER_REVERSED = {name: False for name in THRUSTER_NAMES}
THRUSTER_DIRECTIONS_CONFIRMED = False
# Motor IDs select array slots; reply node ID is a separate protocol field.
M5_MOTOR_IDS = {"FR": 0, "FL": 1, "BR": 2, "BL": 3, "RD": 4, "LD": 6}
M5_REPLY_NODE_ID = None
M5_SUPPLY_VOLTAGE_V = None
M5_POWER_TO_FORCE_CALIBRATION = None
M5_PROPELLER_DIAMETER_M = 0.090
# Observed in the old prototype, not verified against the installed firmware.
LEGACY_RECEIVER_BAUD = 115200
LEGACY_RECEIVER_OUTPUT_SLOTS = 8
# Software publication policy, not a manufacturer-rated update limit.
MAX_OUTPUT_RATE_HZ = 100.0
TARGET_TOPIC = "/sub/m5/power_targets"
INDIVIDUAL_TOPIC_PREFIX = "/sub/m5/power"
PREVIEW_TOPIC = "/sub/m5/power_preview"
SENT_TOPIC = "/sub/m5/power_sent"
NEUTRAL_COMMAND = 0.0
INITIAL_NEUTRAL_S = 0.0
# Uncalibrated simulation proxy only. Real commands remain power fractions.
SIM_FULL_POWER_FORCE_N = 60.0

# Axis scaling assumes equal motor authority in normalized power space.
# Combined commands are scaled together if a thruster would saturate.
MIXER_AXIS_FRACTIONS = {
    "surge": 1.0,
    "sway": 1.0,
    "heave": 1.0,
    "roll": 1.0,
    "yaw": 1.0,
}
