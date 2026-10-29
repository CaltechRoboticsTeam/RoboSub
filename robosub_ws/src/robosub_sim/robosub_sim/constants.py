"""Shared runtime defaults; vehicle hardware lives in the named profiles."""

DEFAULT_VEHICLE_PROFILE = "current_m5"

SIM_SURFACE_Z_M = 0.0

ROLL_SPIN_RATE_RAD_S = 0.6
MANUAL_COMMAND_TIMEOUT_S = 0.5
THRUSTER_COMMAND_TIMEOUT_S = 0.5
THRUSTER_WATCHDOG_PERIOD_S = 0.1
MISSION_PERIOD_S = 0.2

# Overrides every launch option: True forces tunable numbers to code defaults.
LOCK_TUNING_DEFAULTS = False
MIN_CONTROL_RATE_HZ = 1.0
MAX_CONTROL_RATE_HZ = 1000.0

CONTROLLER_DEFAULTS = {
    "rate": 20.0,
    "output_rate": 100.0,
    "depth_kp": 1.0,
    "depth_ki": 0.2,
    "depth_kd": 0.3,
    "heading_kp": 1.0,
    "heading_ki": 0.0,
    "heading_kd": 0.1,
    "pos_kp": 0.5,
    "pos_kd": 0.3,
    "pos_limit": 0.5,
    "roll_kp": 0.75,
    "roll_ki": 0.5,
    "roll_kd": 0.08,
    "roll_ff": 0.56,
    "roll_spin_rate": ROLL_SPIN_RATE_RAD_S,
    "i_limit": 1.0,
    "state_timeout": 0.5,
}

ESTIMATOR_DEFAULTS = {
    "kp": 2.0,
    "ki": 0.02,
    "depth_tau": 0.08,
    "heave_tau": 0.25,
    "dvl_offset": [0.0, 0.0, 0.0],
    "dvl_timeout": 1.0,
}

DVL_DEFAULTS = {
    "rate": 7.0,
    "noise_std": 0.004,
    "scale_error": 0.002,
    "dvl_offset": [0.0, 0.0, 0.0],
    "floor_z": -9.5,
    "min_altitude": 0.2,
    "max_altitude": 80.0,
    "dropout_prob": 0.01,
    "frame_id": "base_link",
}

MISSION_DEFAULTS = {
    "mission": ["dive 1.5", "move 3.0", "goto 0 0"],
    "pos_tol": 0.20,
    "depth_tol": 0.10,
    "heading_tol": 5.0,
    "settle": 1.0,
    "step_timeout": 60.0,
}

# Xbox input: body-relative surge/sway/heave/yaw, with RB held to enable.
GAMEPAD_DEADZONE = 0.08
GAMEPAD_MAX_EFFORT = 0.5
GAMEPAD_PUBLISH_RATE_HZ = 20.0
GAMEPAD_INPUT_TIMEOUT_S = 0.25
# W3C standard Gamepad layout versus ROS SDL game_controller_node layout.
GAMEPAD_ENABLE_BUTTON = {"browser": 5, "local": 10}
