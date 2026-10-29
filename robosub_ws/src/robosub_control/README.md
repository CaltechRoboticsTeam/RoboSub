# Shared vehicle control

Start with these files inside `robosub_control/`:

- `thrusters/thrust_allocation.py`: core motor-allocation calculation.
- `thrusters/thruster_mixer.py`: ROS adapter for body-effort commands.
- `drivers/teensy_actuators.py`: real M5 output implementation.
- `teleop/gamepad.py`: RB toggle, stick mapping and RT/LT roll.
- `teleop/xbox_teleop.py`: controller-input ROS command.
- `config/current_m5.py`: current motor IDs and measured mounts.
- `utils/`: cached tuning numbers and ROS parameter integration.
- `autonomy/`: estimator, hold controller and mission commands.

`robosub_sim` contains simulator-specific code and assets. `robosub_bringup`
contains startup and the HTML menu. Hardware diagnostics live separately in
`tools/diagnostics/`; automated checks live in `tests/control/`.

The full code map is in `docs/software/control.md` at the repository root.
