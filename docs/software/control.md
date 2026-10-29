# Control code map

Shared robot code lives in `robosub_ws/src/robosub_control/robosub_control/`.
Configuration and helper modules do not run as ROS nodes.

| What to change | File under `robosub_control/` |
| --- | --- |
| Core body-effort-to-thruster calculation | `thrusters/thrust_allocation.py` |
| ROS command input and named motor targets | `thrusters/thruster_mixer.py` |
| Real M5 output and its input watchdog | `drivers/teensy_actuators.py` |
| USB framing, acknowledgments and slot conversion | `drivers/teensy_usb.py` |
| Offline decoding of M5 responses | `drivers/videoray.py` |
| Xbox mapping, movement toggle and trigger roll | `teleop/gamepad.py` |
| Controller-input ROS command | `teleop/xbox_teleop.py` |
| Motor IDs, polarity and measured mounts | `config/current_m5.py` |
| Future T200 settings | `config/future_t200.py` |
| Common defaults and timeouts | `config/constants.py` |
| Profile selection | `config/vehicle_profiles.py` |
| Cached tunable number and ROS parameter integration | `utils/tunable_number.py`, `utils/tuning.py` |
| Automatic state estimation, hold and missions | `autonomy/` |
| Numeric preview and simulated actuator response | `thrusters/actuator_commands.py` |

The pure allocator has no ROS or hardware access. The mixer is its ROS
adapter. The Teensy driver implements physical output; the preview adapter
implements software output. Both use the same allocator and vehicle profile.

## Live joystick execution

The browser joystick setup starts four ROS nodes:

```text
tuning_menu: HTML page -> /operator/joy
                             |
xbox_teleop: RB toggle + sticks/triggers -> /sub/cmd_vel
                                               |
thruster_mixer: geometry allocation -> /sub/m5/power_targets
                                               |
teensy_actuators: motor-ID mapping -> USB -> Teensy Serial1 -> M5
```

The menu also reads movement state and USB-acknowledged motor commands.
Live manual driving does not launch the estimator, automatic hold controller
or numeric-preview node. Preview mode substitutes `actuator_commands` for
the physical driver. Local Linux controller input adds the SDL `joy` node.

Press RB once to enable movement and again to disable it. RT rolls right;
LT rolls left. Enabling requires neutral sticks and released triggers.
Input loss, browser disable, focus loss and disconnect reset the toggle.
After reconnecting, center/release all controls, then press RB again.
Pitch remains unavailable as an independent control axis.

## Other packages and tools

`robosub_bringup` owns the master launcher, HTML menu and browser input bridge.
`robosub_sim` owns Gazebo assets, the world builder, simulated sensors and
the passive terminal dashboard. Neither contains the shared vehicle logic.

`tools/diagnostics/` contains explicit field checks:

| Tool | Purpose |
| --- | --- |
| `teensy_test.py` | Neutral USB checks and explicit motor-slot pulses |
| `field_test.py` | Passive MAVROS observations or an explicit PX4 pulse |
| `field_checks.py`, `field_monitor.py` | Helpers for the PX4 field tool |

These are operator diagnostics, not automated unit tests. The deployed
`teensy.sh` and `field.sh` wrappers invoke them directly. `tests/control/`
contains automated tests; hardware transports are emulated there. The unused
PX4-to-USB relay is removed. Joystick commands use the direct M5 USB driver.

See [Xbox controller](gamepad.md) for driving and changing motor IDs, and
[robot startup](runtime.md) for launching simulation or automatic control.
