# Vehicle profiles and thruster commands

The default is `current_m5`: the existing submarine with Pixhawk and VideoRay
M5 thrusters. The future Pixhawk-free T200 submarine has its own saved profile.

| Profile | Settings | Command | Geometry status |
| --- | --- | --- | --- |
| `current_m5` | `robosub_control/config/current_m5.py` | Normalized M5 power, -1 to +1 | User-supplied current mounts |
| `future_t200` | `robosub_control/config/future_t200.py` | T200 force in N, then Basic ESC PWM | Planning copy; future mounts unconfirmed |

Both settings files are inside
`robosub_ws/src/robosub_control/robosub_control/config/`.
Common control/tuning/timeouts remain in `constants.py`. The saved T200
performance data, voltage interpolation, current bounds, PWM mapping and
startup hold are documented in [T200 thrusters and PWM](thrusters-t200.md).

```bash
# Current M5 simulation, the default:
ros2 launch robosub_bringup robot.launch.py vehicle:=current_m5

# Future T200 planning simulation:
ros2 launch robosub_bringup robot.launch.py vehicle:=future_t200

# Current M5 shared logic and numeric previews, without camera access:
ros2 launch robosub_bringup robot.launch.py mode:=real vision:=false
```

Real mode defaults to numeric previews. With `teleop:=browser` or
`teleop:=local`, `actuators:=teensy_usb` sends M5 commands through the existing
USB/Serial1 firmware. See [Xbox driving](gamepad.md). Autonomous physical
operation still needs sensor adapters. Launching does not flash firmware.

## What the existing receiver does

The user confirmed that `prototypes/eddie/Thruster_MavLink.ino` represents the
current receiver design, while warning that the file is outdated. Treat it as
reference evidence rather than production firmware.

```text
Pixhawk motor-output numbers in MAVLink SERVO_OUTPUT_RAW
  -> receiver microcontroller over a serial telemetry link
  -> normalize each of eight numbers: (value - 1500) / 500
  -> construct VideoRay propulsion packet and CRC checksums
  -> RS-485 transceiver and bus
  -> each M5's integrated motor controller
```

The receiver reads **MAVLink messages**, not physical PWM pins. The numbers
look like PWM widths: 1000 becomes -1, 1500 becomes zero, and 2000 becomes +1.
RS-485 describes the electrical communication bus; VideoRay packets describe
the information sent over it. Flashing the receiver installs the translating
program once; it is not part of sending each motor command.

The prototype uses 115200 baud on both serial links, a group destination of
`0x81`, propulsion command `0xAA`, and custom register `0xF0`. It sends eight
float values with separate header/payload CRCs. These are observations about
that file, not verification of the installed board or its wiring.

The VideoRay protocol distinguishes a **motor ID** (which power-array slot a
motor uses) from the **reply node ID** (which device should send telemetry).
The old code's `MOTOR_ID` variable is used for the reply selector. It does not
establish the motor mapping. The operator has now specified the mapping
below; the reply node ID remains separate and unconfigured in the profile.

| M5 motor ID / USB slot | Motor |
| --- | --- |
| 0 | FR |
| 1 | FL |
| 2 | BR |
| 3 | BL |
| 4 | RD |
| 6 | LD |

`thruster_layout.py` defines the six-element logical order
`FR,FL,BR,BL,RD,LD` for ROS arrays and simulation. `current_m5.py` maps those
names to physical motor IDs; LD uses ID 6, while its logical array position
is still 5. RD/LD mean right-down/left-down. USB slots 5 and 7 remain neutral. This records the supplied mapping; it does not reprogram IDs
in the motor controllers.

The old prototype does not clamp normalized commands or provide a local
command-loss watchdog. An invalid zero servo value would become -3. Any
replacement must validate input, define neutral behavior and enforce its own
watchdog before being used on hardware. The new ROS watchdog cannot protect
against a frozen receiver or disconnected upstream link by itself.

## Current M5 command domain

The M5 has an integrated controller and accepts normalized power commands.
A command of 0.5 does **not** establish 50% of maximum physical thrust. We do
not have a measured power-to-newton curve, supply voltage or polarity checks
for this submarine. T200's 16 V reference, Basic ESC
pulse widths, 30 A model limit and 400 Hz update limit do not apply to M5.
The M5 profile caps ROS command publication at 100 Hz as a software policy;
this is not a claimed manufacturer limit or a verified receiver bus rate.

The mixer accounts for the measured directions and moment arms while assuming
equal authority per unit power. This is a starting approximation, not a
calibrated force allocator. Combined requests are scaled together to stay in
[-1, +1]. Existing PID gains still require tuning for this vehicle.

| Topic | Contents |
| --- | --- |
| `/sub/cmd_vel` | Normalized body efforts in `Twist`, not measured speeds |
| `/sub/m5/power_targets` | Six power targets, `FR,FL,BR,BL,RD,LD` order |
| `/sub/m5/power/t0..t5` | Individual power targets using simulation labels |
| `/sub/m5/power_preview` | Six polarity-adjusted power commands in named motor order |
| `/sub/m5/power_sent` | USB-acknowledged, capped motor powers in named order |
| `/sim/thrust/t0..t5` | Simulation-only applied forces in N |

Power arrays use `Float64MultiArray` with the named order in the layout label.
They contain no bus addresses. The command stage rejects malformed arrays,
non-finite values and powers outside [-1, +1]; missing/stale input produces
zero power. It does not publish a PWM preview for M5.

Simulation alone converts power into force using
`SIM_FULL_POWER_FORCE_N = 60.0`, an explicitly uncalibrated placeholder, not an
M5 specification. Its RPM animation coefficient and CAD meshes are also not
validated M5 performance/geometry. Use simulation to check program behavior;
its thrust or tuning predictions need measured calibration before comparison
with the physical submarine.

## Current measured mounts

Onshape axes are +X aft, +Y starboard, +Z up. ROS body positions use
`(-X, -Y, Z)`, about the supplied robot origin rather than a measured center of
mass. BR yaw is 225 degrees clockwise. DVL and camera translations are in the
current M5 profile. The camera position is `(-0.157118304, 0.006, -0.0526)` m
in body axes; its orientation is still a zero placeholder, not a measured
camera/IMU transform. The control-board position is the current Pixhawk mount.
The real DVL adapter must rotate its downward sensor frame into body axes.

Sources: [M5 specifications](https://download.videoray.com/documentation/m5_thruster/html/specifications.html),
[VideoRay command format](https://download.videoray.com/documentation/m5_thruster/html/commands_responses.html),
[manufacturer Python example](https://download.videoray.com/documentation/m5_thruster/html/thruster_py.html).
