# robosub_ws — Robot workspace

For the separate ZED 2i perception launch and calibration workflow, see
[underwater vision](../docs/software/vision.md).
For a file-by-file guide, see [the control code map](../docs/software/control.md).
For the master launcher and developer menu, see
[robot startup and tuning](../docs/software/runtime.md).
For deployment and current-submarine MAVROS checks, see
[field testing](../docs/software/field-testing.md).
For Xbox input from the Mac or Linux, see
[controller setup](../docs/software/gamepad.md).
For measured motor/sensor positions and the M5 power / T200 PWM paths, see
[vehicle profiles and M5/T200 commands](../docs/software/thrusters.md).

ROS 2 **Humble** + Gazebo **Harmonic**, in a container. The movement stack is
exercised with simulated sensors. Real manual M5 driving uses the optional
Teensy USB transport; autonomous hardware integration remains incomplete.

## Why Docker
This desktop runs Ubuntu 24.04, whose native ROS 2 is **Jazzy**. The sub's
Jetson runs **Humble** (JetPack 6 / Ubuntu 22.04). Running Humble in a
container here keeps the ROS version consistent. Simulation does not verify
physical drivers, sensor mounting, calibration or controller tuning.

## Quick start
```bash
./run.sh                      # build/enter the container
colcon build --symlink-install --packages-select \
  robosub_control robosub_sim robosub_bringup
source install/setup.bash
ros2 launch robosub_bringup robot.launch.py mode:=sim
```

Drive it (second terminal — `./run.sh` again attaches to the running container):
```bash
source install/setup.bash
ros2 topic pub -r 10 /sub/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 1.0}, angular: {z: 0.2}}'
```

Watch everything at once (sensors on the left, what the estimator and
controller make of them on the right; drive from another terminal):
```bash
ros2 run robosub_sim dashboard
```

Or topic by topic:
```bash
ros2 topic echo /sub/odom
ros2 topic list
rqt_image_view /sub/camera/image_raw
```

## Layout
| Path | What it is |
|---|---|
| `docker/Dockerfile` | Humble + Harmonic + bridge + tools |
| `docker/compose.yaml` | GPU, X11, host networking, volume mounts |
| `src/robosub_bringup/launch/robot.launch.py` | Master entry: sim/real, vision, mission and tuning |
| `src/robosub_sim/worlds/pool.sdf` | Pool, water, sub, gate |
| `src/robosub_sim/config/bridge.yaml` | Gazebo topics <-> ROS topics |
| `src/robosub_control/robosub_control/thrusters/thruster_mixer.py` | Control allocation |
| `src/robosub_control/robosub_control/thrusters/actuator_commands.py` | M5 power / T200 PWM preview and simulated response |
| `src/robosub_sim/robosub_sim/depth_sensor.py` | Altimeter -> `/depth` (stand-in for the pressure sensor driver) |
| `src/robosub_sim/robosub_sim/dvl_sensor.py` | Ground truth -> `/dvl/velocity`, `/dvl/altitude` (stand-in for the Pathfinder DVL driver) |
| `src/robosub_control/robosub_control/autonomy/state_estimator.py` | `/imu/data_raw` + `/depth` + `/dvl/velocity` -> `/sub/state` (attitude, depth, x/y) |
| `src/robosub_control/robosub_control/autonomy/depth_heading_controller.py` | PID depth + heading + position hold -> `/sub/cmd_vel` |
| `src/robosub_control/launch/vehicle.launch.py` | Shared movement stack and previews |
| `src/robosub_control/robosub_control/drivers/teensy_actuators.py` | Optional real M5 USB output |
| `src/robosub_sim/launch/pool.launch.py` | The sim: Gazebo + bridge + sensor stand-ins + `vehicle.launch.py` |
| `src/robosub_sim/robosub_sim/dashboard.py` | Live terminal view: sensors | estimator | commands | thrust, with rates |
| `src/robosub_control/robosub_control/autonomy/mission.py` | Scripted missions: `dive`, `heading`, `move`, `goto`, `roll`, `wait`, `surface` |

## Topics
| ROS topic | Type | Direction |
|---|---|---|
| `/sub/cmd_vel` | `geometry_msgs/Twist` | you -> mixer |
| `/sub/m5/power_targets` | `std_msgs/Float64MultiArray` | M5 mixer -> command stage, normalized power |
| `/sub/m5/power_preview` | `std_msgs/Float64MultiArray` | M5 logical motor powers; preview only |
| `/sub/thruster_forces` | `std_msgs/Float64MultiArray` | T200 mixer -> command stage, N |
| `/sub/esc/pwm_preview` | `std_msgs/UInt16MultiArray` | T200 channel-ordered pulse widths, us; preview only |
| `/sub/m5/power_sent` | `std_msgs/Float64MultiArray` | USB-acknowledged M5 motor powers |
| `/sim/thrust/t{0..5}` | `std_msgs/Float64` | command stage -> Gazebo, applied force in N |
| `/sub/odom` | `nav_msgs/Odometry` | sim -> you (ground truth, **scoring only** -- never subscribe from control code) |
| `/imu/data_raw` | `sensor_msgs/Imu` | sim -> you (100 Hz, gravity-included, noisy + biased like a MEMS part) |
| `/altimeter` | `ros_gz_interfaces/Altimeter` | sim -> depth_sensor (raw, relative to spawn height) |
| `/depth` | `std_msgs/Float64` | depth_sensor -> you (metres below surface, positive down, 50 Hz) |
| `/sub/camera/image_raw` | `sensor_msgs/Image` | sim -> you |
| `/dvl/velocity` | `geometry_msgs/TwistWithCovarianceStamped` | dvl_sensor -> state_estimator (body-frame velocity over ground, ~7 Hz, silent without bottom lock) |
| `/dvl/altitude` | `std_msgs/Float64` | dvl_sensor -> you (metres above the floor) |
| `/sub/state` | `nav_msgs/Odometry` | state_estimator -> control (x/y from start, z = -depth, orientation, body velocity, rates; `pose.covariance[0] = -1` while the DVL is silent) |
| `/sub/state/rpy` | `geometry_msgs/Vector3Stamped` | state_estimator -> you (roll, pitch, yaw in rad, for echoing) |
| `/sub/setpoint/depth` | `std_msgs/Float64` | you -> controller (metres, +down) |
| `/sub/setpoint/heading` | `std_msgs/Float64` | you -> controller (yaw, rad, CCW +, 0 = heading at estimator start) |
| `/sub/setpoint/roll` | `std_msgs/Float64` | you -> controller (roll, rad, + = port side down; ~25 deg max) |
| `/sub/setpoint/roll_turns` | `std_msgs/Float64` | you -> controller (do N full rolls, then level; 1.0 = one 360) |
| `/sub/setpoint/position` | `geometry_msgs/Point` | you -> controller (x, y metres in the estimator's start frame) |
| `/sub/setpoint/move` | `std_msgs/Float64` | you -> controller (go this many metres forward along the heading setpoint) |
| `/sub/cmd_vel_manual` | `geometry_msgs/Twist` | you -> controller (surge/sway pass-through while it holds depth + heading) |

Sensor topics are named the way the real drivers will name them, so state
estimation and control code does not know whether it is talking to Gazebo or
to the vehicle.

**Depth sign convention.** Gazebo's altimeter reports `vertical_position` as
metres moved *up* since spawn and `vertical_reference` as the world z at spawn,
so world height is `reference + position` and `depth = surface_z - (reference + position)`
with `surface_z = 0`. The sub spawns at z = -3, so `/depth` starts at 3.0 and
grows as it dives.

## Depth + heading hold
`pool.launch.py` also starts `state_estimator` and `depth_heading_controller`.
The controller is idle until it gets a setpoint, so manual `/sub/cmd_vel`
driving works as before. To close the loop:

```bash
ros2 topic pub -1 /sub/setpoint/depth   std_msgs/msg/Float64 '{data: 2.0}'   # dive to 2 m
ros2 topic pub -1 /sub/setpoint/heading std_msgs/msg/Float64 '{data: 1.57}'  # turn 90 deg left
ros2 topic pub -1 /sub/setpoint/roll    std_msgs/msg/Float64 '{data: 0.26}'  # roll 15 deg, port down
ros2 topic echo /sub/state/rpy                                                # watch it
```

Roll control combines PID with a feedforward term for righting torque.
Available roll authority depends on the selected vehicle profile. Gains and
feedforward need tuning against the actual vehicle. A barrel roll uses a
separate multi-turn request because ordinary angle setpoints wrap:

```bash
ros2 topic pub -1 /sub/setpoint/roll_turns std_msgs/msg/Float64 '{data: 1.0}'   # one 360, then level
```

During the roll the depth and heading outputs are scaled by cos(roll), so an
inverted sub still pushes the right way and one on its side does not fight.
Earlier template simulations recorded a 360 in ~10 s at 0.6 rad/s. Those
results predate the vehicle profiles and do not validate current M5 or future
T200 tuning.

Once a setpoint has been sent the controller owns `/sub/cmd_vel`. Drive
forward/sideways through `/sub/cmd_vel_manual` instead (only `linear.x` and
`linear.y` are used); publishing on `/sub/cmd_vel` yourself would just fight it.

## Position (DVL)
The vehicle carries a Teledyne Pathfinder DVL. In sim `dvl_sensor` stands in
for its driver: ~7 Hz body-frame velocity over ground with mm/s noise, a
lever-arm term from the DVL's mounting offset, and silence when bottom lock is
lost. The estimator removes the lever arm, rotates by attitude and integrates
into x/y (metres from where the estimator started, x along the start heading).
With that, the sub can be told where to go:

```bash
ros2 topic pub -1 /sub/setpoint/move std_msgs/msg/Float64 '{data: 3.0}'      # 3 m forward
ros2 topic pub -1 /sub/setpoint/position geometry_msgs/msg/Point '{x: 3.0, y: 2.0}'
```

A non-zero `/sub/cmd_vel_manual` cancels position mode (operator wins). If the
DVL goes silent the estimator freezes x/y and flags it, and the controller
holds still instead of driving blind.

Measured in sim: a 3 m move lands within 3 cm of ground truth; a 180 deg turn
in place moves the estimate < 3 cm (lever arm correct); go-to (3, 2) settles
within 2 cm. On the vehicle, replace `dvl_sensor` with the Pathfinder serial
driver publishing the same two topics: set it to bottom-track only, instrument
coordinates and FRESH-water sound speed (the default is seawater, a few %
scale error), and put the mounting yaw and the DVL->IMU offset in its config.

Measured in sim (gains in the node docstrings): a 2 m dive settles in ~8 s
with ~6 % overshoot and holds within 2 cm; a 115 deg turn settles in ~6 s with
no overshoot. Roll/pitch estimate is within 0.6 deg of truth (that residual is
the accelerometer bias in the world file, which a real IMU has too); yaw
tracks the gyro to 0.2 deg through a full turn. Yaw is relative to where the
sub pointed when the estimator started -- an IMU alone cannot know north.

The estimator ignores the Imu message's `orientation` field on purpose:
Gazebo fills it with ground truth, a raw IMU driver may not fill it at all,
and the point is to run the same code on the vehicle.

## Missions
`mission` runs a list of steps, waiting for each to finish before the next.
It only publishes setpoints and watches `/sub/state`, so it runs unchanged on
the vehicle.

```bash
ros2 run robosub_control mission                      # dive 1.5, forward 3 m, back to origin
ros2 run robosub_control mission --ros-args -p "mission:=['dive 2','heading 90','move 2','roll 1','goto 0 0','surface']"
```

Steps: `dive D` (m), `heading A` (deg, 0 = start heading, + = left), `move M`
(m forward along the held heading), `goto X Y` (m in the start frame; `goto 0 0`
is return-to-origin), `roll N` (full turns), `wait S`, `surface`. A step is
done after `settle` s inside tolerance (20 cm / 10 cm / 5 deg); a step that
takes longer than `step_timeout` stops the mission and the sub parks. Position
is logged every 2 s. Verified in sim: the six-step mission above completes in
~40 s and ends within 4 cm of ground truth.

## Running on the vehicle
The master launcher's `mode:=real` starts the real ZED stack and includes
`vehicle.launch.py` for the same movement executables used
by simulation. Real pressure/DVL adapters and the ZED IMU adapter remain
missing. Without sensor inputs, the estimator waits and automatic hold stays
idle. Manual joystick control sends body-relative efforts directly, with
`actuators:=teensy_usb` enabling M5 motor output. Follow the
[controller setup](../docs/software/gamepad.md) for the launch and HTML page.
There is no robot startup service yet.
Use the vision Python environment and interpreter command in the runtime guide.

Movement tuning defaults live in
`src/robosub_control/robosub_control/config/constants.py`. Current measured mounts and M5
settings live in `current_m5.py`; saved future T200 settings live in
`future_t200.py` in the same folder. The master defaults to
`vehicle:=current_m5`; select `vehicle:=future_t200` for the planning profile.
ROS parameters can override the
startup-only defaults at startup. Tunable PID/filter values use code defaults
unless `tuning:=true`; the developer menu applies changes during execution.
Controller, mission, DVL and mixer timers call
their respective `periodic()` methods; `on_*()` methods handle incoming
messages. Vision settings remain in its existing YAML configuration.

## Gotchas already handled
Each of these cost a debugging cycle. They are in the world file as comments too.

- **No quadratic ANGULAR damping** (`kPP`/`mQQ`/`nRR`). Gazebo integrates
  hydrodynamics explicitly; once the hull gains angular rate those terms
  diverge and the sim dies with
  `ODE INTERNAL ERROR 1: assertion "aabbBound >= dMinIntExact"`.
  Strong *linear* angular damping (`kP`/`mQ`/`nR`) keeps rotation bounded.
  Symptom to recognise: sim is fine at 5 N thrust, dies within 1 s at 20 N.
- **An IMU needs `gz-sim-imu-system` in the world.** The `Sensors` system only
  drives *rendering* sensors (camera, lidar). Without the IMU system plugin the
  sensor exists, the topic exists, and nothing is ever published. Same for the
  altimeter (`gz-sim-altimeter-system`).
- **`ros2 topic hz` measures wall-clock rate.** If the real-time factor drops
  below 1, `/imu/data_raw` reads low on the wall clock (~65 Hz seen) while the
  header stamps still show exactly 100 Hz of sim time. Judge sensor rates by
  stamps (or `gz topic`), not by `hz`. RTF < 1 here has meant the container
  lost the GPU (camera rendering on llvmpipe); check `nvidia-smi` shows
  `gz sim server`. With the GPU the sim holds RTF 1.0 headless and with GUI.
- **Two sims on one box double every topic.** Gazebo transport and ROS are
  shared across containers (host networking), so a second `gz sim` makes the
  bridge see 2x messages and a second bridge makes ROS see 4x. Check
  `ros2 topic info /imu/data_raw` reports one publisher before trusting rates.
- **Avoid `<plane>` collision geometry.** Unbounded AABBs are a known ODE
  hazard; the pool floor is a large box. (Not what caused the crash above, but
  worth keeping.)
- Sub is trimmed **slightly positively buoyant** (39.5 kg displacing 40 kg), so
  a failure floats it instead of sinking it.

## Earlier simulation measurements

These measurements belong to the earlier world/mixer template. They have not
been repeated for the vehicle profiles and are not physical M5 measurements.
| Signal | Rate |
|---|---|
| `/imu/data_raw` | 100 Hz (sim time) |
| `/altimeter`, `/depth` | 50 Hz (sim time) |
| `/sub/odom` | 50 Hz |
| `/sub/camera/image_raw` | 10 Hz on llvmpipe; much higher once the GPU driver is fixed |

Cruise speed under full forward thrust: **~1.3 m/s**.

Open-loop `cmd_vel` drifts laterally over time. That is not a bug to fix in the
model -- it is why you need closed-loop heading control, which is Phase 3.
