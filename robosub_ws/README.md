# robosub_ws — AUV simulation environment

ROS 2 **Humble** + Gazebo **Harmonic**, in a container, so simulation code and
Jetson code are the same code.

## Why Docker
This desktop runs Ubuntu 24.04, whose native ROS 2 is **Jazzy**. The sub's
Jetson runs **Humble** (JetPack 6 / Ubuntu 22.04). Running Humble in a
container here means a node that works in sim works on the vehicle — no
"it worked on my laptop" at competition.

## Quick start
```bash
./run.sh                      # build/enter the container
colcon build --symlink-install
source install/setup.bash
ros2 launch robosub_sim pool.launch.py
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

## Browser view (gzweb)
To watch the sim without X11 (headless box, remote machine, teammates), start
gz-launch's websocket server alongside it:
```bash
ros2 launch robosub_sim pool.launch.py gui:=false web:=true
```
Open <https://app.gazebosim.org/visualization> (the official gzweb client) and
connect to `ws://localhost:9002`. Meshes are streamed over the same socket.

The page is served over HTTPS, so browsers allow a plain `ws://` connection
only to `localhost`. To view from another machine, tunnel the port:
```bash
ssh -L 9002:localhost:9002 <user>@<sim-host>
```
The server has no authorization key; anyone who can reach port 9002 can watch.
The plugin ships with `gz-harmonic` (`libgz-launch7`), so the image needs
nothing extra.

## Layout
| Path | What it is |
|---|---|
| `docker/Dockerfile` | Humble + Harmonic + bridge + tools |
| `docker/compose.yaml` | GPU, X11, host networking, volume mounts |
| `src/robosub_sim/worlds/pool.sdf` | Pool, water, sub, gate |
| `src/robosub_sim/config/bridge.yaml` | Gazebo topics <-> ROS topics |
| `src/robosub_sim/config/websocket.gzlaunch` | gzweb websocket server (port 9002) |
| `src/robosub_sim/robosub_sim/thruster_mixer.py` | Control allocation |
| `src/robosub_sim/robosub_sim/depth_sensor.py` | Altimeter -> `/depth` (stand-in for the pressure sensor driver) |
| `src/robosub_sim/robosub_sim/dvl_sensor.py` | Ground truth -> `/dvl/velocity`, `/dvl/altitude` (stand-in for the Pathfinder DVL driver) |
| `src/robosub_sim/robosub_sim/state_estimator.py` | `/imu/data_raw` + `/depth` + `/dvl/velocity` -> `/sub/state` (attitude, depth, x/y) |
| `src/robosub_sim/robosub_sim/depth_heading_controller.py` | PID depth + heading + position hold -> `/sub/cmd_vel` |
| `src/robosub_sim/launch/vehicle.launch.py` | What runs on the Jetson: drivers -> estimator -> controller -> mixer (no Gazebo) |
| `src/robosub_sim/launch/pool.launch.py` | The sim: Gazebo + bridge + sensor stand-ins + `vehicle.launch.py` |
| `src/robosub_sim/robosub_sim/dashboard.py` | Live terminal view: sensors | estimator | commands | thrust, with rates |
| `src/robosub_sim/robosub_sim/mission.py` | Scripted missions: `dive`, `heading`, `move`, `goto`, `roll`, `wait`, `surface` |

## Topics
| ROS topic | Type | Direction |
|---|---|---|
| `/sub/cmd_vel` | `geometry_msgs/Twist` | you -> mixer |
| `/sub/thrust/t{1..6}` | `std_msgs/Float64` | mixer -> sim |
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

Roll is held against the sub's own righting moment (~20 N.m at 90 deg). The
mixer puts full thrust behind roll (~35 N.m), a feedforward term cancels the
righting torque at every angle, and the integral mops up the rest, so any
angle holds. A barrel roll cannot be a setpoint (angles wrap), so:

```bash
ros2 topic pub -1 /sub/setpoint/roll_turns std_msgs/msg/Float64 '{data: 1.0}'   # one 360, then level
```

During the roll the depth and heading outputs are scaled by cos(roll), so an
inverted sub still pushes the right way and one on its side does not fight.
Measured: a 360 takes ~10 s at 0.6 rad/s, peak effort 0.84, depth stays within
3 cm, pitch/yaw wander < 8 deg while inverted and recover on their own.

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
ros2 run robosub_sim mission                      # dive 1.5, forward 3 m, back to origin
ros2 run robosub_sim mission --ros-args -p "mission:=['dive 2','heading 90','move 2','roll 1','goto 0 0','surface']"
```

Steps: `dive D` (m), `heading A` (deg, 0 = start heading, + = left), `move M`
(m forward along the held heading), `goto X Y` (m in the start frame; `goto 0 0`
is return-to-origin), `roll N` (full turns), `wait S`, `surface`. A step is
done after `settle` s inside tolerance (20 cm / 10 cm / 5 deg); a step that
takes longer than `step_timeout` stops the mission and the sub parks. Position
is logged every 2 s. Verified in sim: the six-step mission above completes in
~40 s and ends within 4 cm of ground truth.

## Running on the vehicle
`vehicle.launch.py` is the Jetson launch: it starts the mixer, estimator and
controller and has commented slots for the four drivers (IMU, pressure, DVL,
thrusters) with the exact topic contracts each must meet. `pool.launch.py`
includes it, so the sim always exercises the same nodes the sub runs. The
drivers are the only code that differs between sim and vehicle.

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

## Measured behaviour (verified, software rendering)
| Signal | Rate |
|---|---|
| `/imu/data_raw` | 100 Hz (sim time) |
| `/altimeter`, `/depth` | 50 Hz (sim time) |
| `/sub/odom` | 50 Hz |
| `/sub/camera/image_raw` | 10 Hz on llvmpipe; much higher once the GPU driver is fixed |

Cruise speed under full forward thrust: **~1.3 m/s**.

Open-loop `cmd_vel` drifts laterally over time. That is not a bug to fix in the
model -- it is why you need closed-loop heading control, which is Phase 3.
