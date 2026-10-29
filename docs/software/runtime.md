# Robot startup and tuning

Start with `robosub_ws/src/robosub_bringup/launch/robot.launch.py`. It selects
the I/O environment and includes smaller launch descriptions. Both modes run
the same `state_estimator`, `depth_heading_controller`, `thruster_mixer` and
`actuator_commands` executables through `robosub_sim/launch/vehicle.launch.py`.

## Build and run

In a ROS Humble environment, from the repository root:

```bash
cd robosub_ws
colcon build --symlink-install --packages-select robosub_sim robosub_bringup
source install/setup.bash
ros2 launch robosub_bringup robot.launch.py mode:=sim
```

Simulation also needs the existing Gazebo Harmonic and bridge dependencies.
For real vision, first install/build the ZED stack using the
[vision setup](vision.md). Source both workspaces if they are built separately.
Activate its Python environment and invoke the ROS CLI through that Python
interpreter so launch-time ZED checks can import the SDK binding:

```bash
python3 /opt/ros/humble/bin/ros2 launch robosub_bringup robot.launch.py mode:=real
```

Real mode starts the physical ZED wrapper/perception plus shared movement
programs. It defaults to the uncorrected, no-target `development.yaml`. Use
`vision_config:=/absolute/path/to/vision.yaml` for a calibrated target setup,
or `vision:=false` to omit the camera.

Manual M5 movement is available with `actuators:=teensy_usb` and teleop
enabled; see [Xbox driving](gamepad.md). Pressure/DVL adapters, measured
sensor transforms and the ZED-to-estimator IMU adapter are still missing. The movement estimator waits for its inputs. Real autonomous
mission startup is rejected until those drivers are integrated.

| Argument | Default | Meaning |
| --- | --- | --- |
| `vehicle` | `current_m5` | Current M5 or saved `future_t200` profile |
| `mode` | `sim` | `sim` or `real` |
| `gui` | `true` | Gazebo window; ignored in real mode |
| `vision` | `auto` | ZED in real mode; no camera in sim |
| `vision_config` | development config | Optional absolute vision YAML path |
| `mission` | `false` | Start the existing mission in simulation |
| `tuning` | `false` | Enable live numeric parameters and the developer menu |
| `teleop` | `off` | Xbox input: `browser` on Mac, or `local` on Linux |
| `teleop_scale` | `0.5` | Maximum normalized Xbox effort, greater than 0 through 1 |
| `actuators` | `preview` | `teensy_usb` enables live M5 output in real teleop |
| `teensy_device` | Teensy serial 19620450 by-id path | USB device for live M5 output |
| `joy_device` | `0` | SDL controller index for local input |
| `tuning_file` | empty | Reload exported values; requires tuning enabled |
| `tuning_port` | `8765` | Local developer-menu port on the robot |
| `control_rate` | `20.0` | PID callback frequency, 1-1000 Hz |
| `output_rate` | `100.0` | Command publication cap, limited to control rate |

Simulation bridges `/clock`; sensors, movement and mission use simulation
time. Real mode uses the system clock. A launch file starts programs; each
program's ROS executor dispatches its timer and message callbacks.

The shared command stage selects M5 normalized power or T200 PWM by vehicle
profile. `vehicle:=current_m5` is the default; `vehicle:=future_t200` selects
the saved future settings. The T200 path converts force to pulse widths. In
simulation its modeled applied forces feed Gazebo; in real mode it publishes
a numeric preview until custom-board transport is implemented. See
[vehicle profiles and thruster commands](thrusters.md) for measured mounts,
M5 interface details and the separately saved T200 voltage/channel/PWM
settings.

## Live tuning from a Mac, Windows or Linux computer

On the robot, or in the simulation environment:

```bash
ros2 launch robosub_bringup robot.launch.py mode:=sim tuning:=true
```

On the Mac, forward the developer menu through SSH:

```bash
ssh -N -L 8765:127.0.0.1:8765 crt@100.95.114.100
```

Open `http://127.0.0.1:8765` in a browser. The page has numeric fields,
Apply/Refresh buttons and YAML export. Tuning values refresh manually; Xbox command previews refresh while that
panel is enabled.
The server binds to robot loopback; the Mac needs SSH and a browser, not ROS.
[The Xbox guide](gamepad.md) adds controller input and live command previews
to this same menu using `teleop:=browser` or `teleop:=local`. Xbox mode
disables automatic hold outputs; `teleop:=off` restores the normal controller.
For a simulation running locally, open that same address without a tunnel.

PID gains, position output limit, integral limit, roll feedforward/rate and
estimator filter gains/time constants are exposed. Bounds, types and finite
values are checked. Each Apply updates one node atomically. Successful
changes are recorded in ROS parameter events and that node's log. Updates
to multiple nodes are separate transactions.

Exported YAML contains the last loaded/applied values, not unsent edits.
Reload it with:

```bash
ros2 launch robosub_bringup robot.launch.py mode:=sim tuning:=true \
  tuning_file:=/absolute/path/to/robot-tuning.yaml
```

Without a tuning file, restart returns to code defaults. To hard-lock every
tunable number, set `LOCK_TUNING_DEFAULTS = True` in
`robosub_sim/robosub_sim/constants.py`. This overrides launch options, ignores
numeric tuning overrides, makes the values read-only and creates no tuning
subscription. Normal `tuning:=false` also fixes tunable numbers to defaults.

Rate, timeout and mounting parameters are startup-only. Changing these at
runtime would require rebuilding timers or other state, so they are not live
tunable fields. Hardware calibration and thrust limits are not exposed.

## Register another number

Create one registry per node and register numbers during construction:

```python
from robosub_sim.tuning import TunableNumbers

self.tuning = TunableNumbers(self)
self.gain = self.tuning.number("gain", 1.0, minimum=0.0)
value = self.gain.get()
```

`get()` only reads a cached value. To update an existing PID directly:

```python
self.gain.bind_to(self.pid, "kp")
```

The registry validates proposed updates without changing control state.
After ROS confirms a committed update, its event callback updates cached
values and bound attributes. There is no network lookup in the PID loop.

## Higher loop rates

```bash
ros2 launch robosub_bringup robot.launch.py mode:=sim \
  control_rate:=1000.0 output_rate:=100.0
```

This requests a 1 ms PID interval while limiting command publication to
100 Hz. The command stage repeats the latest command at that capped rate.
It does not send six thruster messages for every PID calculation. The PID
uses elapsed clock time, not an assumed 1 ms. Changing the callback rate does
not increase sensor update rates or guarantee millisecond deadlines. Python,
ROS scheduling and other workloads can delay execution. The current M5
profile has a 100 Hz software output ceiling pending bus verification. The
future T200 profile allows the Basic ESC R3 maximum of 400 Hz. Higher output
rates are rejected for the selected profile.

Measure the callback separately, with synthetic state and remapped output:

```bash
python3 tools/benchmark_control_loop.py --rate 1000 --seconds 5
```

This probe uses an isolated local ROS domain and starts no hardware driver.
Its results do not establish timing with vision/GPU load or physical I/O.

Run the numeric, ROS parameter-service and launch-routing checks with:

```bash
python3 -m unittest discover -s tests/control -v
```

ROS-dependent checks skip when Humble is unavailable. Runtime tests isolate
their ROS domain and remap controller output away from actuator topics.
