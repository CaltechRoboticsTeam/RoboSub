# Xbox controller

Use the master launcher's `teleop` option. Both input methods run the same
Xbox mapping, mixer and selected vehicle command stage.

| Stick | Robot-relative movement | Positive ROS body axis |
| --- | --- | --- |
| Left up / down | Forward / backward | +X forward |
| Left left / right | Strafe left / right | +Y left |
| Right left / right | Turn left / right | +yaw left |
| Right up / down | Rise / descend | +Z up |
| RT / LT | Roll right / left | +roll right |

Translation follows the robot's heading, like robot-relative FRC swerve.
These are normalized effort commands, not requested speeds or depth targets.
Start with both sticks centered, triggers released and RB released.
**Press RB once to enable movement; press it again to disable movement.**
RT rolls right; LT rolls left, proportional to trigger squeeze. Equal trigger
inputs cancel. Releasing RB leaves movement enabled. The default maximum effort is 50%. Set
`teleop_scale:=1.0` for the full normalized range, or a smaller positive value
for a lower limit. Deadzone, timeout, button mapping and defaults live in
`robosub_control/robosub_control/config/constants.py`.

## Controller on the Mac, connected to the Jetson

Pair a compatible Xbox controller in macOS Bluetooth settings or connect it
by USB if that model is supported by macOS. The Mac only needs SSH and a
browser with standard Gamepad API support; ROS runs on the Jetson.

Deploy from the Mac repository root when the source changes:

```bash
./deploy/jetson/deploy.sh crt@100.95.114.100
```

For driving, put the robot in the water, keep people clear of the thrusters,
and have propulsion power removal available. Pair the controller to the Mac.
Stop any existing joystick launch or USB diagnostic before starting another.

In Mac Terminal 1, connect to the Jetson and start the live driver:

```bash
ssh crt@100.95.114.100
cd ~/robosub-field/current
./joystick.sh --execute teleop_scale:=0.5
```

Leave that terminal running. This enables the Jetson USB -> Teensy Serial1 ->
M5 path using the existing firmware. It bypasses PX4 arming. `teleop_scale`
caps both each body effort and the final motor power; combined outputs are
scaled together when needed. Start lower if desired, for example `0.2`.
A small command may be below a motor's starting threshold.

In a second Mac terminal, forward the HTML page:

```bash
ssh -N -L 8765:127.0.0.1:8765 crt@100.95.114.100
```

Open [http://127.0.0.1:8765](http://127.0.0.1:8765) in Chrome or another
browser supporting standard gamepads. Keep both terminals running.
The HTML is served by ROS; do not open `tuning.html` as a local file.

1. Keep the page focused and press a controller button to make it appear.
2. Select the controller and click **Enable controller input**.
3. Center both sticks and release RB once.
4. Press **RB** once, then move the sticks to drive relative to the robot's front.
   RT rolls right and LT rolls left.
5. Press **RB** again or click **Disable controller input** to request stop. Use Ctrl-C in Terminal 1 to stop the stack.

M5 motor IDs are **0=FR, 1=FL, 2=BR, 3=BL, 4=RD, 6=LD**.
The page shows body efforts and the actual USB-acknowledged motor powers after
polarity, cap and integer conversion. An ACK confirms Teensy acceptance,
not measured thrust or motor stopping. Verify motor directions in water
before sustained driving; polarity and power-to-thrust calibration remain
unmeasured. Releasing the sticks requests zero effort, not station keeping.

For software-only previews, omit `--execute`:

```bash
./joystick.sh teleop_scale:=0.5
```

Without the deployment wrapper, the equivalent live ROS command is:

```bash
ros2 launch robosub_bringup robot.launch.py mode:=real vision:=false \
  teleop:=browser tuning:=true actuators:=teensy_usb teleop_scale:=0.5
```

Use the localhost address through SSH: the Gamepad API requires a secure
context, and localhost qualifies. The menu stays bound to robot loopback.
No public port or ROS installation on the Mac is needed.

## Change M5 motor IDs yourself

On the Mac, edit:

```text
robosub_ws/src/robosub_control/robosub_control/config/current_m5.py
```

Change the numbers in `M5_MOTOR_IDS`:

```python
M5_MOTOR_IDS = {"FR": 0, "FL": 1, "BR": 2, "BL": 3, "RD": 4, "LD": 6}
```

Use a different integer from 0 through 7 for each motor. Unassigned slots
stay neutral. These numbers select existing motor IDs; editing them does
not reprogram the IDs stored inside the thrusters. Leave `THRUSTER_NAMES`
and the geometry alone when changing motor IDs.

Stop driving and press Ctrl-C in the Jetson launch terminal. From the Mac
repository root, deploy:

```bash
./deploy/jetson/deploy.sh crt@100.95.114.100
```

Then restart on the Jetson and refresh the browser page:

```bash
cd ~/robosub-field/current
./joystick.sh --execute teleop_scale:=0.5
```

The page labels show the configured motor IDs, including **6:LD**. Mapping
changes need a process restart; refreshing the page alone cannot apply them.
No Teensy firmware rebuild or flashing is needed for IDs 0 through 7.

## Controller on the Linux simulation computer

Outside a container, install the ROS driver if needed:

```bash
sudo apt install ros-humble-joy
ros2 run joy joy_enumerate_devices
```

For the existing Linux Docker workflow, rebuild the image once to include
`joy`, then start a fresh container with input access:

```bash
cd robosub_ws
# Stop/exit an existing robosub container before enabling its device access.
docker compose -f docker/compose.yaml build
ROBOSUB_GAMEPAD=1 ./run.sh
```

The optional Compose override mounts `/dev/input` read-only and allows Linux
input devices. SDL reads event devices, so passing only `/dev/input/js0` is
insufficient. The host user/rootless Docker setup must already have permission
to read the controller's event device. Do not change permissions on unrelated
input devices to work around a controller permission problem.

Inside the ROS environment:

```bash
colcon build --symlink-install --packages-select \
  robosub_control robosub_sim robosub_bringup
source install/setup.bash
ros2 launch robosub_bringup robot.launch.py mode:=sim teleop:=local tuning:=true
```

The ROS `joy/game_controller_node` uses SDL's controller database to provide a
consistent layout. `joy_device:=0` selects its first device; use the enumeration
output to choose another index. This path does not require a browser to drive.
Open the developer menu for read-only command previews and tuning. Controller
input goes directly from the local ROS driver to the shared mapping code.

For a controller on the Mac while sim runs on Linux, use
`mode:=sim teleop:=browser` on Linux, and SSH-forward its menu just like the
Jetson example. This also avoids USB device forwarding into the container.

## Execution and loss of input

```text
Mac browser Gamepad API -> SSH / HTTP -> Joy input
                  OR
Linux SDL game_controller_node -> Joy input
  -> XboxTeleop.on_joy() caches validated input
  -> XboxTeleop.periodic() at 20 Hz, using wall time
  -> /sub/cmd_vel normalized body effort
  -> geometry mixer -> M5 power / T200 PWM preview
                      -> simulation-only force -> Gazebo
  -> optional teensy_actuators -> USB -> Serial1 -> M5
```

The mapper validates four stick axes and two triggers, and applies a
continuous 8% deadzone. Translation
magnitude is bounded on diagonals. Browser and ROS SDL layouts have different
axis signs and RB button indices; the selected source handles that difference.
The generic `joy_node` has device-dependent ordering and is not substituted
for `game_controller_node`.

Xbox mode gives the command topic to manual control. Automatic hold and
estimation nodes are omitted, and launch rejects `mission:=true`
with teleop enabled. The old surge/sway `/sub/cmd_vel_manual` interface remains
available in normal `teleop:=off` operation. To use hold/mission mode, restart
with teleop off. Releasing the sticks requests zero effort, not position hold;
buoyancy, inertia and currents can still move the submarine.

Movement disables on another RB press, invalid joystick data or a 0.25-second input timeout
plus scheduling delay. All of these reset the toggle; reconnecting cannot
resume movement until controls are neutral and RB is pressed again. The watchdog uses monotonic wall time even if simulation
is paused. Reconnection requires neutral sticks/triggers and RB released, then a new press. The Mac
page also disables on lost focus, hidden page, controller disconnect, or HTTP
failure. A sequence check discards delayed input older than a stop; one browser
holds a short input lease to prevent two pages from driving simultaneously.
The mixer/command stage retain their independent timeouts. The USB driver
uses a 0.25-second wall-time target timeout and requires a fresh neutral frame
to resume. Bad target layouts stop output; a USB failure shuts down launch.
The existing Teensy firmware independently times out at 250 ms. Physical
power removal remains the stop method if the command path fails.

Sources: [ROS joystick driver and mapping](https://github.com/ros-drivers/joystick_drivers/tree/ros2/joy),
[Gamepad API](https://developer.mozilla.org/en-US/docs/Web/API/Gamepad_API/Using_the_Gamepad_API),
[standard gamepad layout](https://w3c.github.io/gamepad/#remapping).
