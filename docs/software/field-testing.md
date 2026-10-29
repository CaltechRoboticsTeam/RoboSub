# Current-submarine field tests

These tools observe or exercise the existing PX4 command path. The current
robot has six M5 thrusters. The Teensy is connected to PX4 over UART and now
also to the Jetson over USB; USB enumeration alone does not establish its
motor-bus wiring. See the [USB bridge guide](teensy-usb.md) for bypassing the
PX4-to-Teensy command link. The new ROS mixer remains a separate preview path
until physical motor-ID mapping and integration are complete.

## Deployment and passive checks

From the Mac repository root:

```bash
./deploy/jetson/deploy.sh crt@100.95.114.100
```

The deployment creates a versioned folder under `~/robosub-field/`, verifies
file hashes, builds the runtime and runs isolated tests. It preserves the old
`~/Robosub` checkout, actual legacy sources, and running MAVROS/DVL programs.
It does not flash firmware, open serial ports, change parameters or start
another MAVROS connection.

On the Jetson:

```bash
cd ~/robosub-field/current
./field.sh status --seconds 10
```

The report includes:

- Connection, arming state, mode, cached autopilot/firmware and configuration.
- Actual sample rates and last samples for IMU, DEPTH, DVL odometry and servo
  output; missing samples remain missing rather than being called healthy.
- Raw MAVLink message 36 (`SERVO_OUTPUT_RAW`) grouped by its port number.
- Eight protocol output slots, observed minima/maxima and the receiver's
  expected
  normalization `(value - 1500) / 500`. A zero servo value is invalid, not
  neutral; the old unchecked calculation would turn it into -3.
- Competing ROS manual-control/RC-override publishers and FCU status messages.

Reports are written to `~/robosub-field/logs/`. Values such as unknown battery
voltage and invalid DVL values remain JSON `null`. Reported PWM-like numbers
are FCU outputs, not confirmation that the receiver or a physical motor moved.

## Review a low-input pulse without transmitting

```bash
./field.sh pulse --seconds 3 --axis throttle --effort 0.05 --hold 0.5 --ramp 0.25
```

This is a **dry run**. It reads telemetry/cache, checks the current assumptions
and prints a plan. No control publisher or arming request is created.
The legacy throttle axis uses `MANUAL_CONTROL.z = 500 + 500 * effort`, so a
5% pulse has a peak input of 525, with neutral at 500. The yaw input uses
`r = 1000 * effort`, neutral zero; all other inputs remain neutral.
These are stick fractions, not calibrated thruster power or physical speeds.
Axis meaning depends on the installed PX4 firmware and mixer. Verify which
outputs/motors respond before calling the throttle input physical forward.

The helper is specific to the observed PX4 submarine airframe 60002. It refuses
another autopilot/airframe, non-MANUAL mode, an already-armed vehicle, stale
state/output telemetry, invalid or non-neutral servo values, or competing ROS
manual-control/override publishers. The test does not change mode, parameter
values, safety-switch state, firmware or calibration. Other ground stations
on separate MAVLink links cannot be detected by this ROS publisher check;
only the test operator should have control during a pulse.

## Prepare servo telemetry before executing a pulse

The live link initially reported servo outputs at about 1 Hz. A one-second
pulse cannot be monitored reliably at that rate. The pulse tool therefore
requires at least 5 Hz observed servo telemetry and a sample age below 0.75 s.
An operator can request message 36 at 10 Hz on the existing Jetson link:

```bash
source ~/robosub-field/current/env.sh
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval \
  "{message_id: 36, message_rate: 10.0}"
cd ~/robosub-field/current
./field.sh status --seconds 10
./field.sh pulse --seconds 3
```

This service transmits a telemetry-rate request to the Pixhawk. It is not run
by deployment or passive status. Confirm fresh outputs in the report rather
than relying only on its success response. This request affects the connected
Jetson link; the receiver's separate link still needs verification. The tool
does not loosen its output guard when telemetry is insufficient.

## Execute only with an operator at the submarine

For movement testing, verify the receiver path, keep thrusters in water,
secure the vehicle, keep hands clear, and have propulsion power removal ready.
FCU output telemetry alone does not establish the downstream RS-485/M5 path.

```bash
./field.sh pulse --seconds 3 --axis throttle --effort 0.05 \
  --hold 0.5 --ramp 0.25 --execute
```

**`--execute` sends neutral, requests arming, runs the pulse, sends neutral and
requests disarming.** It does not automatically switch to MANUAL. Neutral is
streamed for one second before arming; the default nonzero command envelope is
at most one second including ramps. The input is hard-capped at +/-10%, hold
time at one second, and each ramp at 0.5 seconds. Defaults are 5%, 0.5-second
hold and 0.25-second ramps. Do not replace them with the old startup note's
`throttle:=1.0`.

During the test, loss of fresh state/servo data, mode/arming changes, a competing
ROS input publisher, or reported servo output beyond 1500 +/-50 us aborts the
pulse. That output guard reacts to telemetry; it is not an electrical current
limiter or a guarantee of motor speed. SIGINT/Ctrl-C and SIGTERM request cleanup
while ROS remains alive. Cleanup waits for disarm acknowledgment/state. If it
cannot confirm disarm, the command exits with an error and tells the operator
to remove propulsion power. A killed process or a frozen receiver cannot be
made safe by a ROS cleanup handler; the receiver needs its own watchdog.

Do not increase effort just because a 5% pulse produces no rotation. The
observed `UUV_MGM_THRTL` gain is 0.5 and thrust saturation is about 0.3; the
mixer may further reduce individual outputs. A small command may be below the
motor start threshold. Use the recorded output values to distinguish no input,
no FCU output, no receiver forwarding and no motor response.

## What the live inspection established

On 2026-10-28, without arming or publishing control commands:

- MAVROS was already using `/dev/ttyTHS1:115200`; the Pixhawk reported PX4
  1.17.0, MANUAL and disarmed. This is PX4, not ArduSub.
- `SYS_AUTOSTART=60002` selects the eight-output BlueROV2 Heavy preset in
  upstream PX4. The six measured mounts in `current_m5.py` are not proof that
  this FCU mixer matches the physical vehicle. Resolve that before full
  holonomic control; this pulse deliberately tests the existing FCU path.
- The first eight reported outputs were 1500. Cached telemetry-port baud
  settings were both 115200; this matches the reference receiver's baud.
- `MAV_0_CONFIG=101`, `MAV_0_MODE=0`, `MAV_2_CONFIG=102`, `MAV_2_MODE=2` were
  observed. The two links have separate MAVLink stream configurations. A
  message seen on the Jetson link is not proof it is sent on the Teensy link.
- USB device identities include a PX4 FMU v5.x, an FTDI RS-232 adapter and,
  after connection, Teensy USB serial `19620450`. The Teensy is a 4.1. The
  older reference and supplied installed sketch disagree about which UART
  connects to RS-485; physical wiring still requires verification.

The original C++ state subscription was reproduced on the Jetson with its
default QoS and received connected state after roughly three seconds of ROS
discovery. That QoS is compatible with MAVROS. A stopped MAVROS process also
produced the all-topics-missing report. Keep MAVROS running and allow discovery
before concluding the Pixhawk is absent.

The actual Jetson source also contains `status_node.cpp` and
`thruster_check.cpp`, absent from the Mac prototype. The old thruster check
switches modes and arms automatically, with a full-stick override in
`flo_startup_file`. The DVL program sends initialization/configuration commands
when started. Preserve those running programs; do not start a duplicate just
because a topic name exists without data.

## Localize a no-thrust failure

| Observation | What it establishes / next boundary |
| --- | --- |
| No fresh `/mavros/state` | Fix Jetson/FCU communication first |
| Manual pulse runs but all servo outputs remain 1500 | Inspect FCU mode, input selection, arming and mixer; the Teensy is not yet implicated |
| Servo outputs change on the Jetson link | FCU output generation is observed on that link; verify message 36 on the receiver's actual telemetry link |
| Receiver sees message 36 but produces no RS-485 packet | Inspect receiver firmware, IDs, power and transceiver wiring |
| RS-485 packet goes out but no valid M5 response | Inspect bus power, wiring/termination, motor IDs and packet/CRC handling |
| Valid M5 response but no rotation | Inspect faults, supply/current/RPM and command magnitude; ROS success is not physical proof |

The supplied M5 order is 0=FR, 1=FL, 2=BR, 3=BL, 4=RD, 6=LD.
Verify physical motor directions before sustained driving. Deployment does
not flash a receiver. The separate USB pulse tool requires an explicit slot
selection; [the joystick guide](gamepad.md) uses the supplied six-motor map.

Sources: [MAVROS native manual-send units](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/manual_control.cpp),
[PX4 UUV control](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uuv_att_control/uuv_att_control.cpp),
[PX4 airframe reference](https://docs.px4.io/main/en/airframes/airframe_reference.html).
