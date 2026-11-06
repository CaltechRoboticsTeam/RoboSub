# Teensy M5 USB bridge

Current path: Jetson USB → Teensy 4.1 Serial1 → RS-485 → six VideoRay M5s.
Motor IDs: **FR=0, FL=1, BR=2, BL=3, RD=4, LD=6**; slots 5 and 7 stay neutral.
USB driving bypasses PX4 arming. See the [driving guide](https://github.com/CaltechRoboticsTeam/RoboSub/blob/main/deploy/jetson/README.md).

## Build

```bash
arduino-cli core update-index \
  --additional-urls https://www.pjrc.com/teensy/package_teensy_index.json
arduino-cli core install teensy:avr@1.62.0 \
  --additional-urls https://www.pjrc.com/teensy/package_teensy_index.json
./firmware/teensy_usb_m5/build.sh rs485-serial1
```

Output: `firmware/build/rs485-serial1/teensy_usb_m5.ino.hex` and SHA-256 manifest.
Building/deploying does not flash. The Serial1 image transmits neutral on
boot; flashing requires the operator's explicit action. The default build
without `rs485-serial1` is USB-only and cannot drive motors.

## Terminal passthrough

Build the separate terminal image for the existing Serial1 wiring:

```bash
./firmware/teensy_usb_m5/build.sh passthrough-serial1
```

Output: `firmware/build/passthrough-serial1/teensy_usb_m5.ino.hex`.
This only compiles; installing it requires a separate, authorized flash.
It replaces normal driving/depth firmware and sends no automatic motor
packets, including on boot or USB disconnect. Restore the `rs485-serial1`
image after configuration, before using ROS again.

1. Stop ROS driving and diagnostics. With thruster power off, isolate the
   target thruster from the others on RS-485; passthrough cannot select motor 6.
2. With the terminal image installed, open the Teensy's USB serial port in
   PuTTY: **115200, 8N1, no flow control**, character-at-a-time input. Enable
   terminal session logging to capture the configuration. On the Jetson the
   port is `/dev/serial/by-id/usb-Teensyduino_USB_Serial_19620450-if00`.
3. Type `M5-PASSTHROUGH` and press Enter. Wait for `Forwarding at 115200 8N1`.
   This local command enables forwarding; it is not sent to the thruster.
4. Keep the Teensy on USB, power-cycle the isolated thruster, then type
   `+++++` within five seconds. On its diagnostic menu, press `c` to enter
   configuration. **Number keys on the first menu immediately run the motor.**
   See VideoRay's [entry procedure](https://download.videoray.com/documentation/m5_thruster/html/diagnostics_test_mode.html)
   and [menu behavior](https://download.videoray.com/documentation/m5_thruster/html/motor_activation_mode.html).
5. Capture motor 6's menu and compare it with a working thruster's menu before
   changing or saving settings. Send one command at a time, waiting for replies.

**Ctrl-]** or closing USB disables forwarding, but does **not** stop the motor
or leave its diagnostic mode. Normal motor watchdog commands are absent in
this build. Power off the thruster before closing the session. A new USB
session requires `M5-PASSTHROUGH` again. Do not paste normal `H1`/`C1` commands
while forwarding is enabled.

The receiver is disabled during transmission to suppress local echo. The
UART interrupt releases DE at the final stop bit; a bounded wait re-enables
reception without calling the USB event processing in `flush()`.

For binary tools, enable with `M5-BINARY` instead. Every subsequent byte,
including Ctrl-], is forwarded unchanged. Closing USB ends this mode;
Ctrl-] does not. `tools/diagnostics/m5_probe.py` uses it for read-only voltage
requests to all individual Node IDs, including 29. Preview is the default;
`--execute` opens the device and sends the requests. Power-cycle the isolated
motor and wait five seconds before a probe, without entering its text menu.

## Diagnostics

Real launches publish `diagnostic_msgs/DiagnosticArray` on `/diagnostics`,
under `sub/Diagnostics`, and show the same rows in the driver station.
Depth, raw pressure ADC, MAVROS/Pixhawk, DVL input, Teensy, command flow and
each named motor have separate status. Derived depth and DVL velocity
do not independently prove pressure calibration or a connected DVL.

The Serial1 **v3** image captures CRC-checked M5 replies and cycles reply
requests through individual Node IDs 0–127 without changing motor powers.
At the normal 20 Hz USB command rate, a scan takes about 6.4 seconds.
The ROS driver reads CSR `0x4c` to associate each responding node with its
stored motor assignment. Reads write no configuration and do not renew
the motor-command watchdog. Allow 15 seconds for discovery; silence means
unconfirmed, not a proven wiring fault. Duplicate motor assignments and
reported M5 faults are errors. A detected assignment still needs to match
the physical motor location. No response can reveal two devices sharing
the same Node ID reliably.

The ROS driver requires firmware v3. Building does not flash the Teensy.

On the Jetson, with the normal mission stopped:

```bash
cd ~/robosub-field/current
./teensy.sh neutral
./teensy.sh neutral --seconds 5 --execute
./teensy.sh pulse --slot 0 --delta-us 25
```

Commands without `--execute` are previews. Adding it to a pulse drives the
selected motor. Use submerged thrusters, keep people clear, and have power
removal available. `--delta-us` accepts ±500, equivalent to ±1 normalized
M5 command; these values are packet fields, not physical PWM pulses.
`--ramp` sets ramp seconds; `--hold` accepts seconds or `forever`.
**Ctrl-C** sends neutral immediately. Never run diagnostics alongside driving.

An ACK proves USB acceptance; transmit counters prove writes. Neither proves
motor movement. `--require-feedback` additionally requires fresh, CRC-valid,
fault-free motor replies.

## Protocol

CRC32-framed ASCII: `H1` capabilities, `C1` commands, `I1` capabilities reply,
`A1` acknowledgment, `D1` pressure/UART telemetry, `F1` fault, `R1` receive
capture, `Q1,session,node` motor identity requests and `K1,session,node`
USB acknowledgments. These acknowledgments are separate
from the motor's captured CSR reply. Commands carry session IDs and
increasing sequence numbers.
A 250 ms watchdog or USB disconnect sends neutral; re-enabling requires a
fresh neutral command. Pressure calibration is preserved from the supplied
sketch; ADC values ≤1 are unavailable.

The supplied wiring/settings use DE=3, RE=2, A0 pressure, 115200 baud,
group address `0x81`. The firmware scans Node IDs 0–127 for replies.
[VideoRay command reference](https://download.videoray.com/documentation/m5_thruster/html/commands_responses.html).

## Collect depth calibration data

The Teensy already includes raw ADC counts in `D1` at 10 Hz. The ROS driver
publishes them unchanged on `/sub/sensors/pressure_adc` (`std_msgs/msg/Int32`),
including 0 and 1 for diagnosis. `/sub/sensors/depth_usb` still uses the
unverified fixed calibration. This change requires a rebuilt ROS package,
not a firmware flash. Preview mode leaves USB closed and supplies no ADC data.

During a normal recorded run, leave the driver station disabled. Measure
depth from the water surface to the pressure sensor port, in metres,
positive down. Use at least three distinct known depths across the intended
range, including near the surface, and repeat points while coming back up.
Wait for the sensor to settle at each depth.

In another Jetson terminal using the same ROS environment, publish the
**measured** depth while holding still (replace `0.50`):

```bash
ros2 topic pub --rate 1 /sub/calibration/depth_reference \
  std_msgs/msg/Float64 '{data: 0.50}'
```

Hold for 10–20 seconds, then **Ctrl-C this publisher before moving**. Restart
it with the next measured depth once settled. These messages are reference
labels only; they do not command the robot. Only ADC samples between the
first and last reference messages of each hold belong to that measurement;
do not carry a reference value forward through a gap.

The normal run recorder captures both topics automatically in `bag/*.mcap`.
Send all MCAP segments from the calibration run, or open them in Foxglove
and add these two series to a Plot panel:

```text
/sub/sensors/pressure_adc.data
/sub/calibration/depth_reference.data
```

Use **Log time** for both series; their timestamps are receipt times, not
ADC acquisition times. Right-click the plot to
[download CSV](https://docs.foxglove.dev/docs/visualization/panels/plot#download-csv-data).
Send that CSV with both series, the measured-depth uncertainty, and whether
each hold was on the way down or up. No smoothing or calculated-depth series
is needed. This lets us compare ADC means, spread and repeatability at each
known depth before fitting or applying new coefficients.
