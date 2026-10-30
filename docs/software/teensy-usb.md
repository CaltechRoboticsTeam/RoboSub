# Jetson to Teensy over USB

The current submarine has **six VideoRay M5 thrusters**. Its legacy
receiver took eight servo fields from PX4, converted each with
`(value - 1500) / 500`, and sent a VideoRay RS-485 packet. Those eight fields
are protocol slots, not eight physical motors. The operator-specified map is
**0=FR, 1=FL, 2=BR, 3=BL, 4=RD, 6=LD**; slots 5 and 7 stay neutral
in the joystick driver. These values do not generate electrical PWM pulses for M5s.
The separate future T200/ESC configuration is unchanged.

The new path is:

```text
Xbox -> body effort -> mixer -> Jetson USB driver -> Teensy -> RS-485 -> M5
                                 |
                 or an explicit single-slot USB diagnostic
```

PX4's UART to the Teensy is no longer a command source in the new firmware.
Direct USB commands bypass PX4 arming entirely. The master launcher enables
the joystick transport with `actuators:=teensy_usb` in real M5 teleop mode.
The default remains preview.

## Confirmed UART

The operator confirmed the TTL/RS-485 module connects to **hardware UART1**
on the Teensy 4.1. Use the `rs485-serial1` image for this submarine.
The command path is Jetson USB -> Teensy Serial1 -> TTL/RS-485 module ->
daisy-chained M5 thrusters. Direct USB tests do not depend on PX4.

DE pin 3, RE pin 2, 115200 baud, group address `0x81`,
reply node 1, A0 and the depth calibration are preserved from the supplied
sketch; none establishes the physical motor-ID mapping.

The default build remains **USB-only**: DE stays low, no UART bytes are
transmitted, and nonneutral commands are rejected. It receives on both UARTs
to report activity counters. The installed Serial1 image enables neutral
RS-485 transmission on boot. Nonzero output requires an explicit enabled
command from joystick control or an explicit USB diagnostic; no motor pulse runs automatically.

## Build without touching hardware

Install Arduino CLI and PJRC's board package on the build computer:

```bash
arduino-cli core update-index \
  --additional-urls https://www.pjrc.com/teensy/package_teensy_index.json
arduino-cli core install teensy:avr@1.62.0 \
  --additional-urls https://www.pjrc.com/teensy/package_teensy_index.json
./firmware/teensy_usb_m5/build.sh
```

This produces `firmware/build/usb-only/teensy_usb_m5.ino.hex` plus a SHA-256
manifest. It does not launch an uploader. Flashing replaces the running
sketch, including its PX4 depth-forwarding behavior. The supplied original
source is preserved at `firmware/reference/teensy-installed-2026-10-28.ino`.
No binary backup or automatic rollback of that installed image is available.

Build the confirmed Serial1 variant with:

```bash
./firmware/teensy_usb_m5/build.sh rs485-serial1
```

It produces `firmware/build/rs485-serial1/teensy_usb_m5.ino.hex` and its
manifest. This image transmits neutral on boot, so installing it is itself
a live-bus operation. Compilation does not establish motor delivery.

Use [PJRC's Teensy Loader](https://www.pjrc.com/teensy/loader_cli.html) with
`--mcu=TEENSY41` for a reviewed image. A Linux uploader must run on the Jetson
where the Teensy is plugged in. Its program button may be needed to enter the
bootloader. Do not upload to a guessed device or use a firmware-erase reset.

## Jetson tests

Deploy from the Mac using `./deploy/jetson/deploy.sh crt@100.95.114.100`.
The transfer includes firmware sources and locally built images; it does
not flash them. Then on the Jetson:

```bash
cd ~/robosub-field/current
./teensy.sh neutral
./teensy.sh neutral --seconds 5 --execute
```

The first command is a dry run and does not open USB. The second opens only
the observed Teensy by-id device, completes the protocol handshake, sends
neutral at about 20 Hz and waits for matching acknowledgments. The report
includes the bus selection, packet count, raw response-byte count, ADC/depth
and both UART receive counters. With USB-only firmware, the bus selection and
packet count must be zero. Old firmware cannot acknowledge this protocol.

After bus wiring and operator readiness are established, a single-slot test
can isolate the receiver from PX4. Keep the vehicle secured, thrusters in
water, people clear and propulsion power removal available:

```bash
./teensy.sh pulse --slot 0 --delta-us 25
./teensy.sh pulse --slot 0 --delta-us 25 --execute
```

For the requested all-on/then-off check, `--all-slots` replaces `--slot 0`.
It sends the same bounded pulse to all eight legacy packet slots, which the
six connected motors select by their configured IDs. This does not establish
their physical mapping by itself. The command is still a dry run unless
`--execute` is supplied:

```bash
./teensy.sh pulse --all-slots --delta-us 25 --hold 0.5
```

After the Serial1 image is installed and the operator is ready for movement,
the physical all-on/then-off test is:

```bash
./teensy.sh pulse --all-slots --delta-us 500 --ramp 20 --hold 1 --execute
```

This bypasses PX4 arming. The explicitly requested full-command profile is
three seconds neutral, 20 seconds ramp up, one second at +1.0 requested M5
power, then 20 seconds ramp down. The peak value is 2000 us. No physical PWM
pulse is generated: that integer is converted into a normalized power field
inside the VideoRay packet. The script then sends neutral and checks its USB
acknowledgment. Physical motor stopping is not established by that ACK.

The explicit slot is zero-based: slot 0 is FR, the first float in the legacy
VideoRay command, following the operator-specified mapping. Other slots remain
neutral. Defaults are three seconds of neutral, a three-second ramp up,
0.5-second hold, ramp down and acknowledged neutral cleanup. Negative deltas
reverse the requested power sign. Verify that the responding motor matches the supplied mapping. A 5% command may be below the motor's
starting threshold; command magnitude alone cannot establish communication.

`--ramp` sets each ramp from 0.25 to 30 seconds; `--delta-us` accepts +/-500
(+/-1.0 power). `--hold` accepts any finite nonnegative duration in seconds,
or `forever` to keep sending the target until Ctrl-C:

```bash
./teensy.sh pulse --all-slots --delta-us 500 --ramp 2 --hold forever --execute
```

Ctrl-C sends neutral immediately and checks the Teensy's acknowledgment;
there is no additional software ramp-down on an operator stop. The motor
controller may apply its own deceleration. Numeric durations, such as
`--hold 30`, finish with the configured ramp down. The command-loss watchdog
remains enabled and output requires fresh commands continuously. Long holds
keep only the most recent pressure sample in memory.

The published M5 example
configuration has a slew rate of 0.00005 power per millisecond, or 5% per
second, so a controller with that setting can take about 20 seconds to reach
full power. Actual installed settings may differ. The old PX4 `throttle=1.0`
was full stick input, which PX4 could reduce through its gain and saturation.

The field pulse can test the transmit path even when return traffic is broken.
Add `--require-feedback` to require a fresh, CRC-valid, fault-free status from
the configured reply node 1 before and during the pulse; this requires a
capture-capable image. The current field image reports protocol version 1
and raw byte counters only.
Node 1 is preserved from the
supplied sketch; its existence on this vehicle has not been verified.

## Observed USB-only check

On 2026-10-28 the USB-only image was flashed to Teensy 4.1 serial `19620450`.
A five-second check received 51 depth frames and matching command ACKs.
The reported bus selection and RS-485 transmit count were both zero.
Serial2 accumulated substantial receive traffic while Serial1 saw only two
bytes. That is consistent with the supplied sketch's PX4-on-Serial2 layout,
but byte counts alone do not identify packets or verify the RS-485 wiring.
A0 read zero, so pressure depth was unavailable. Sensor power, connection and
calibration still need a physical check. No nonneutral motor test was run.
The same board reported a command-timeout fault after a deliberate 0.35-second
pause, then acknowledged neutral cleanup. Both bus-enabled UART variants were
compiled and staged for later use.

## Observed UART1 check

After the operator confirmed hardware UART1, the Serial1 image was installed
on the same Teensy. A five-second neutral check received matching ACKs and
51 sensor frames. At the final sample the board reported 532 transmitted
RS-485 packets and 531 received UART1 bytes since boot. Received bytes are
not validated M5 responses; these counts do not establish motor delivery.
A0 still read zero, so depth remained unavailable.

The UART1 image also reported a timeout after a deliberate 0.35-second command
pause, then acknowledged neutral cleanup. No nonneutral command was sent.
The operator pulse command above is prepared for the physical test.

## Limits and evidence

- Firmware rejects values outside **1500 +/-500 us**, equivalent to +/-1.0
  M5 power. The field tool also honors the limit reported by the installed
  firmware, so older +/-25 images cannot be driven above their own limit.
- CRC32, session IDs and increasing sequence numbers reject malformed,
  corrupted or out-of-order commands in the current session. Neutral must
  precede enabled commands.
- A nominal 250 ms command watchdog sends neutral and requires a fresh
  neutral command before enabling again. USB disconnect also sends neutral.
- Slow USB readers cannot block firmware telemetry writes. Input work is
  bounded; UART transmit completion takes about 4.2 ms per packet.
- An ACK proves Teensy acceptance; the bus packet counter records writes
  only when the bus is enabled. Host support for version 2 receive capture
  distinguishes echoes, noise and valid replies when a capture-capable image
  is used. The current field image supplies counters, which do not prove
  valid responses or motor delivery. One responding motor would not prove
  that all six received a stop.
- Software and an MCU watchdog cannot guarantee stop after loss of MCU
  power, a firmware crash or failed wiring. Physical power removal remains
  the operator's stop mechanism.

Protocol details: newline-terminated ASCII bodies followed by `*CRC32` in
eight hexadecimal digits. `H1,session` requests capabilities; commands are
`C1,session,sequence,enabled,p0,...,p7`. Responses are `I1` (capabilities),
`A1` (acceptance/counters), `D1` (ADC/depth/UART counters), or `F1` (fault).
Version 2 also emits `R1,tx_count,dropped_bytes,hex_data` for receive capture.
Both directions protect the body with standard CRC32. Lines are bounded to
159 bytes before the terminating NUL.

Sources: [PJRC USB versus UART](https://www.pjrc.com/teensy/td_serial.html),
[VideoRay command fields](https://download.videoray.com/documentation/m5_thruster/html/commands_responses.html),
[VideoRay example configuration](https://download.videoray.com/documentation/m5_thruster/html/_print.html).
