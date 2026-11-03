# Teensy M5 USB bridge

Current path: Jetson USB → Teensy 4.1 Serial1 → RS-485 → six VideoRay M5s.
Motor IDs: **FR=0, FL=1, BR=2, BL=3, RD=4, LD=6**; slots 5 and 7 stay neutral.
USB driving bypasses PX4 arming. See the [driving guide](../../deploy/jetson/README.md).

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

## Diagnostics

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
fault-free motor replies and capture-capable firmware.

## Protocol

CRC32-framed ASCII: `H1` capabilities, `C1` commands, `I1` capabilities reply,
`A1` acknowledgment, `D1` pressure/UART telemetry, `F1` fault; v2 adds `R1`
receive capture. Commands carry session IDs and increasing sequence numbers.
A 250 ms watchdog or USB disconnect sends neutral; re-enabling requires a
fresh neutral command. Pressure calibration is preserved from the supplied
sketch; ADC values ≤1 are unavailable.

The supplied wiring/settings use DE=3, RE=2, A0 pressure, 115200 baud,
group address `0x81`, reply node 1. Source: preserved
`firmware/reference/teensy-installed-2026-10-28.ino`.
[VideoRay command reference](https://download.videoray.com/documentation/m5_thruster/html/commands_responses.html).
