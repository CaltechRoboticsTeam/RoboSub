# T200 thrusters, geometry and PWM

Saved settings live in `robosub_control/robosub_control/config/future_t200.py`.
Select them with `vehicle:=future_t200`. The mounts below are a planning copy
of the current M5 vehicle, **not measured geometry for the future vehicle**.
`GEOMETRY_CONFIRMED` remains false. Positions were supplied from Onshape: +x aft, +y starboard, +z up. The conversion to ROS body axes is
`(x, y, z) -> (-x, -y, z)`. Positions are relative to the supplied robot origin,
not a measured center of mass.

## Configuration

| Motor | ROS position [m] | Clockwise yaw | Pitch down | Channel | Topic ID |
| --- | --- | --- | --- | --- | --- |
| FR | 0.352, -0.215, 0.033 | 315 deg | 0 deg | 0 | t0 |
| FL | 0.352, 0.229, 0.033 | 45 deg | 0 deg | 1 | t1 |
| BR | -0.352, -0.215, 0.033 | 225 deg | 0 deg | 2 | t2 |
| BL | -0.352, 0.229, 0.033 | 135 deg | 0 deg | 3 | t3 |
| RD | 0.012, -0.287, -0.014 | 0 deg | 90 deg | 4 | t4 |
| LD | 0.012, 0.301, -0.014 | 0 deg | 90 deg | 5 | t5 |

`THRUSTER_CHANNELS` follows the shared zero-based software order. Future
physical board wiring remains unverified. `THRUSTER_REVERSED[name]` selects whether positive force along the
configured mount axis requires the ESC's reverse command. All reversal flags
start at `False` pending a direction check. Negative force selects the reverse
performance curve. Electrical inversion mirrors the resulting pulse around
1500 us; it does not change the physical forward/reverse thrust ratings.

`THRUSTER_SUPPLY_VOLTAGE_V = 16.0` is a reference constant because the actual
LiPo voltage/cell count was not provided. Set it to the voltage at the ESCs.
The voltage, channel and direction confirmation flags remain false. These
settings and the battery/PDP current budget need physical verification before
connecting an output driver. The software does not change the supplied power.

Copied ROS translations for planning:

| Component | Position relative to body origin [m] |
| --- | --- |
| DVL | -0.153, 0.007, -0.016 |
| Custom boards at former Pixhawk location | 0.0, 0.006, 0.049 |
| Camera | -0.157118304, 0.006, -0.0526 |

The camera offset from the former Pixhawk location was supplied in Onshape
axes as `(6.185760 in, 0, -4 in)`. Its orientation is an explicit zero
placeholder, with `CAMERA_ORIENTATION_CONFIRMED = False`. It is not a measured
camera/IMU extrinsic transform. Physical camera TF still uses the vision
configuration; zero placeholder orientation does not enable a body-frame pose.
The DVL faces downward. Its real adapter must rotate velocity into body axes
before publishing, as required by the estimator's existing input contract.

## Command path

```text
controller normalized efforts
  -> geometry allocator: body force [N] and torque [N.m]
  -> individual force targets [N]
  -> T200 voltage/current/thrust lookup
  -> channel-ordered pulse widths [us]
       -> simulation: modeled applied forces -> Gazebo
       -> real mode: numeric PWM preview; board transport is not implemented
```

`thrust_allocation.py` builds each motor's wrench column as `[direction,
position x direction]` and solves surge/sway/heave/roll/yaw. Pitch is not
independently controllable with this layout. Off-center mounts create coupled
moments that the allocator accounts for on the controlled axes.

Unit effort requests the symmetric, available single-axis capacity for the
configured voltage. `MIXER_AXIS_FRACTIONS` can
reduce that capacity. Combined commands scale together if any motor would
exceed its available forward/reverse thrust, preserving the requested
controlled-wrench direction. These scales replace the fixed 30/60 N gains.

At the 16 V reference with all reversal flags false, approximate per-unit
scales are: surge 111.52 N, sway 112.88 N, heave 77.96 N, roll 23.47 N.m and
yaw 64.79 N.m. PID gains and roll feedforward still require tuning for the
actual mass, buoyancy, drag and electrical setup.

## ESC and thrust model

The [Basic ESC R3 specifications](https://bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/)
define 1500 us neutral, 1100/1900 us maximum reverse/forward, a neutral band
of +/-25 us and a maximum pulse update rate of 400 Hz. Pulse width is not a
percentage duty-cycle command. The controller may calculate at 1 kHz while
command publication is capped separately; rates over 400 Hz are rejected.
The preview/simulation holds neutral for `ESC_NEUTRAL_INIT_S = 3.0` seconds,
our implementation of the vendor's requirement for a few seconds at neutral.

`t200.py` uses the manufacturer's static measurements for 10-20 V with a
Basic ESC R3 and stock BLHeli_S 16.6. It interpolates between measured
voltages, handles directional asymmetry/deadband and limits modeled current
to the configured value, at most 30 A per ESC. It never extrapolates voltage.
At 16 V the source-data endpoints are about +51.44/-39.91 N per motor.

Data provenance and preprocessing are recorded alongside
`robosub_control/robosub_control/thrusters/data/t200.json`. This is an open-loop estimate:
installed water flow, voltage sag, wiring and temperature affect actual
thrust/current. A modeled current bound is not electrical or thermal
protection, and is not a configured whole-battery current limit.

## ROS topics and custom-board contract

- `/sub/cmd_vel`: `Twist` containing normalized efforts, not speeds.
- `/sub/thrust/t0..t5`: individual target forces in N, using the shared zero-based IDs.
- `/sub/thruster_forces`: coherent `Float64MultiArray` in
  `FR,FL,BR,BL,RD,LD` order; the layout label must match.
- `/sub/esc/pwm_preview`: `UInt16MultiArray`, pulse width in us, ordered by
  board channel 0 through 5. This topic does not generate physical pulses.
- `/sim/thrust/t0..t5`: modeled applied force in N after PWM resolution and
  deadband. Only simulation publishes these; the Gazebo bridge consumes them.

The mixer and ESC stage independently go neutral when their input stops.
The ESC stage also enforces its publication rate and startup neutral hold.
The eventual custom-board driver/firmware must implement the actual transport,
channel-to-pin mapping, timed pulses, startup neutral and its own command-loss
watchdog. No serial format, bus address or pin assignment has been invented.

## Verification

```bash
python3 -m unittest discover -s tests/control -v
```

Numeric checks cover force units, voltage interpolation, directional curves,
current/range bounds, coordinate conversion, allocation saturation and generated
world geometry. ROS checks exercise force-to-PWM flow and neutral handling in
an isolated domain, without a physical output driver.

Sources: [T200 product specifications](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/),
[manufacturer performance workbook](https://cad.bluerobotics.com/T200-Public-Performance-Data-10-20V-September-2019.xlsx),
[Gazebo force-command semantics](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1Thruster.html).
