# Field testing

For normal driving, deployment, Foxglove and logs, use the
[Jetson guide](../../deploy/jetson/README.md).

The current M5 path is Jetson USB → Teensy Serial1 → RS-485 → motors.
[USB diagnostics and firmware](teensy-usb.md) cover that path.

For passive MAVROS/Pixhawk diagnostics on the Jetson:

```bash
cd ~/robosub-field/current
./field.sh status --seconds 10
```

Reports go to `~/robosub-field/logs/` and include sensor rates, servo outputs
and competing command publishers. They do not establish physical movement.

`field.sh pulse` exercises the legacy PX4 command path. Without `--execute`
it only checks telemetry and prints a plan:

```bash
./field.sh pulse --seconds 3 --axis throttle --effort 0.05
```

Execution arms PX4, sends the bounded pulse, then sends neutral and disarms.
It requires MANUAL mode, airframe 60002, fresh neutral servo outputs at
5 Hz or faster, and no competing command publisher. To request 10 Hz:

```bash
source ./env.sh
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval \
  "{message_id: 36, message_rate: 10.0}"
```

This changes the existing MAVLink stream rate. Use direct USB driving for
the current motor path; do not run either pulse tool alongside a mission.
