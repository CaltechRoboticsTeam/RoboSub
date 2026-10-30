# Jetson field deployment

From the Mac repository root:

```bash
./deploy/jetson/deploy.sh crt@100.95.114.100
```

This copies runtime sources, guides, tools and control tests into a new
`~/robosub-field/releases/` directory, verifies SHA-256 transfer checksums,
builds `robosub_control`, `robosub_sim` and `robosub_bringup`, and runs tests in isolated ROS
domain 198. It refuses that test domain if a running MAVROS process uses it.
The `current` symlink changes only after successful build/tests. Existing
checkouts and running robot programs are left alone; no startup service,
controller, camera or MAVROS instance is started by deployment.

The snapshot includes uncommitted files. `SNAPSHOT.json` records their hashes
and the base Git revision. `reference/legacy_jetson/` preserves the actual old
Jetson source, which includes files missing from the Mac prototype. The old
startup note and receiver sketch are copied as reference, not executed.

On the Jetson:

```bash
cd ~/robosub-field/current
./field.sh status --seconds 10
./field.sh pulse --seconds 3
```

Both commands above are passive. The second validates a pulse plan but does
not create a manual-control publisher or send an arming command.

See [the field-testing guide](../../docs/software/field-testing.md) for the
telemetry-rate setup, explicit execution command, output limits and the
Teensy troubleshooting path.
Every wrapper invocation writes a report under `~/robosub-field/logs/`.

For the HTML tuning/joystick page and robot-relative driving, on the Jetson:

```bash
cd ~/robosub-field/current
./joystick.sh --execute teleop_scale:=1.0
```

In another Mac terminal:

```bash
ssh -N -L 8765:127.0.0.1:8765 crt@100.95.114.100
```

Open [http://127.0.0.1:8765](http://127.0.0.1:8765), enable controller input,
use the sticks/triggers, and click Disable to stop. RB is unused;
Ctrl-C stops the launch. Use this with the robot in water and people clear.

Omit `--execute` for previews without opening USB. The live path bypasses PX4
and requires the existing Teensy Serial1 image. Do not run a USB pulse alongside it. All six motor slots use the supplied order:
0=FR, 1=FL, 2=BR, 3=BL, 4=RD, 6=LD; protocol slots 5 and 7 stay neutral.
