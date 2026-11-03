# Jetson development

On the Mac, leave this tunnel running:

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -L 18765:127.0.0.1:8765 \
  -L 18766:127.0.0.1:8766 \
  crt@crt-jetson.taile4657b.ts.net
```

Deploy from another Mac terminal:

```bash
cd ~/Documents/GitHub/RoboSub
./deploy/jetson/deploy.sh crt@crt-jetson.taile4657b.ts.net
```

Deployment copies your working tree, builds it, runs `tests/control/`, and
selects the new release. Failed tests prompt before deployment; use
`--deploy-anyway` to accept failures without a prompt. Build failures stop it.
Test output is saved in the release's `control-tests.log`.

Connect to the Jetson and start driving:

```bash
ssh crt@crt-jetson.taile4657b.ts.net
cd ~/robosub-field/current
source ./env.sh
ros2 launch robosub_bringup robot.launch.py \
  mode:=real \
  vehicle:=current_m5 \
  vision:=false \
  gui:=false \
  plots:=false \
  teleop:=browser \
  actuators:=teensy_usb \
  teleop_scale:=1.0
```

Open the [driver station](http://127.0.0.1:18765). In Foxglove, choose
**Open connection → Foxglove WebSocket** and enter `ws://127.0.0.1:18766`.
MAVROS and the existing DVL reader must be running; this launch consumes
those processes' sensor topics. `actuators:=preview` leaves USB closed.

For the Bluetooth Xbox controller in Zen/Firefox on Mac, leave
`ros2 run robosub_bringup mac_gamepad` running in a Mac ROS shell too.
Mac-local simulation starts this reader automatically with the driver station.

To stop, click **Disable**, press **Ctrl-C** in the mission terminal, and
wait for recording to close. Before redeploying, exit that Jetson shell;
reconnect and source the new release afterward.

To pull logs, run on the Mac:

```bash
mkdir -p ~/Developer/robosub-logs/jetson
rsync -av \
  crt@crt-jetson.taile4657b.ts.net:.ros/robosub_runs/ \
  ~/Developer/robosub-logs/jetson/
```

Every run records automatically. Open the run's `bag/*.mcap` in Foxglove.
[USB diagnostics](../../docs/software/teensy-usb.md) are separate from driving.
