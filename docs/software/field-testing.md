# Field testing

Start the robot using the [Jetson guide](https://github.com/CaltechRoboticsTeam/RoboSub/blob/main/deploy/jetson/README.md).
The driver station shows device status under **Diagnostics**. To read it
from a Jetson terminal:

```bash
ros2 topic echo /diagnostics
```

Use the [Teensy diagnostics](teensy-usb.md) for USB and RS-485 checks.
