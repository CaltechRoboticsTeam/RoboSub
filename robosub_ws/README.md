# RoboSub ROS 2

ROS 2 Humble control with Gazebo Harmonic simulation.
[Deploy, drive and pull logs](../deploy/jetson/README.md).

## Simulation

On the configured Mac:

```bash
setup_ros_shell
ros2 launch robosub_bringup sim_test.launch.py gui:=true plots:=false
```

This runs the Bumblebee Woollett world, dives to 1 m, moves forward 5 m,
then holds position and depth for 10 s. A separate observer scores Gazebo
ground truth and saves a result under `~/.ros/robosub_tests/`.
Scores are advisory: missing a target reports measurements and exits normally.
Deployment does not run Gazebo or use these scores as a gate.

For Xbox driving instead:

```bash
ros2 launch robosub_bringup robot.launch.py \
  world:=bb_woollett gui:=true plots:=false teleop:=browser
```

Open the [driver station](http://127.0.0.1:8765) and click **Enable**.
Sticks/triggers drive directly. Hold a face button for assistance:

| Button | Action |
| --- | --- |
| A | Move forward 1 m, then hold position/depth/heading |
| B | Hold current depth |
| X | Go 0.5 m deeper and hold |
| Y | Turn clockwise 90° and hold heading/depth |

Release ends assistance; sticks take over immediately. Assistance needs
fresh depth/IMU feedback; A also needs DVL. **Disable** stops commands.

## Foxglove and logs

Every master launch starts MCAP recording and a Foxglove bridge.
Connect to `ws://127.0.0.1:8766`; real-robot tunneling is in the deploy guide.
Logs live under `~/.ros/robosub_runs/<run>/bag/*.mcap`.

Use Foxglove's **Parameters** panel on `/depth_heading_controller` or
`/state_estimator`. Accepted changes affect the next callback. Copy desired
constants into `robosub_control/config/constants.py` to keep them after restart.
Useful plots: `/sub/reference/depth` vs `/depth`, `/sub/reference` vs
`/sub/state`, and `/sub/state/rpy`. Camera images: `/sub/camera/image_raw`.

## Control path

`robot.launch.py` selects real or simulated inputs for the same estimator,
PID controllers, trapezoidal motion profile and thruster mixer.

Sim supplies an IMU, pressure-depth stand-in and synthesized DVL velocity.
Real mode's `hardware_sensors` adapts the existing MAVROS IMU, USB/MAVROS
pressure and DVL odometry. Both feed `/imu/data_raw`, `/depth` and
`/dvl/velocity` → estimator → `/sub/state` → controller → `/sub/cmd_vel`
→ mixer → M5 output or simulated forces. Body axes are forward/left/up;
depth is positive down. The estimator uses a Mahony attitude filter and
integrates DVL velocity for XY; yaw is relative to startup.

`current_m5.py` holds mounts and motor IDs; sim physics is in
`robosub_sim/config/current_m5_physics.json`. Drag, added mass and the M5
power curve are approximations: pool measurements are still needed for
physical PID tuning. The rendered camera does not feed visual SLAM.
[Real ZED vision](../docs/software/vision.md) is a separate stack.
