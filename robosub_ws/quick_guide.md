Pixhawk motor-output numbers in MAVLink SERVO_OUTPUT_RAW
  -> receiver microcontroller over a serial telemetry link
  -> normalize each of eight numbers: (value - 1500) / 500
  -> construct VideoRay propulsion packet and CRC checksums
  -> RS-485 transceiver and bus
  -> each M5's integrated motor controller


 1500 us neutral, 1100/1900 us maximum reverse/forward, a neutral band
of +/-25 usf 400 Hz… seems ROS has a pub limit of 100?

HOWEVERcontroller normalized efforts
  -> geometry allocator: body force [N] and torque [N.m]
  -> individual forces like swereve [N] 
  -> T200 voltage/current/thrust lookup for FF (THIS IS ENTIRELY ABSENT IN M5!)
all of above in thrust_allocation.py…. is VERY important logic, kinda ugly rn.
  -> channel-ordered pulse widths [us]
       -> simulation: modeled applied forces -> Gazebo
       -> real mode: numeric PWM preview; board transport is not implemented

Motor Positions Org Center
FL -> -.352m,-.229m,.033m, 45 deg clockwise,
FR -> -.352m,.215m,.033m, 315 deg clockwise,
BR -> .352m,.215m,.033m, 215 deg clockwise
BL -> .352m,-.229m,.033m, 135 deg clockwise
DL -> -.012m,-.301m,-.014m, 90 deg down
DR -> -.012m,.287m,-.014m, 90 deg down


/sub/cmd_vel -> Normalized chassis speeds in `Twist`, not measured speeds  tho, entirely “setpoint”
| `/sub/m5/power_targets` |`FL,FR,BR,BL,DL,DR` order 
| `/sub/m5/power/t1..t6` individual power target
| `/sub/m5/power_preview` Six polarity-adjusted power commands in named motor order for PWM outputs sake
| `/sim/thrust/t1..t6` | Simulation-only applied forces in N\




LoggableNumbers (yes, I reinvented this from FRC)
ssh -N -L 8765:127.0.0.1:8765 crt@100.95.114.100self.tuning = TunableNumbers(self)

self.some_gain = self.tuning.number(
    “gain”,
    1.0,
    minimum=0.0,
)

value = self.gain.get()


Or, if you just wanna not deal with .get and user publishers (I REC THIS WAY, LESS ISSUES, and USE ROS STUFF NICELY!)

self.gain.bind_to(self.pid, "kp")


Then, just run!:ros2 launch robosub_bringup robot.launch.py mode:=sim



FOR NORMAL DEV:
Basically always run:ssh -N -o ExitOnForwardFailure=yes \
  -L 18765:127.0.0.1:8765 \
  -L 18766:127.0.0.1:8766 \
  crt@crt-jetson.taile4657b.ts.netthen in another term (for gamepad if wanted)setup_ros_shell
ros2 run robosub_bringup mac_gamepad
That forwards the DS web panel (For Zen) and Foxglove to this device, at 18765 and 18766 respectively.

cd ~/Documents/GitHub/RoboSub

./deploy/jetson/deploy.sh crt@crt-jetson.taile4657b.ts.net
Then, after deploying, get back into the jetson, and run your mission.
ssh crt@crt-jetson.taile4657b.ts.netcd ~/robosub-field/current
source ./env.shthen finally run whatever YOUR mission code is (for simple driving, do:)ros2 launch robosub_bringup robot.launch.py \
  mode:=real \
  vehicle:=current_m5 \
  vision:=false \
  gui:=false \
  plots:=false \
  teleop:=browser \
  actuators:=teensy_usb \
  teleop_scale:=1.0
After a run, you DISABLE then ctrl c the jetson ROS mission, and if u want the log file, simply “pull” the folder
To GET logs:ON THE MAC
mkdir -p ~/Developer/robosub-logs/jetson

rsync -av \
  crt@crt-jetson.taile4657b.ts.net:.ros/robosub_runs/ \
  ~/Developer/robosub-logs/jetson/