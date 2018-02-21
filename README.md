# lemonbot
Drivers for the Lemon Bot hardware platform.

# Bringup the system

roslaunch lemonbot_bringup all.launch

# Control the PTU using the terminal

Bringup the system, see [this section](#bringup-the-system)

and now request a new position

rostopic pub /ptu/cmd sensor_msgs/JointState "header:
seq: 0
stamp: {secs: 0, nsecs: 0}
frame_id: ''
name: ['ptu_pan','ptu_tilt']
position: [0.5,0.1]
velocity: [1,1]
effort: [0]"

# System calibration

## Calibration files

There are two calibration files, for the extrinsic calibration of both the camera and the laser. These files are located under `lemonbot_calibration/calibration`. They look like:

```yaml
xyz: [0, 0, 0]
rpy: [0, 0, 0]
```

These files are automatically loaded to the `robot_description` using the xacro preprocessor.
