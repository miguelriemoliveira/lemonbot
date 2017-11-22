# lemonbot
Drivers for the Lemon Bot hardware platform.

# 

# Control the PTU using the terminal

Bringup the system

    roslaunch lemonbot_bringup all.launch

and now request a new position

    rostopic pub /ptu/cmd sensor_msgs/JointState "header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
    name: ['ptu_pan','ptu_tilt']
    position: [0.5,0.1]
    velocity: [1,1]
    effort: [0]"
