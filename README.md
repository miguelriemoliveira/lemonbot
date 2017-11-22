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
