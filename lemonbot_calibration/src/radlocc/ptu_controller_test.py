#!/usr/bin/env python

import rospy
import numpy as np

from ptu_controller import PTUController

if __name__ == '__main__':
    rospy.init_node('ptu_controller_test')

    controller = PTUController('/SetPTUState')

    target_angles = np.linspace(-np.pi/2, np.pi/2, 10)

    controller.goto(target_angles[0], 1)

    for angle in target_angles:
        controller.goto(angle, 0.2)
