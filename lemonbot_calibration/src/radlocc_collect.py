#!/usr/bin/env python

import numpy as np

import rospy

from radlocc import CollectNode, PTUController, DataSaver

if __name__ == '__main__':
    rospy.init_node("radlocc_collect")

    angle_min = rospy.get_param("~angle_min")
    angle_max = rospy.get_param("~angle_max")
    steps = rospy.get_param('~steps')
    output = rospy.get_param('~output', '.')

    radlocc_collect_node = CollectNode(
        pitch_range=np.linspace(angle_min, angle_max, steps),
        ptu_controller=PTUController("/SetPTUState"),
        data_saver=DataSaver('./radlocc_data'),
        image_topic="/camera/image_color",
        laserscan_topic="/laserscan")

    radlocc_collect_node.run()
