#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

import rospy

from radlocc import CollectNode, PTUController, DataSaver

if __name__ == '__main__':
    rospy.init_node("radlocc_collect")

    ds = DataSaver('./radlocc_data')

    radlocc_collect_node = CollectNode(
        ptu_controller=PTUController("/SetPTUState"),
        data_saver=ds,
        image_topic="/camera/image_color",
        laserscan_topic="/laserscan")

    angles = np.linspace(-0.05, 0.05, 2)

    while raw_input("another one? ") != 'q':
        radlocc_collect_node.run(angles)

    ds.save()
