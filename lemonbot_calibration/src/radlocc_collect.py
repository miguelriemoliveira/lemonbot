#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

import rospy

from radlocc import CollectNode, PTUController, DataSaver

if __name__ == '__main__':
    rospy.init_node("radlocc_collect")

    ds = DataSaver('./radlocc_data')

    radlocc_collect_node = CollectNode(
        ptu_controller=None,
        data_saver=ds,
        image_topic="/camera/image_color",
        laserscan_topic="/laserscan")

    while raw_input("another one? ") != 'q':
        radlocc_collect_node.run()

    ds.save()
