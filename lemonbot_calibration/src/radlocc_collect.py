#!/usr/bin/env python

import numpy as np
import pandas as pd
from threading import Event

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan

class PTUController:
    
    def __init__(self):
        pass
    
    def goto(self, pitch):
        print("going to ", pitch)

class RadloccCollectNode:

    def __init__(self, pitch_range, ptu_controller, image_topic, laserscan_topic):

        self._pitch_range = pitch_range

        self._images = []
        self._laserscans = []
        self._image_acquire = Event()
        self._laserscan_acquire = Event()

        self._ptu_controller = ptu_controller
        self._bridge = CvBridge()

    def loop(self):
        self._ptu_controller.goto(self._pitch_range[0])
        
        for pitch_angle in self._pitch_range:
            self._laserscan_acquire.clear()

            self._image_acquire.set()
            self._image_acquire.wait()

            self._ptu_controller.goto(pitch_angle)

            self._laserscan_acquire.set()
    
    def laser_callback(self, laser_msg):
        print("received laserscan")

    def image_callback(self, image_msg):
        
        # image = self._bridge.imgmsg_to_cv2(image_msg, "brg8")
        print("received image")

if __name__ == '__main__':
    node = rospy.init_node("radlocc_collect")

    radlocc_collect_node = RadloccCollectNode(np.linspace(-np.pi / 6, np.pi / 6, 20))

    radlocc_collect_node.loop()

