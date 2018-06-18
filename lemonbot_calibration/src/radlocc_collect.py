#!/usr/bin/env python

import numpy as np
import pandas as pd
from threading import Event, Thread

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan

from time import sleep

from actionlib import SimpleActionClient
from flir_pantilt_d46.msg import PtuGotoAction, PtuGotoGoal


class PTUController:

    def __init__(self, ptu_topic):
        self._action_client = SimpleActionClient(ptu_topic, PtuGotoAction)

    def goto(self, pan):
        print("going to ", pan)
        goal = PtuGotoGoal(
            pan=pan,
            pan_vel=1,
            tilt=0,
            tilt_vel=1)

        self._action_client.send_goal_and_wait(goal)


class RadloccCollectNode:

    def __init__(self, pitch_range, ptu_controller, image_topic, laserscan_topic):

        self._pitch_range = pitch_range

        self._images = []
        self._laserscans = []
        self._image_acquire = Event()
        self._laserscan_acquire = Event()

        self._image_subscriber = rospy.Subscriber(
            image_topic, Image, callback=self.image_callback, queue_size=2)
        self._laserscan_subscriber = rospy.Subscriber(
            laserscan_topic, LaserScan, callback=self.laser_callback, queue_size=10)

        self._ptu_controller = ptu_controller
        self._bridge = CvBridge()

    def run(self):
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

    radlocc_collect_node = RadloccCollectNode(
        pitch_range=np.linspace(-np.pi / 6, np.pi / 6, 20),
        ptu_controller=PTUController("/SetPtuState"),
        image_topic="/camera/image_raw/compressed",
        laserscan_topic="/laserscan")

    radlocc_collect_node.run()
