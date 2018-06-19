from threading import Event

import rospy

from sensor_msgs.msg import Image, LaserScan

from ptu_controller import PTUController
from data_saver import DataSaver


class CollectNode:

    def __init__(self, pitch_range, image_topic, laserscan_topic, ptu_controller, data_saver):

        self._pitch_range = pitch_range

        self._images = []
        self._laserscans = []
        self._image_acquire = False
        self._laserscan_acquire = False

        self._image_subscriber = rospy.Subscriber(
            image_topic, Image, callback=self.image_callback, queue_size=10)
        self._laserscan_subscriber = rospy.Subscriber(
            laserscan_topic, LaserScan, callback=self.laser_callback, queue_size=10)

        self._ptu_controller = ptu_controller

        self._data_saver = data_saver

    def run(self):
        self._ptu_controller.goto(self._pitch_range[0], 1)

        for pitch_angle in self._pitch_range:
            self._image_acquire = True
            while not self._image_acquire:
                pass

            self._laserscan_acquire = True

            self._ptu_controller.goto(pitch_angle)

            self._laserscan_acquire = False

        self._data_saver.save()

    def laser_callback(self, laser_msg):
        if self._laserscan_acquire:
            print("received laserscan")
            self._data_saver.save_laser(laser_msg)

    def image_callback(self, image_msg):
        if self._image_acquire:
            self._image_acquire = False
            print("received image")
            self._data_saver.save_image(image_msg)
