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
        self._image_acquire = Event()
        self._laserscan_acquire = Event()

        self._image_subscriber = rospy.Subscriber(
            image_topic, Image, callback=self.image_callback, queue_size=2)
        self._laserscan_subscriber = rospy.Subscriber(
            laserscan_topic, LaserScan, callback=self.laser_callback, queue_size=10)

        self._ptu_controller = ptu_controller

        self._data_saver = data_saver

    def run(self):
        self._ptu_controller.goto(self._pitch_range[0])

        for pitch_angle in self._pitch_range:
            self._image_acquire.set()
            self._image_acquire.wait()

            self._laserscan_acquire.set()

            self._ptu_controller.goto(pitch_angle)

            self._laserscan_acquire.clear()

    def laser_callback(self, laser_msg):
        if self._laserscan_acquire.is_set:
            print("received laserscan")
            self._data_saver.save_laser(laser_msg)

    def image_callback(self, image_msg):
        if self._image_acquire.is_set:
            self._image_acquire.clear()
            print("received image")
            self._data_saver.save_image(image_msg)
