#!/usr/bin/env python

from __future__ import print_function, with_statement

import numpy as np
from threading import Event, Thread

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan

from cv2 import imwrite

from time import sleep

from actionlib import SimpleActionClient
from flir_pantilt_d46.msg import PtuGotoAction, PtuGotoGoal

import os
from os import path


class RadloccDataSaver:
    """
    The data saver for Radlocc.

    Stores the both the images and the laserscans into the text format required for the radlocc to read, 
    which is the laserscans text file, which is of the following format:
    `<timestamp> StartAngleRads AngleIncrementRads EndAngleRads RangeUnitType NoAngles [Ranges]`
    and the images text file, which only has the timestamps, corresponding to an image stored in disk,
    one for each row in the text file.
    """

    def __init__(self, path):
        """
        Parameters
        ----------
        path: str
            The base path for storing the files. Either it exists and it's an empty directory or it
            is mkdir'ed during the initialization.
        """
        if(path.exists(path)):
            assert(path.isdir(path), "path, if exists, should be a directory")
            assert(os.listdir(path) == [], "path, if exists, should be empty")
        else:
            os.mkdir(path)

        self._base_path = path

        self._laserscans = []
        self._images = []

        self._image_id = 0

    def save_image(self, image):
        id = self._image_id
        self._image_id += 1
        image_path = path.join(self._base_path, 'image_{:03d}.png'.format(id))

        imwrite(image_path, image)

        self._images.append({
            'secs': image.header.stamp.secs,
            'nsecs': image.header.stamp.nsecs,
        })

    def save_laser(self, laserscan):
        stamp = laserscan.header.stamp
        self._laserscans.append({
            'timestamp': '{secs}.{nsecs}'.format(secs=stamp.secs, nsecs=stamp.nsecs),
            'angle_min': laserscan.angle_min,
            'angle_increment': laserscan.angle_increment,
            'angle_max': laserscan.angle_max,
            'range_unit_type': 3,
            'ranges': laserscan.ranges,
        })

    def save(self):
        with open('laser.txt', 'w') as f:
            for scan in self._laserscans:
                ranges = scan['ranges']
                header = [
                    scan['timestamp'],
                    scan['angle_min'],
                    scan['angle_increment'],
                    scan['angle_max'],
                    scan['range_unit_type'],
                    len(ranges)
                ]
                row = np.append(header, ranges)

                f.write(' '.join(row) + '\n')

        with open('image_stamps.txt', 'w') as f:
            for scan in self._images:
                row = [
                    scan['secs'],
                    scan['nsecs'],
                ]

                f.write(' '.join(row) + '\n')


class PTUController:
    """
    PTUController is a very simple action server client for the PTU.
    """

    def __init__(self, ptu_topic):
        """
        Initializes the controller

        Parameters
        ----------
        ptu_topic: str
            The topic of the ptu action server.
        """

        self._action_client = SimpleActionClient(ptu_topic, PtuGotoAction)

        self._action_client.wait_for_server()

    def goto(self, pan, vel=0.1):
        """
        Moves the PTU to a defined value of pan.

        Parameters
        ----------
        pan: float
            The target value
        vel: float
            The angular velocity of the pan. Defaults to 0.1 rads per secound.
        """

        print("going to ", pan)
        goal = PtuGotoGoal(
            pan=(pan * np.pi / 180),
            pan_vel=1,
            tilt=0,
            tilt_vel=1)

        self._action_client.send_goal_and_wait(goal)


class RadloccCollectNode:

    def __init__(self, pitch_range, image_topic, laserscan_topic, ptu_controller=None, data_saver=None):

        self._pitch_range = pitch_range

        self._images = []
        self._laserscans = []
        self._image_acquire = Event()
        self._laserscan_acquire = Event()

        self._image_subscriber = rospy.Subscriber(
            image_topic, Image, callback=self.image_callback, queue_size=2)
        self._laserscan_subscriber = rospy.Subscriber(
            laserscan_topic, LaserScan, callback=self.laser_callback, queue_size=10)

        self._ptu_controller = ptu_controller if ptu_controller else PTUController(
            "/SetPtuState")
        self._bridge = CvBridge()

        self._data_saver = data_saver if data_saver else RadloccDataSaver(
            './data')

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
            image = self._bridge.imgmsg_to_cv2(image_msg, "brg8")
            self._data_saver.save_image(image)
        


if __name__ == '__main__':
    node = rospy.init_node("radlocc_collect")

    radlocc_collect_node = RadloccCollectNode(
        pitch_range=np.linspace(-np.pi / 6, np.pi / 6, 20),
        ptu_controller=PTUController("/SetPtuState"),
        data_saver=RadloccData
        image_topic="/camera/image_raw/compressed",
        laserscan_topic="/laserscan")

    radlocc_collect_node.run()
