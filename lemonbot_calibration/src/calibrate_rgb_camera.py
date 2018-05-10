#!/usr/bin/env python

import argparse

import numpy as np

import rospy

import tf2_ros
import tf

from tf.transformations import euler_from_quaternion

import geometry_msgs.msg
from fiducial_msgs.msg import FiducialTransformArray

from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera, reset

class CalibrationHandler:
    
    def __init__(self, world_frame, hand_frame, camera_frame, object_frame):
        self.world_frame = world_frame
        self.hand_frame = hand_frame
        self.camera_frame = camera_frame
        self.object_frame = object_frame

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._world_effector_pub = rospy.Publisher(
            'world_effector', geometry_msgs.msg.Transform, queue_size=2)
        self._camera_object_pub = rospy.Publisher(
            'camera_object', geometry_msgs.msg.Transform, queue_size=2)
        
        self._calibrator_service = rospy.ServiceProxy(
            'compute_effector_camera', compute_effector_camera)
        
    def save_pose(self):
        world2hand = self._tf_buffer.lookup_transform(
            self.world_frame,
            self.hand_frame,
            rospy.Time())
        camera2object = self._tf_buffer.lookup_transform(
            self.camera_frame,
            self.object_frame,
            rospy.Time())
        
        self._world_effector_pub.publish(world2hand.transform)
        self._camera_object_pub.publish(camera2object.transform)
    
    def compute_calibration(self):
        return self._calibrator_service()

if __name__ == '__main__':

    rospy.init_node('hand2eye_rgb_camera_node')

    handler = CalibrationHandler(
        world_frame='lemonbot_base_link',
        hand_frame='lemonbot_ptu_mount_link',
        camera_frame='lemonbot_camera_mount_link',
        object_frame='lemonbot_calibration_object',
    )

    print('ready...')
    print('press <enter> to register the poses')
    print('type <y> to finish the capture')

    while raw_input('') != 'y':
        handler.save_pose()
        print('saved state')

    results = handler.compute_calibration()

    print(results)

    quaternion = (
        results.effector_camera.rotation.x,
        results.effector_camera.rotation.y,
        results.effector_camera.rotation.z,
        results.effector_camera.rotation.w)

    print(euler_from_quaternion(quaternion))