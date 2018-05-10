#!/usr/bin/env python

"""

"""

import rospy

import tf2_ros
import tf

import std_msgs.msg
import geometry_msgs.msg
from fiducial_msgs.msg import FiducialTransformArray

class FiducialBroadcaster:
    def __init__(self, object_frame_id, fiducial_id):
        self.object_frame_id = object_frame_id
        self.fiducial_id = fiducial_id

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()

        self._fiducial_tf_sub = rospy.Subscriber('fiducial_transforms',
                                                 FiducialTransformArray,
                                                 self.update_fiducial,
                                                 queue_size=2)
    
    def update_fiducial(self, msg):
        header = msg.header
        # find fiducial with the correct id
        fid = next( (f for f in msg.transforms if f.fiducial_id == self.fiducial_id), None)

        # if none was found, publish nothing (return)
        if fid is None: return
        
        msg = geometry_msgs.msg.TransformStamped()
        msg.child_frame_id = self.object_frame_id
        msg.header = header
        msg.transform.translation.x = fid.transform.translation.x
        msg.transform.translation.y = fid.transform.translation.y
        msg.transform.translation.z = fid.transform.translation.z

        msg.transform.rotation.x = fid.transform.rotation.x
        msg.transform.rotation.y = fid.transform.rotation.y
        msg.transform.rotation.z = fid.transform.rotation.z
        msg.transform.rotation.w = fid.transform.rotation.w

        self._tf_broadcaster.sendTransform(msg)


if __name__ == '__main__':
    print("hello")

    rospy.init_node('fiducial_broadcaster_node')

    object_frame_id = rospy.get_param('~object_frame_id')
    fiducial_id = rospy.get_param('~fiducial_id')

    broadcaster = FiducialBroadcaster(object_frame_id, fiducial_id)

    rospy.spin()
