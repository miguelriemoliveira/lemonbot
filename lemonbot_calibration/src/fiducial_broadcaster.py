#!/usr/bin/env python

"""

"""

import rospy

import tf2_ros
import tf

import geometry_msgs.msg
from fiducial_msgs.msg import FiducialTransformArray

class FiducialBroadcaster:
    def __init__(self, base_frame_id, object_frame_id, fid_id):
        self.base_link = base_link
        self.base_frame_id = base_frame_id
        self.object_frame_id = object_frame_id

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener()
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()

        self._fiducial_tf_sub = rospy.Subscriber('fiducial_transforms',
                                                 FiducialTransformArray,
                                                 self.update_fiducial,
                                                 queue_size=2)
    
    def update_fiducial(msg):
        # find fiducial with the correct id
        fid = next( (f for f in msg.transforms if f.fiducial_id == self.fid_id), None)
        
        # if none was found, publish nothing (return)
        if fid is None: return
        
        msg = geometry_msgs.msg.TransformStamped()
        msg.header.frame_id = self.base_frame_id
        msg.child_frame_id = self.object_frame_id
        msg.transform.translation.x = fid.translation.x
        msg.transform.translation.y = fid.translation.y
        msg.transform.translation.z = fid.translation.z

        fid.transform.rotation.x = fid.rotation.x
        fid.transform.rotation.y = fid.rotation.y
        fid.transform.rotation.z = fid.rotation.z
        fid.transform.rotation.w = fid.rotation.w

        self._tf_broadcaster.sendTransform(msg)



if __name__ == '__main__':
    print("hello")

    rospy.init_node('fiducial_broadcaster_node')

    base_frame_id = rospy.get_param('~base_frame_id')
    object_frame_id = rospy.get_param('~object_frame_id')
    fid_id = rospy.get_param('~fid_id')

    broadcaster = FiducialBroadcaster(base_frame_id, object_frame_id, fid_id)

    while ros.ok()
        pass
