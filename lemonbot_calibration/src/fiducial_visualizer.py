#!/usr/bin/env python

import rospy

from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Vector3
from fiducial_msgs.msg import FiducialTransformArray
from visualization_msgs.msg import MarkerArray, Marker

class FiducialVisualizer:
    def __init__(self, fiducial_topic):

        self._fiducial_tf_sub = rospy.Subscriber(fiducial_topic,
                                                 FiducialTransformArray,
                                                 self.update_fiducial,
                                                 queue_size=2)
        
        self._vis_pub = rospy.Publisher('visualization_msgs', MarkerArray, queue_size=2)
    
    def update_fiducial(self, msg):
        header = msg.header
        
        markers = [
            Marker(
                header=header,
                ns='fiducials',
                id=tf.fiducial_id,
                type=Marker.CYLINDER,
                action=Marker.ADD,
                pose=Pose(
                    position=tf.transform.translation,
                    orientation=tf.transform.rotation,
                ),
                scale=Vector3(0.1, 0.1, 0.1),
                color=ColorRGBA(1, 0, 0, 1),
                duration=10,
            )
            for tf in msg.transforms
        ]

        self._vis_pub.publish(MarkerArray(makers=markers))


if __name__ == '__main__':

    rospy.init_node('fiducial_broadcaster_node')

    broadcaster = FiducialVisualizer("fiducial_transforms")

    rospy.spin()
