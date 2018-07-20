#!/usr/bin/env python

from math import sqrt

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

        self._vis_pub = rospy.Publisher(
            'visualization_msgs', MarkerArray, queue_size=2)

    def update_fiducial(self, msg):
        header = msg.header

        markers = [
            Marker(
                header=header,
                ns='fiducials',
                id=tf.fiducial_id,
                type=Marker.CUBE,
                action=Marker.ADD,
                pose=Pose(
                    position=tf.transform.translation,
                    orientation=tf.transform.rotation,
                ),
                scale=Vector3(sqrt(tf.fiducial_area) / 1000,
                              sqrt(tf.fiducial_area) / 1000,
                              0.01),
                color=ColorRGBA(1, 0, 0, 1),
            )
            for tf in msg.transforms
        ]

        self._vis_pub.publish(MarkerArray(markers=markers))


if __name__ == '__main__':

    rospy.init_node('fiducial_broadcaster_node')

    broadcaster = FiducialVisualizer("fiducial_transforms")

    rospy.spin()
