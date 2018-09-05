import numpy as np
import rospy
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

from .laserscans import Laserscans
from .images import Images

class LemonbotExporter(object):

    def __init__(self, image_topic, laserscan_topic, frames):
        self._image_topic = image_topic
        self._laserscans_topic = laserscan_topic
        self._ptu_source_frame = frames["base_link"]
        self._ptu_target_frame = frames["ptu_mount_link"]
        self._camera_frame = frames["camera_link"]
        self._laser_frame = frames["laser_link"]

        self._tf_buffer = Buffer()
        self._tf_list = TransformListener(self._tf_buffer)

        self._cv_brd = CvBridge()
    
        self._image_sub = rospy.Subscriber(
            self._image_topic,
            Image,
            callback=self._image_callback,
            queue_size=10,
        )
        
        self._laserscan_sub = rospy.Subscriber(
            self._laserscans_topic,
            LaserScan,
            callback=self._laserscan_callback,
            queue_size=100,
        )

        self._laserscans = Laserscans()
        self._images = Images()

        while len(self._laserscans) < 10: pass

        laser_params = self._laserscans.parameters
        laser_params['transform'] = self.get_laser_transform().tolist()

        self._params = {
            'camera': {
                'intrinsics': self.get_camera_info(),
                'transform': self.get_camera_transform().tolist(),
            },
            'laser': laser_params,
        }
    
        print(self._params)

    def save(self, path):
        from os.path import join
        import yaml

        with open(join(path, "params.yaml"), 'w') as f:
            yaml.dump(self._params, f)

        self._laserscans.save(path)
        self._images.save(path)
    
    def get_camera_info(self):
        msg = rospy.wait_for_message("/camera/camera_info", CameraInfo)
        return {
            'width': msg.width,
            'height': msg.height,
            'distortion_model': msg.distortion_model,
            'D': list(msg.D),
            'K': list(msg.K),
            'R': list(msg.R),
            'P': list(msg.P),
        }

    def get_camera_transform(self):
        tf = self._tf_buffer.lookup_transform(
            source_frame=self._camera_frame,
            target_frame=self._ptu_target_frame,
            time=rospy.Duration(0), timeout=rospy.Duration(1.0)
        )
        return self._process_transform(tf)

    def get_laser_transform(self):
        tf = self._tf_buffer.lookup_transform(
            source_frame=self._laser_frame,
            target_frame=self._ptu_target_frame,
            time=rospy.Duration(0), timeout=rospy.Duration(1.0)
        )
        return self._process_transform(tf)

    def _laserscan_callback(self, ls):
        transform = self.get_ptu_transform(ls.header.stamp)
        self._laserscans.append(ls, transform)

        print("received laserscan %d" % len(self._laserscans))

    def _image_callback(self, img):
        timestamp = img.header.stamp.to_nsec()
        np_img = self._cv_brd.imgmsg_to_cv2(img)
        transform = self.get_ptu_transform(img.header.stamp)
        self._images.append(timestamp, np_img, transform)

        print("received image %d" % len(self._images))
    

    def _process_transform(self, tf):
        t, r = tf.transform.translation, tf.transform.rotation

        return np.array([t.x, t.y, t.z, r.x, r.y, r.z, r.w])

    def get_ptu_transform(self, stamp):
        tf = self._tf_buffer.lookup_transform(
            source_frame=self._ptu_target_frame,
            target_frame=self._ptu_source_frame,
            time=stamp, timeout=rospy.Duration(1.0)
        )
        return self._process_transform(tf)