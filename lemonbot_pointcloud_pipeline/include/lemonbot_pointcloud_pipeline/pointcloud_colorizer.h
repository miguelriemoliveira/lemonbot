#ifndef POINTCLOUD_COLORIZER_HPP
#define POINTCLOUD_COLORIZER_HPP

#include <algorithm>
#include <optional>
#include <cmath>
#include <tuple>
#include <sstream>

#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>

#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>

#include <cv_bridge/cv_bridge.h>

#include <image_geometry/pinhole_camera_model.h>

namespace lemonbot::pointcloud_pipeline
{

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudWithColor;

class PointCloudColorizer
{
public:
  PointCloudColorizer(std::string image_topic, std::string pointcloud_topic, std::string pointcloud_color_topic);

protected:
  using PointCloudWithColor = pcl::PointCloud<pcl::PointXYZRGB>;
  using PointCloudWithoutColor = pcl::PointCloud<pcl::PointXYZ>;

  void receiveImage(const cv_bridge::CvImage::ConstPtr &img);

  void receivePointCloud(const PointCloudWithoutColor::ConstPtr &pc);

  /**
   * @brief colorizes an entire point cloud pointwise based on an input image.
   */
  PointCloudWithColor colorize(
      const PointCloudWithoutColor &pc,
      const cv_bridge::CvImage &img,
      const tf::StampedTransform &tf);

  /**
   * @brief colorizes a point based on it's projection on an image.
   */
  std::optional<pcl::PointXYZRGB> colorize(
      const pcl::PointXYZ &point,
      const cv_bridge::CvImage &img);

private:
  ros::NodeHandle _nh;
  ros::Subscriber _image_sub;
  ros::Subscriber _pointcloud_sub;

  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformListener _tf_listener;

  ros::Publisher _pointcloud_color_pub;

  std::vector<std::pair<cv_bridge::CvImage, tf::StampedTransform>> _image_transforms;
  image_geometry::PinholeCameraModel _camera_model;
};
}; // namespace lemonbot::pointcloud_pipeline

#endif