#ifndef POINTCLOUD_COLORIZER_HPP
#define POINTCLOUD_COLORIZER_HPP

#include <algorithm>

#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>

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

  void receiveImage(const sensor_msgs::Image::ConstPtr &img);

  void receivePointCloud(const PointCloudWithoutColor::ConstPtr &pc);

  static PointCloudWithColor colorize(const PointCloudWithoutColor &pc, const sensor_msgs::Image &img);

  static pcl::PointXYZRGB colorize(const pcl::PointXYZ &point,
                                   const sensor_msgs::Image &img,
                                   const sensor_msgs::CameraInfo &camera_info);

private:
  ros::NodeHandle _nh;
  ros::Subscriber _image_sub;
  ros::Subscriber _pointcloud_sub;

  ros::Publisher _pointcloud_color_pub;

  std::vector<sensor_msgs::Image> _images;
  sensor_msgs::CameraInfo _camera_info;
};
};

#endif