#ifndef POINTCLOUD_COLORIZER_HPP
#define POINTCLOUD_COLORIZER_HPP

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

namespace lemonbot::acquisition
{
class PointCloudColorizer
{
public:
  PointCloudColorizer(std::string image_topic, std::string pointcloud_topic);

protected:
  void receiveImage(const sensor_msgs::Image::ConstPtr& img)
  {
    ROS_WARN("received_image");
  }

  void receivePointCloud(const sensor_msgs::PointCloud2::ConstPtr& pc)
  {
    ROS_WARN("received_pointcloud");
  }

private:
  ros::NodeHandle _nh;
  ros::Subscriber _image_sub;
  ros::Subscriber _pointcloud_sub;
};

PointCloudColorizer::PointCloudColorizer(std::string image_topic, std::string pointcloud_topic)
  : _image_sub(_nh.subscribe<sensor_msgs::Image>(image_topic, 10, &PointCloudColorizer::receiveImage, this))
  , _pointcloud_sub(
        _nh.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic, 10, &PointCloudColorizer::receivePointCloud, this))
{
}
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pointcloud_colorizer");

  auto colorizer = lemonbot::acquisition::PointCloudColorizer("/acquisition/images", "/result");

  ros::spin();
}

#endif  // POINTCLOUD_COLORIZER_HPP