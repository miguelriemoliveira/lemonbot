#ifndef POINTCLOUD_COLORIZER_HPP
#define POINTCLOUD_COLORIZER_HPP

#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>

namespace lemonbot::acquisition
{
class PointCloudColorizer
{
public:
  PointCloudColorizer(std::string image_topic, std::string pointcloud_topic, std::string pointcloud_color_topic);

protected:
  using PointCloudWithColor = pcl::PointCloud<pcl::PointXYZRGB>;
  using PointCloudWithoutColor = pcl::PointCloud<pcl::PointXYZ>;

  void receiveImage(const sensor_msgs::Image::ConstPtr& img)
  {
    _images.push_back(*img);
  }

  void receivePointCloud(const PointCloudWithoutColor::ConstPtr& pc)
  {
    for (const auto& img : _images)
    {
      auto colorized = PointCloudColorizer::colorize(*pc, img);

      _pointcloud_color_pub.publish(colorized);
    }
  }

  static PointCloudWithColor colorize(const PointCloudWithoutColor& pc, const sensor_msgs::Image& img)
  {
    PointCloudWithColor new_pc;

    for (auto point : pc.points)
    {
      pcl::PointXYZRGB p;
      p.x = point.x;
      p.y = point.y;
      p.z = point.z;
      new_pc.points.push_back(p);
    }

    return new_pc;
  }

private:
  ros::NodeHandle _nh;
  ros::Subscriber _image_sub;
  ros::Subscriber _pointcloud_sub;

  ros::Publisher _pointcloud_color_pub;

  std::vector<sensor_msgs::Image> _images;
};

PointCloudColorizer::PointCloudColorizer(std::string image_topic, std::string pointcloud_topic,
                                         std::string pointcloud_color_topic)
  : _image_sub(_nh.subscribe<sensor_msgs::Image>(image_topic, 10, &PointCloudColorizer::receiveImage, this))
  , _pointcloud_sub(
        _nh.subscribe<PointCloudWithoutColor>(pointcloud_topic, 10, &PointCloudColorizer::receivePointCloud, this))
  , _pointcloud_color_pub(_nh.advertise<PointCloudWithColor>(pointcloud_color_topic, 10))
{
}
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pointcloud_colorizer");

  auto colorizer = lemonbot::acquisition::PointCloudColorizer("/acquisition/images", "/result", "colorized");

  ros::spin();
}

#endif  // POINTCLOUD_COLORIZER_HPP