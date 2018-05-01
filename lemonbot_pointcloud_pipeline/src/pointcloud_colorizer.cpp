#include <lemonbot_pointcloud_pipeline/pointcloud_colorizer.h>

using namespace lemonbot::pointcloud_pipeline;

PointCloudColorizer::PointCloudColorizer(std::string image_topic, std::string pointcloud_topic,
                                         std::string pointcloud_color_topic)
    : _image_sub(
          _nh.subscribe<sensor_msgs::Image>(image_topic + "/image_raw", 10, &PointCloudColorizer::receiveImage, this)),
      _pointcloud_sub(
          _nh.subscribe<PointCloudWithoutColor>(pointcloud_topic, 10, &PointCloudColorizer::receivePointCloud, this)),
      _pointcloud_color_pub(_nh.advertise<PointCloudWithColor>(pointcloud_color_topic, 10))
{
}

void PointCloudColorizer::receiveImage(const sensor_msgs::Image::ConstPtr &img)
{
  _images.push_back(*img);
}

void PointCloudColorizer::receivePointCloud(const PointCloudWithoutColor::ConstPtr &pc)
{
  for (const auto &img : _images)
  {
    auto colorized = PointCloudColorizer::colorize(*pc, img);

    _pointcloud_color_pub.publish(colorized);
  }
}

PointCloudColorizer::PointCloudWithColor PointCloudColorizer::colorize(const PointCloudWithoutColor &pc,
                                                                       const sensor_msgs::Image &img)
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

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pointcloud_colorizer");

  auto colorizer = PointCloudColorizer("/acquisition/images", "/result", "colorized");

  ros::spin();
}
