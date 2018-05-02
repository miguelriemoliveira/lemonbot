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
  _camera_info = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>(image_topic + "/camera_info", ros::Duration(0));
}

void PointCloudColorizer::receiveImage(const sensor_msgs::Image::ConstPtr &img)
{
  _images.push_back(*img);
}

void PointCloudColorizer::receivePointCloud(const PointCloudWithoutColor::ConstPtr &pc)
{
}

PointCloudWithColor PointCloudColorizer::colorize(
    const PointCloudWithoutColor &pc,
    const sensor_msgs::Image &img)
{
  PointCloudWithColor colorized_pc;

  return colorized_pc;
}

pcl::PointXYZRGB PointCloudColorizer::colorize(
    const pcl::PointXYZ &point,
    const sensor_msgs::Image &img,
    const sensor_msgs::CameraInfo& camera_info)
{

}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pointcloud_colorizer");

  auto colorizer = PointCloudColorizer("/acquisition/images", "/result", "colorized");

  ros::spin();
}
