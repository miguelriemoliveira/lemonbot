#include <lemonbot_pointcloud_pipeline/pointcloud_colorizer.h>

using namespace lemonbot::pointcloud_pipeline;

PointCloudColorizer::PointCloudColorizer(std::string image_topic, std::string pointcloud_topic,
                                         std::string pointcloud_color_topic)
    : _image_sub(
          _nh.subscribe<cv_bridge::CvImage>(image_topic, 10, &PointCloudColorizer::receiveImage, this)),
      _pointcloud_sub(
          _nh.subscribe<PointCloudWithoutColor>(pointcloud_topic, 10, &PointCloudColorizer::receivePointCloud, this)),
      _pointcloud_color_pub(_nh.advertise<PointCloudWithColor>(pointcloud_color_topic, 10)),
      _tf_listener(_tf_buffer)
{
  auto camera_info = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/camera_info", ros::Duration(0));

  _camera_model.fromCameraInfo(camera_info);
}

void PointCloudColorizer::receiveImage(const cv_bridge::CvImage::ConstPtr &img)
{
  ROS_INFO("received image");

  geometry_msgs::TransformStamped transformStamped;

  while (!_tf_buffer.canTransform("lemonbot_base_link",
                                  "lemonbot_camera_mount_link",
                                  img->header.stamp))
    ;

  transformStamped = _tf_buffer.lookupTransform("lemonbot_base_link",
                                                "lemonbot_camera_mount_link",
                                                img->header.stamp);
  tf::StampedTransform transform;
  tf::transformStampedMsgToTF(transformStamped, transform);

  _image_transforms.push_back(std::make_pair(*img, transform));

  ROS_INFO("found transformation");
}

void PointCloudColorizer::receivePointCloud(const PointCloudWithoutColor::ConstPtr &pc)
{
  ROS_INFO("received pointcloud");

  for (const auto & [ img, tf ] : _image_transforms)
  {
    ROS_INFO("colorizing with image");

    const auto colorized_pc = colorize(*pc, img, tf);
    _pointcloud_color_pub.publish(colorized_pc);
  }
}

PointCloudWithColor PointCloudColorizer::colorize(
    const PointCloudWithoutColor &pc,
    const cv_bridge::CvImage &img,
    const tf::StampedTransform &tf)
{
  PointCloudWithoutColor transformed;
  pcl_ros::transformPointCloud(pc, transformed, tf);

  PointCloudWithColor colorized_pc;
  colorized_pc.points.reserve(pc.points.size());

  pcl::PointXYZRGB nan_point;
  nan_point.x = NAN;
  nan_point.y = NAN;
  nan_point.z = NAN;
  nan_point.r = 0;
  nan_point.g = 0;
  nan_point.b = 0;

  for (const auto &point : transformed.points)
  {
    auto new_point = colorize(point, img);

    if (new_point)
    {
      colorized_pc.push_back(*new_point);
    }
    else
    {
      colorized_pc.push_back(nan_point);
    }
  }

  pcl_ros::transformPointCloud(colorized_pc, colorized_pc, tf.inverse());

  return colorized_pc;
}

std::optional<pcl::PointXYZRGB> PointCloudColorizer::colorize(
    const pcl::PointXYZ &point,
    const cv_bridge::CvImage &img)
{
  if (point.z < 0)
    return std::nullopt;

  const auto[u, v] = _camera_model.project3dToPixel(cv::Point3d{point.x, point.y, point.z});

  const auto[width, height] = _camera_model.fullResolution();

  const bool within_range =
      u >= 0 && u < width &&
      v >= 0 && v < height;
  if (!within_range)
    return std::nullopt;

  const auto color = img.image.at<cv::Vec3b>(v, u);

  pcl::PointXYZRGB out;
  out.x = point.x;
  out.y = point.y;
  out.z = point.z;
  out.r = color[0];
  out.g = color[1];
  out.b = color[2];

  return out;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pointcloud_colorizer");

  auto colorizer = PointCloudColorizer("/acquisition/images", "/result", "/colorized");

  ros::spin();
}
