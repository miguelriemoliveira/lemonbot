#include <lemonbot_acquisition/laser_projector.h>

LaserProjector::LaserProjector(std::string ls_topic, std::string pc_topic)
  : Transformer(ls_topic, pc_topic, std::bind(&LaserProjector::transform, this, std::placeholders::_1))
{
}

PointCloud LaserProjector::transform(const sensor_msgs::LaserScan& ls)
{
  PointCloud cloud;

  pcl_conversions::toPCL(ls.header, cloud.header);
  cloud.header.seq = ls.header.seq;
  cloud.header.frame_id = ls.header.frame_id;

  cloud.height = 1;

  auto size = ls.ranges.size();
  cloud.width = size;

  cloud.points.reserve(cloud.width);

  for (auto step = 0; step < size; step++)
  {
    float angle = ls.angle_min + step * ls.angle_increment;
    float range = ls.ranges[step];

    auto point = pcl::PointXYZ{ range * std::cos(angle), range * std::sin(angle), 0 };
    cloud.points.push_back(point);
  }

  return cloud;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "laser_projector");

  auto laser_projector = LaserProjector("input", "output");

  ros::spin();
}
