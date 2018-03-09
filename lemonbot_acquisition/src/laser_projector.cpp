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

  auto min = ls.range_min;
  auto max = ls.range_max;

  cloud.points.reserve(ls.ranges.size());

  for (auto step = 0; step < ls.ranges.size(); step++)
  {
    float angle = ls.angle_min + step * ls.angle_increment;
    float range = ls.ranges[step];

    if (range > min && range < max)
    {
      auto point = pcl::PointXYZ{ range * std::cos(angle), range * std::sin(angle), 0 };
      cloud.points.push_back(point);
    }
  }

  cloud.width = cloud.points.size();

  return cloud;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "laser_projector");

  auto laser_projector = LaserProjector("input", "output");

  ros::spin();
}
