#include <algorithm>
#include <functional>

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

#include <lemonbot_acquisition/transformer.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class LaserProjector : public Transformer<sensor_msgs::LaserScan, PointCloud>
{
public:
  LaserProjector(std::string ls_topic, std::string pc_topic)
    : Transformer(ls_topic, pc_topic, std::bind(&LaserProjector::transform, this, std::placeholders::_1))
  {
  }

protected:
  PointCloud transform(const sensor_msgs::LaserScan& ls)
  {
    PointCloud cloud;

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

private:
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "laser_projector");

  LaserProjector("/laserscan", "laserpoints");
}
