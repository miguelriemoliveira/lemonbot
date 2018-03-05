#include <cmath>
#include <iostream>
#include <map>

#include <ros/ros.h>

#include <laser_geometry/laser_geometry.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <actionlib/client/simple_action_client.h>

#include <flir_pantilt_d46/PtuGotoAction.h>

int main()
{
  sensor_msgs::LaserScan ls;
  for (float r : { 0.0, 0.1, 0.2, 0.3, 0.4, 0.5 })
  {
    ls.ranges.push_back(r);
    ls.intensities.push_back(1.0);
  }
  sensor_msgs::PointCloud pc;
  laser_geometry::LaserProjection _projector;
  _projector.projectLaser(ls, pc);

  std::cout << pc.points.size() << std::endl;
}