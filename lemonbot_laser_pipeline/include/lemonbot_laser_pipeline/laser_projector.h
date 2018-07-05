#ifndef LASER_PROJECTOR_H
#define LASER_PROJECTOR_H

#include <algorithm>
#include <functional>

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

#include <lemonbot_utils/transformer.h>

namespace lemonbot::laser_pipeline
{
using namespace lemonbot::utils;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class LaserProjector : public Transformer<sensor_msgs::LaserScan, PointCloud>
{
public:
  LaserProjector(std::string ls_topic, std::string pc_topic);

protected:
  PointCloud transform(const sensor_msgs::LaserScan& ls);

private:
};
}

#endif