#ifndef LASER_ANGLE_FILTER_H
#define LASER_ANGLE_FILTER_H

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

#include <lemonbot_utils/transformer.h>

namespace lemonbot::laser_pipeline
{
using namespace lemonbot::utils;

class LaserAngleFilter : public Transformer<sensor_msgs::LaserScan, sensor_msgs::LaserScan>
{
public:
  LaserAngleFilter(std::string in_topic, std::string out_topic, float angle_min, float angle_max);

protected:
  sensor_msgs::LaserScan transform(const sensor_msgs::LaserScan& ls);

private:
  const float _angle_min;
  const float _angle_max;
};
}

#endif  // LASER_ANGLE_FILTER_H