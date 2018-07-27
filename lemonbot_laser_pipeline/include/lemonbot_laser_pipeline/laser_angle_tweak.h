#ifndef LASER_ANGLE_TWEAK_H
#define LASER_ANGLE_TWEAK_H

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

#include <lemonbot_utils/transformer.h>

namespace lemonbot::laser_pipeline
{
using namespace lemonbot::utils;

class LaserAngleTweak : public Transformer<sensor_msgs::LaserScan, sensor_msgs::LaserScan>
{
public:
  LaserAngleTweak(std::string in_topic, std::string out_topic, float angle_multiplier);

protected:
  sensor_msgs::LaserScan transform(const sensor_msgs::LaserScan &ls);

private:
  const float _angle_multiplier;
};
}  // namespace lemonbot::laser_pipeline

#endif