#include <lemonbot_laser_pipeline/laser_angle_tweak.h>

using namespace lemonbot::laser_pipeline;

LaserAngleTweak::LaserAngleTweak(std::string in_topic, std::string out_topic, float angle_multiplier)
  : _angle_multiplier(angle_multiplier)
  , Transformer(in_topic, out_topic, std::bind(&LaserAngleTweak::transform, this, std::placeholders::_1))
{
}

sensor_msgs::LaserScan LaserAngleTweak::transform(const sensor_msgs::LaserScan &ls)
{
  auto ls_out = sensor_msgs::LaserScan{};
  ls_out.header = ls.header;
  ls_out.angle_min = ls.angle_min * _angle_multiplier;
  ls_out.angle_max = ls.angle_max * _angle_multiplier;
  ls_out.angle_increment = ls.angle_increment * _angle_multiplier;
  ls_out.time_increment = ls.time_increment;
  ls_out.scan_time = ls.scan_time;
  ls_out.range_min = ls.range_min;
  ls_out.range_max = ls.range_max;
  ls_out.ranges = ls.ranges;
  ls_out.intensities = ls.intensities;

  return ls_out;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_angle_tweak");

  auto nh = ros::NodeHandle{ "~" };

  float angle_multiplier;
  auto got_all_params = nh.getParam("angle_multiplier", angle_multiplier);

  if (!got_all_params)
    throw std::runtime_error("got no params");

  auto laser_range_filter = LaserAngleTweak("input", "output", angle_multiplier);

  ros::spin();

  return 0;
}