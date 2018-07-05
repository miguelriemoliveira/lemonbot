#include <lemonbot_laser_pipeline/laser_angle_filter.h>

using namespace lemonbot::laser_pipeline;

LaserAngleFilter::LaserAngleFilter(std::string in_topic, std::string out_topic, float angle_min, float angle_max)
  : _angle_min(angle_min)
  , _angle_max(angle_max)
  , Transformer(in_topic, out_topic, std::bind(&LaserAngleFilter::transform, this, std::placeholders::_1))
{
}

sensor_msgs::LaserScan LaserAngleFilter::transform(const sensor_msgs::LaserScan& ls)
{
  int min_idx = (_angle_min - ls.angle_min) / ls.angle_increment;
  min_idx = min_idx >= 0 ? min_idx : 0;

  int max_idx = (ls.angle_max - _angle_max) / ls.angle_increment;
  max_idx = max_idx > 0 ? max_idx : 0;

  float angle_min = ls.angle_min + min_idx * ls.angle_increment;
  float angle_max = ls.angle_max - max_idx * ls.angle_increment;

  auto ls_out = sensor_msgs::LaserScan{};
  ls_out.header = ls.header;
  ls_out.angle_min = angle_min;
  ls_out.angle_max = angle_max;
  ls_out.angle_increment = ls.angle_increment;
  ls_out.time_increment = ls.time_increment;
  ls_out.scan_time = ls.scan_time;
  ls_out.range_min = ls.range_min;
  ls_out.range_max = ls.range_max;
  ls_out.ranges.assign(ls.ranges.begin() + min_idx, ls.ranges.end() - max_idx);
  ls_out.intensities.assign(ls.intensities.begin() + min_idx, ls.intensities.end() - max_idx);

  return ls_out;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_range_filter");

  auto nh = ros::NodeHandle{ "~" };

  float angle_min;
  float angle_max;
  auto got_all_params = nh.getParam("angle_min", angle_min) && nh.getParam("angle_max", angle_max);

  if (!got_all_params)
    throw std::runtime_error("got no params");

  auto laser_range_filter = LaserAngleFilter("input", "output", angle_min, angle_max);

  ros::spin();
}