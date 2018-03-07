#include <lemonbot_acquisition/pointcloud_accumulator.h>

PointcloudAccumulator::PointcloudAccumulator(std::string input, std::string output)
  : Transformer(input, output, std::bind(&PointcloudAccumulator::accumulate, this, std::placeholders::_1))
{
}

PointCloud PointcloudAccumulator::accumulate(const PointCloud& pc)
{
  _accumulated.header = pc.header;
  _accumulated += pc;
  return _accumulated;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "lemonbot_accumulator");

  PointcloudAccumulator accumulator("input", "output");

  ros::spin();

  return 0;
}