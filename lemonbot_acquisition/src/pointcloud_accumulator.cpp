#include <lemonbot_acquisition/pointcloud_accumulator.h>

PointcloudAccumulator::PointcloudAccumulator(std::string input, std::string output, std::string done_topic,
                                             std::string result_topic)
  : Transformer(input, output, std::bind(&PointcloudAccumulator::accumulate, this, std::placeholders::_1))
  , _done_sub(_nh.subscribe<std_msgs::Bool>(done_topic, 1, &PointcloudAccumulator::done, this))
  , _result_pub(_nh.advertise<PointCloud>(result_topic, 1))
{
}

PointCloud PointcloudAccumulator::accumulate(const PointCloud& pc)
{
  _accumulated.header = pc.header;
  _accumulated += pc;
  return _accumulated;
}

void PointcloudAccumulator::done(const std_msgs::Bool::ConstPtr&)
{
  _result_pub.publish(_accumulated);

  _accumulated.points.clear();
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "lemonbot_accumulator");

  PointcloudAccumulator accumulator("input", "output", "done", "result");

  ros::spin();

  return 0;
}
