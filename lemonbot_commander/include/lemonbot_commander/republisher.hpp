#include "ros/ros.h"

namespace lemonbot_acquisition
{
template <typename T>
void republisher(ros::Publisher& sub, const std::string& topic)
{
  auto msg = ros::topic::waitForMessage<T>(topic);

  sub.publish(msg);
}
}
