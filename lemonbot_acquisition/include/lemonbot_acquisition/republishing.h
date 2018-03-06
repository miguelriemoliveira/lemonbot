#ifndef LEMONBOT_ACQUISITION_REPUBLISHING_H
#define LEMONBOT_ACQUISITION_REPUBLISHING_H

#include <functional>
#include <string>

#include <ros/ros.h>

namespace lemonbot
{
template <typename MsgType>
void republish(std::string topic_name, ros::Publisher& pub, ros::Duration timeout = ros::Duration(0))
{
  auto msg = ros::topic::waitForMessage<MsgType>(topic_name, timeout);
  pub.publish(msg);
}

template <typename InMsgType, typename OutMsgType>
void republishWithTransformation(std::string& topic_name, ros::Publisher pub,
                                 std::function<OutMsgType(InMsgType)> transform)
{
  auto msg = ros::topic::waitForMessage<InMsgType>(topic_name);
  auto transformed = transform(msg);
  pub.publish(transformed);
}
}

#endif