#ifndef PASSTHROUGH_H
#define PASSTHROUGH_H

#include <ros/ros.h>

template <typename MsgType>
class Passthrough
{
public:
  Passthrough(std::string in_topic, std::string out_topic)
    : _pub(_nh.advertise<MsgType>(out_topic, 2))
    , _sub(_nh.subscribe<MsgType>(in_topic, 2, &Passthrough::receiveAndPublish, this))
  {
  }

protected:
  void receiveAndPublish(const typename MsgType::ConstPtr& msg)
  {
    _pub.publish(msg);
  }

private:
  ros::NodeHandle _nh;
  ros::Publisher _pub;
  ros::Subscriber _sub;
};

#endif
