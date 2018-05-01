#ifndef TRANSFORMER_H
#define TRANSFORMER_H

#include <functional>

#include <ros/ros.h>

namespace lemonbot::utils {

template <typename InMsgType, typename OutMsgType>
class Transformer
{
  typedef std::function<OutMsgType(const InMsgType&)> transformer;

public:
  Transformer(std::string in_topic, std::string out_topic, transformer transform)
    : _pub(_nh.advertise<OutMsgType>(out_topic, 2))
    , _sub(_nh.subscribe<InMsgType>(in_topic, 2, &Transformer::receiveAndTransform, this))
    , _transform(transform)
  {
  }

protected:
  void receiveAndTransform(const typename InMsgType::ConstPtr& msg)
  {
    auto transformed = _transform(*msg);
    _pub.publish(std::move(transformed));
  }

private:
  ros::NodeHandle _nh;
  ros::Publisher _pub;
  ros::Subscriber _sub;
  transformer _transform;
};

}

#endif