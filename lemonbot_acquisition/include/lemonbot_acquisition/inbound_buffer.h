#ifndef INBOUND_BUFFER_H
#define INBOUND_BUFFER_H

#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include <ros/ros.h>

template <typename MsgType>
class InboundBuffer
{
public:
  InboundBuffer(std::string topic)
    : _sub(_nh.subscribe<MsgType>(topic, 10, &InboundBuffer::receiveMessage, this)), _available(true)
  {
  }

  MsgType receive()
  {
    reset();

    for (; ros::ok(); ros::spinOnce())
    {
      std::lock_guard lock(_mtx);
      if (_available)
      {
        break;
      }
    }
    return _msg;
  }

protected:
  void receiveMessage(const typename MsgType::ConstPtr& msg)
  {
    std::lock_guard lock(_mtx);
    _msg = *msg;
    _available = true;
  }

  void reset()
  {
    std::lock_guard lock(_mtx);
    _available = false;
  }

private:
  ros::NodeHandle _nh;
  ros::Subscriber _sub;
  MsgType _msg;
  bool _available;
  std::mutex _mtx;
};

#endif