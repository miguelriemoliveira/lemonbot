#ifndef WAIT_FOR_DONE_H
#define WAIT_FOR_DONE_H

#include <mutex>

#include <ros/ros.h>

#include <std_msgs/Bool.h>

template <typename MsgType>
class WaitForDone
{
public:
  WaitForDone(std::string input, std::string output, std::string done_topic)
    : _msg_sub(_nh.subscribe<MsgType>(input, 10, &WaitForDone::msgCallback, this))
    , _msg_pub(_nh.advertise<MsgType>(output, 10))
    , _done_sub(_nh.subscribe<std_msgs::Bool>(done_topic, 1, &WaitForDone::doneCallBack, this))
  {
  }

protected:
  void msgCallback(const typename MsgType::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(_mtx);
    _last_msg = *msg;
  }
  void doneCallBack(const std_msgs::Bool::ConstPtr& done)
  {
    std::lock_guard<std::mutex> lock(_mtx);
    _msg_pub.publish(_last_msg);
  }

private:
  ros::NodeHandle _nh;
  ros::Subscriber _msg_sub;
  ros::Publisher _msg_pub;
  ros::Subscriber _done_sub;

  MsgType _last_msg;

  std::mutex _mtx;
};

#endif