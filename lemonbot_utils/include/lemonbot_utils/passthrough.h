#ifndef PASSTHROUGH_H
#define PASSTHROUGH_H

#include <ros/ros.h>

namespace lemonbot::utils {

/**
 * @brief generic ros message class that passes through a message from a topic
 *        to another topic, during it's lifetime.
 */
template <typename MsgType>
class Passthrough
{
public:
  /**
   * @brief initializes the passthrough, starting the republishing
   *        automatically.
   * @param in_topic the input topic.
   * @param out_topic the output topic.
   */
  Passthrough(std::string in_topic, std::string out_topic)
    : _pub(_nh.advertise<MsgType>(out_topic, 10))
    , _sub(_nh.subscribe<MsgType>(in_topic, 10, &Passthrough::receiveAndPublish, this))
  {
  }

protected:
  /**
   * @brief callback for the message subscriber.
   */
  void receiveAndPublish(const typename MsgType::ConstPtr& msg)
  {
    _pub.publish(msg);
  }

private:
  ros::NodeHandle _nh;
  ros::Publisher _pub;
  ros::Subscriber _sub;
};

}

#endif
