#ifndef LEMONBOT_ACQUISITION_NODE_H
#define LEMONBOT_ACQUISITION_NODE_H

#include <chrono>
#include <exception>
#include <sstream>
#include <string>
#include <thread>

#import <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <flir_pantilt_d46/PtuGotoAction.h>

namespace lemonbot
{
class AcquisitionNode
{
public:
  enum Type
  {
    CONTINUOUS,
    POINT2POINT,
  };
  struct Params
  {
    float min;     // The minimum angle of pan.
    float max;     // The maximum angle of pan.
    float nsteps;  // The number of steps between the min and maximum pan angle.
    float vel;     // The maximum angular velocity of the pan.
  };
  struct Options
  {
    float max_vel;
    Type type = Type::CONTINUOUS;
    ros::Duration timeout;
    std::string ptu_topic;
    std::chrono::milliseconds pause;
  };
  AcquisitionNode(Params params, Options opts);

  void start();

protected:
  void atEachPoint();
  void atBegin();
  void atEnd();

private:
  ros::NodeHandle _nh;

  Params _params;
  Options _opts;

  actionlib::SimpleActionClient<flir_pantilt_d46::PtuGotoAction> _ptu_client;
};
}

#endif