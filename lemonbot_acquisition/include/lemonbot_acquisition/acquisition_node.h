#ifndef LEMONBOT_ACQUISITION_NODE_H
#define LEMONBOT_ACQUISITION_NODE_H

#import <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <flir_pantilt_d46/PtuGotoAction.h>

namespace lemonbot
{
class AcquisitionNode
{
public:
  struct Params
  {
    float min;     // The minimum angle of pan.
    float max;     // The maximum angle of pan.
    float nsteps;  // The number of steps between the min and maximum pan angle.
    float vel;     // The maximum angular velocity of the pan.
  };
  struct Options
  {
    ros::Duration timeout = ros::Duration(30);
    std::string ptu_topic = "/PtuGoto";
  };
  AcquisitionNode(Params params, Options opts = Options{});

  void start();

protected:
  void atEachPoint();
  void atBegin();
  void atEnd();

private:
  Params _params;
  Options _opts;

  ros::NodeHandle _nh;
  actionlib::SimpleActionClient _ptu_client;
  flir_pantilt_d46::PtuGotoGoal _goal;
};
}

#endif