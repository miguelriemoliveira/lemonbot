#ifndef LEMONBOT_ACQUISITION_NODE_H
#define LEMONBOT_ACQUISITION_NODE_H

#include <chrono>
#include <exception>
#include <sstream>
#include <string>
#include <thread>

#import <ros/ros.h>

#include <std_msgs/Bool.h>

#include <sensor_msgs/LaserScan.h>

#include <actionlib/client/simple_action_client.h>

#include <flir_pantilt_d46/PtuGotoAction.h>

#include <lemonbot_acquisition/inbound_buffer.h>
#include <lemonbot_acquisition/passthrough.h>
#include <lemonbot_acquisition/republishing.h>

namespace lemonbot
{
class AcquisitionNode
{
public:
  enum Type
  {
    CONTINUOUS,
    POINT2POINT,
    HYBRID,
  };
  struct Params
  {
    float min;     // The minimum angle of pan.
    float max;     // The maximum angle of pan.
    float nsteps;  // The number of steps between the min and maximum pan angle.
    float vel;     // The maximum angular velocity of the pan.
    Type type;
  };
  struct Options
  {
    float max_vel;
    ros::Duration timeout = ros::Duration{ 5 };
    std::string ptu_topic;
    std::string laser_out_topic;
    std::string laser_in_topic;
    std::string done_topic;
    std::chrono::milliseconds pause = std::chrono::milliseconds{ 300 };
  };
  AcquisitionNode(Options& opts);

  void start(Params& params);

protected:
  template <Type mode>
  void startAcquisition(Params& params);

  void gotoTiltPan(float pan, float pan_vel, float tilt = 0.0f, float tilt_vel = 0.0f);

private:
  ros::NodeHandle _nh;

  Params _params;
  Options _opts;

  actionlib::SimpleActionClient<flir_pantilt_d46::PtuGotoAction> _ptu_client;

  ros::Publisher _done_pub;
};
}

#endif