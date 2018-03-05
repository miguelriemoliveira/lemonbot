#include <functional>
#include <sstream>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include "lemonbot_commander/commander.hpp"
#include "lemonbot_commander/republisher.hpp"

using namespace lemonbot_acquisition;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "lemonbot_commander");

  auto nh = ros::NodeHandle{};

  auto repub = nh.advertise<std_msgs::Float32>("chatter2", 10);

  auto interpolation_params = Commander::InterpolationParams{ 0, 100, 101, 1.0f };

  auto interface_params = Commander::InterfaceParams{};

  auto commander = lemonbot_acquisition::Commander{ interpolation_params, interface_params };

  commander.addCallback([](float angle) { ROS_INFO("Going to angle %f", angle); });

  commander.addCallback([&](float angle) { lemonbot_acquisition::republisher<std_msgs::Float32>(repub, "chatter"); });

  auto rate = ros::Rate{ 100 };

  commander.addCallback([&](float _) { rate.sleep(); });

  commander.startAcquisition();

  return 0;
}