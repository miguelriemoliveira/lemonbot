#include <cmath>
#include <iostream>
#include <map>

#include <ros/ros.h>

#include <laser_geometry/laser_geometry.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <actionlib/client/simple_action_client.h>

#include <flir_pantilt_d46/PtuGotoAction.h>

using namespace std;
using namespace std::literals;

struct options
{
  string ptu = "/cmd";
  float min = -M_PI / 2;
  float max = M_PI / 2;
  float nsteps = 100;
  float vel = 0.5f;
} options;

int main(int argc, char* argv[])
{
  // Initialize ROS connections
  ros::init(argc, argv, "lemonbot_acquisition_node");

  auto nh = ros::NodeHandle{};

  // Create actionlib client for ptu
  actionlib::SimpleActionClient<flir_pantilt_d46::PtuGotoAction> act_client(options.ptu, true);

  ROS_INFO("started ptu action client at %s", options.ptu.c_str());

  // wait for server to acknoledge
  act_client.waitForServer();

  auto _projector = laser_geometry::LaserProjection{};

  // create a new goal
  flir_pantilt_d46::PtuGotoGoal goal;
  goal.pan = 0.0f;
  goal.tilt = 0.0f;
  goal.pan_vel = options.vel;
  goal.tilt_vel = 1.0f;

  options.nsteps--;

  float delta = (options.max - options.min) / (options.nsteps);

  for (int step = 0; step <= (options.nsteps); ++step)
  {
    float position = options.min + delta * step;
    ROS_INFO("goint to position %f", position);

    goal.pan = position;

    act_client.sendGoal(goal);

    auto done = act_client.waitForResult(ros::Duration(30.0));
  }

  return 0;
}