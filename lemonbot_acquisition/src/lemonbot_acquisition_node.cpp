#include <cmath>
#include <iostream>
#include <map>

#include <ros/ros.h>

#include <laser_geometry/laser_geometry.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <actionlib/client/simple_action_client.h>

#include <flir_pantilt_d46/PtuGotoAction.h>

#include <laser_assembler/AssembleScans.h>
#include <laser_assembler/AssembleScans2.h>

using namespace std;
using namespace std::literals;

using namespace laser_assembler;

struct options
{
  string ptu = "/SetPTUState";
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

  ros::service::waitForService("assemble_scans");

  ros::ServiceClient client = nh.serviceClient<AssembleScans2>("assemble_scans2");

  AssembleScans2 srv;

  ROS_INFO("connected!");

  // create a new goal
  flir_pantilt_d46::PtuGotoGoal goal;
  goal.pan = -90;
  goal.tilt = 0.0f;
  goal.pan_vel = 10.0f;
  goal.tilt_vel = 10.0f;

  options.nsteps--;

  float delta = (options.max - options.min) / (options.nsteps);

  act_client.sendGoalAndWait(goal);

  srv.request.begin = ros::Time::now();

  // for (int step = 0; step <= (options.nsteps); ++step)
  // {
  //   float position = options.min + delta * step;
  //   position *= 180 / M_PI;
  //   ROS_INFO("goint to position %f", position);

  //   goal.pan = position;

  //   act_client.sendGoal(goal);

  //   auto done = act_client.waitForResult(ros::Duration(30.0));
  // }

  goal.pan = +90;
  act_client.sendGoalAndWait(goal);

  srv.request.end = ros::Time::now();

  client.call(srv);

  auto pub = nh.advertise<decltype(srv.response.cloud)>("result", 10);

  auto r = ros::Rate{ 10 };

  while (ros::ok())
  {
    pub.publish(srv.response.cloud);
    r.sleep();
  }

  return 0;
}