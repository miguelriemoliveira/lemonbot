#include <lemonbot_acquisition/acquisition_node.h>

using namespace lemonbot;
using namespace std;

AcquisitionNode::AcquisitionNode(Params params, Options opts)
  : _params(params)
  , _opts(opts)
  , _ptu_client(opts.ptu_topic, true)
  , _laser_pub(_nh.advertise<sensor_msgs::LaserScan>(_opts.laser_out_topic, 10))
{
  ostringstream info;

  info << "AcquisitionNode started with params { "
       << "min: " << _params.min << "; "
       << "max: " << _params.max << "; "
       << "nsteps: " << _params.nsteps << "; "
       << "vel: " << _params.vel << "; "
       << "}";

  ROS_INFO("%s", info.str().c_str());

  if (!_ptu_client.waitForServer(_opts.timeout))
  {
    throw(std::runtime_error("wait for server timeout"));
  }

  ROS_INFO("Connected to PtuActionServer");
}

void AcquisitionNode::start()
{
  gotoPan(_params.min, _opts.max_vel);

  ROS_INFO("Reached minimum position");

  if (_opts.type == Type::CONTINUOUS)
  {
    startContinuous();
  }
  else
  {
    startPoint2Point();
  }

  ROS_INFO("Finished this capture");
}

void AcquisitionNode::startContinuous()
{
  auto sub = _nh.subscribe<sensor_msgs::LaserScan>(
      _opts.laser_in_topic, 10, [&](const sensor_msgs::LaserScanConstPtr& ls) { _laser_pub.publish(ls); });

  gotoPan(_params.max, _params.vel);
}

void AcquisitionNode::startPoint2Point()
{
  auto delta = (_params.max - _params.min) / (_params.nsteps - 1);
  for (int step = 0; step < _params.nsteps; step++)
  {
    auto pan = _params.min + step * delta;

    gotoPan(pan, _params.vel);

    auto laser_scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>(_opts.laser_in_topic);

    _laser_pub.publish(laser_scan);

    std::this_thread::sleep_for(_opts.pause);
  }
}

void AcquisitionNode::gotoPan(float angle, float velocity)
{
  auto goal = flir_pantilt_d46::PtuGotoGoal{};
  goal.pan = angle;
  goal.tilt = 0.0f;
  goal.pan_vel = velocity;
  goal.tilt_vel = 10.0f;
  _ptu_client.sendGoal(goal);

  while (_ptu_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    ros::spinOnce();
}
