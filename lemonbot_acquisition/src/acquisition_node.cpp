#include <lemonbot_acquisition/acquisition_node.h>

using namespace lemonbot;
using namespace std;

AcquisitionNode::AcquisitionNode(Options& opts)
  : _opts(opts), _ptu_client(opts.ptu_topic, true), _done_pub(_nh.advertise<std_msgs::Bool>(_opts.done_topic, 1))
{
  if (!_ptu_client.waitForServer(_opts.timeout))
  {
    throw(std::runtime_error("wait for server timeout"));
  }

  ROS_INFO("Connected to PtuActionServer");
}

void AcquisitionNode::start(Params& params)
{
  _params = params;

  gotoPan(_params.min, _opts.max_vel);

  if (_opts.type == Type::CONTINUOUS)
  {
    startContinuous();
  }
  else
  {
    startPoint2Point();
  }

  auto done = std_msgs::Bool{};
  done.data = true;
  _done_pub.publish(done);
}

void AcquisitionNode::startContinuous()
{
  auto passthrough = Passthrough<sensor_msgs::LaserScan>(_opts.laser_in_topic, _opts.laser_out_topic);

  gotoPan(_params.max, _params.vel);
}

void AcquisitionNode::startPoint2Point()
{
  auto delta = (_params.max - _params.min) / (_params.nsteps - 1);
  for (int step = 0; step < _params.nsteps; step++)
  {
    auto pan = _params.min + step * delta;

    gotoPan(pan, _params.vel);

    auto laser_pub = _nh.advertise<sensor_msgs::LaserScan>(_opts.laser_out_topic, 10);

    republish<sensor_msgs::LaserScan>(_opts.laser_in_topic, laser_pub);

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
