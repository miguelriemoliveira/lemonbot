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

template <>
void AcquisitionNode::startAcquisition<AcquisitionNode::Type::CONTINUOUS>(Params& params)
{
  auto passthrough = Passthrough<sensor_msgs::LaserScan>(_opts.laser_in_topic, _opts.laser_out_topic);

  gotoTiltPan(params.max, params.vel);
}

template <>
void AcquisitionNode::startAcquisition<AcquisitionNode::Type::POINT2POINT>(Params& params)
{
  auto delta = (params.max - params.min) / (params.nsteps - 1);
  for (int step = 0; step < params.nsteps; step++)
  {
    auto pan = params.min + step * delta;

    gotoTiltPan(pan, params.vel);

    auto laser_pub = _nh.advertise<sensor_msgs::LaserScan>(_opts.laser_out_topic, 10);

    republish<sensor_msgs::LaserScan>(_opts.laser_in_topic, laser_pub);

    std::this_thread::sleep_for(_opts.pause);
  }
}

template <>
void AcquisitionNode::startAcquisition<AcquisitionNode::Type::HYBRID>(Params& params)
{
  auto delta = (params.max - params.min) / (params.nsteps - 1);
  for (int step = 0; step < params.nsteps; step++)
  {
    auto pan = params.min + step * delta;

    auto passthrough = Passthrough<sensor_msgs::LaserScan>(_opts.laser_in_topic, _opts.laser_out_topic);

    gotoTiltPan(pan, params.vel);

    std::this_thread::sleep_for(_opts.pause);
  }
}

void AcquisitionNode::start(Params& params)
{
  using Type = AcquisitionNode::Type;

  gotoTiltPan(params.min, _opts.max_vel);

  switch (_opts.type)
  {
    case Type::CONTINUOUS:
      startAcquisition<Type::CONTINUOUS>(params);
      break;
    case Type::POINT2POINT:
      startAcquisition<Type::POINT2POINT>(params);
      break;
    case Type::HYBRID:
      startAcquisition<Type::HYBRID>(params);
      break;
  }

  auto done = std_msgs::Bool{};
  done.data = true;
  _done_pub.publish(done);
}

void AcquisitionNode::gotoPan(float pan, float pan_vel)
{
  gotoTiltPan(pan, pan_vel);
}

void AcquisitionNode::gotoTiltPan(float pan, float pan_vel, float tilt, float tilt_vel)
{
  auto goal = flir_pantilt_d46::PtuGotoGoal{};
  goal.pan = pan;
  goal.tilt = tilt;
  goal.pan_vel = pan_vel;
  goal.tilt_vel = tilt_vel;
  _ptu_client.sendGoal(goal);

  while (_ptu_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    ros::spinOnce();
}