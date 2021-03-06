#include <lemonbot_acquisition/acquisition_node.h>

using namespace lemonbot;
using namespace lemonbot::acquisition;
using namespace lemonbot::utils;
using namespace std;

AcquisitionNode::AcquisitionNode(Options &opts)
    : _opts(opts), _ptu_client(opts.ptu_topic, true), _done_pub(_nh.advertise<std_msgs::Bool>(_opts.done_topic, 1))
{
  if (!_ptu_client.waitForServer(_opts.timeout))
  {
    throw(std::runtime_error("wait for server timeout"));
  }

  ROS_INFO("Connected to PtuActionServer");
}

template <>
void AcquisitionNode::startAcquisition<AcquisitionNode::Type::CONTINUOUS>(Params &params)
{
  InboundBuffer<sensor_msgs::Image> camera_buffer{_opts.camera_in_topic};
  auto camera_pub = _nh.advertise<sensor_msgs::Image>(_opts.camera_out_topic, 10);

  auto passthrough = Passthrough<sensor_msgs::LaserScan>(_opts.laser_in_topic, _opts.laser_out_topic);

  auto pan_delta = (params.pan.max - params.pan.min) / (params.pan.nsteps - 1);
  auto tilt_delta = params.tilt.nsteps > 1 ? (params.tilt.max - params.tilt.min) / (params.tilt.nsteps - 1) : 0.0f;

  for (int tilt_step = 0; tilt_step < params.tilt.nsteps; tilt_step++)
    for (int pan_step = 0; pan_step < params.pan.nsteps; pan_step++)
    {
      auto pan = (tilt_step % 2 == 0) ? params.pan.min + pan_step * pan_delta : params.pan.max - pan_step * pan_delta;
      auto tilt = params.tilt.min + tilt_step * tilt_delta;

      std::this_thread::sleep_for(_opts.pause);
      camera_pub.publish(camera_buffer.receive());

      gotoTiltPan(pan, params.pan.vel, tilt, params.tilt.vel);
    }
}

template <>
void AcquisitionNode::startAcquisition<AcquisitionNode::Type::POINT2POINT>(Params &params)
{
  throw(std::logic_error("not implemented"));
}

void AcquisitionNode::start(Params &params)
{
  using Type = AcquisitionNode::Type;

  gotoTiltPan(params.pan.min, _opts.pan_limits.vel);

  switch (params.type)
  {
  case Type::CONTINUOUS:
    startAcquisition<Type::CONTINUOUS>(params);
    break;
  case Type::POINT2POINT:
    startAcquisition<Type::POINT2POINT>(params);
    break;
  }

  auto done = std_msgs::Bool{};
  done.data = true;
  _done_pub.publish(done);
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