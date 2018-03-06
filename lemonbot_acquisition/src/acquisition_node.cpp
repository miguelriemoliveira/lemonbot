#include <lemonbot_acquisition/acquisition_node.h>

using namespace lemonbot;
using namespace std;

AcquisitionNode::AcquisitionNode(Params params, Options opts)
  : _params(params), _opts(opts), _ptu_client(opts.ptu_topic, true)
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
  auto goal = flir_pantilt_d46::PtuGotoGoal{};
  goal.pan = _params.min;
  goal.tilt = 0.0f;
  goal.pan_vel = _opts.max_vel;
  goal.tilt_vel = 10.0f;

  _ptu_client.sendGoalAndWait(goal);

  ROS_INFO("Reached minimum position");

  goal.pan_vel = _params.vel;

  if (_opts.type == Type::CONTINUOUS)
  {
    goal.pan = _params.max;
    _ptu_client.sendGoalAndWait(goal);
  }
  else
  {
    auto delta = (_params.max - _params.min) / (_params.nsteps - 1);
    for (int step = 0; step < _params.nsteps; step++)
    {
      goal.pan = _params.min + step * delta;
      _ptu_client.sendGoalAndWait(goal);
      std::this_thread::sleep_for(_opts.pause);
    }
  }

  ROS_INFO("Reached maximum position");
}
