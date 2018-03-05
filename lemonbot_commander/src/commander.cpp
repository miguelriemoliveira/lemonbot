#include "lemonbot_commander/commander.hpp"

namespace lemonbot_acquisition
{
Commander::Commander(const InterpolationParams& interpolation_params, const InterfaceParams& interface_params)
  : _interpolation(interpolation_params), _interface(interface_params)
{
}

void Commander::startAcquisition()
{
  float step = (_interpolation.max - _interpolation.min) / _interpolation.nsteps;
  for (int i = 0; i < _interpolation.nsteps; ++i)
  {
    float angle = _interpolation.min + i * step;
    onEachStep(angle);
  }
}

void Commander::addCallback(std::function<void(float)> callback)
{
  _callbacks.push_back(callback);
}

void Commander::onEachStep(float angle)
{
  for (auto& callback : _callbacks)
  {
    ros::spinOnce();
    callback(angle);
  }
}
}
