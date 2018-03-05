#ifndef COMMANDER_HPP
#define COMMANDER_HPP

#include <functional>
#include <vector>

#include <ros/ros.h>

namespace lemonbot_acquisition
{
class Commander
{
public:
  struct InterpolationParams
  {
    float min;
    float max;
    int nsteps;
    float vel;
  };
  struct InterfaceParams
  {
  };
  Commander(const InterpolationParams&, const InterfaceParams&);

  void startAcquisition();

  void addCallback(std::function<void(float)>);

protected:
  void onEachStep(float angle);

private:
  InterpolationParams _interpolation;
  InterfaceParams _interface;
  std::vector<std::function<void(float)>> _callbacks;
};
}

#endif