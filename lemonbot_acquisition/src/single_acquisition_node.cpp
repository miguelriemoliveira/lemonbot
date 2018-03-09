#include <chrono>
#include <thread>

#include <ros/ros.h>

#include <lemonbot_acquisition/acquisition_node.h>

using namespace std::literals;
using namespace lemonbot;

AcquisitionNode::Params get_params()
{
  auto nh = ros::NodeHandle{ "~" };

  auto params = AcquisitionNode::Params{};

  // clang-format off
  auto got_all
     = nh.getParam("min", params.min)
    && nh.getParam("max", params.max)
    && nh.getParam("vel", params.vel);
  // clang-format on

  if (got_all)
    return params;
  else
    throw(std::runtime_error{ "no enough params specified" });
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "lemonbot_acquisition_node");

  auto params = get_params();

  auto opts = AcquisitionNode::Options{
    .type = AcquisitionNode::Type::CONTINUOUS,
    .ptu_topic = "/SetPTUState",
    .max_vel = 30.0f,
    .laser_in_topic = "/laserscan",
    .laser_out_topic = "/registered",
    .done_topic = "/done",
  };

  AcquisitionNode node(opts);

  node.start(params);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  return 0;
}
