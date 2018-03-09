#include <bitset>
#include <chrono>
#include <thread>

#include <ros/ros.h>

#include <lemonbot_acquisition/acquisition_node.h>

using namespace std::literals;
using namespace lemonbot;

const auto ptu_topic = "/SetPTUState";
const auto laser_in_topic = "/laserscan";
const auto laser_out_topic = "/registered";
const auto done_topic = "/done";

AcquisitionNode::Params get_params(ros::NodeHandle nh)
{
  auto params = AcquisitionNode::Params{};

  std::string type;

  // clang-format off
  auto got_all
     = nh.getParam("type", type)
    && nh.getParam("min", params.min)
    && nh.getParam("max", params.max)
    && nh.getParam("vel", params.vel)
    && nh.getParam("nsteps", params.nsteps);
  // clang-format on

  using Type = AcquisitionNode::Type;

  if (type == "continuous")
    params.type = Type::CONTINUOUS;
  else if (type == "point2point")
    params.type = Type::POINT2POINT;
  else if (type == "hybrid")
    params.type = Type::HYBRID;
  else
    throw(std::runtime_error{ "Acquisition Type not specified or not valid" });

  if (got_all)
    return params;
  else
    throw(std::runtime_error{ "no enough params specified" });
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "lemonbot_acquisition_node");

  auto nh = ros::NodeHandle{ "~" };

  auto params = get_params(nh);

  auto opts = AcquisitionNode::Options{
    .ptu_topic = ptu_topic,
    .max_vel = 30.0f,
    .laser_in_topic = laser_in_topic,
    .laser_out_topic = laser_out_topic,
    .done_topic = done_topic,
    .pause = 1000ms,
  };

  AcquisitionNode node(opts);

  node.start(params);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  return 0;
}
