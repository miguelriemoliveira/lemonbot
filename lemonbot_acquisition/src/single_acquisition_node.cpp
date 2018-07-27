#include <bitset>
#include <chrono>
#include <thread>

#include <ros/ros.h>

#include <lemonbot_acquisition/acquisition_node.h>

using namespace std::literals;
using namespace lemonbot;
using namespace lemonbot::acquisition;

const auto ptu_topic = "/SetPTUState";
const auto laser_in_topic = "laserscan";
const auto laser_out_topic = "acquisition/laserscan";
const auto done_topic = "/done";

AcquisitionNode::Params get_params(ros::NodeHandle nh)
{
  auto params = AcquisitionNode::Params{};

  std::string type;

  // clang-format off
  auto got_all
     = nh.getParam("type", type)
    && nh.getParam("pan_min", params.pan.min)
    && nh.getParam("pan_max", params.pan.max)
    && nh.getParam("pan_vel", params.pan.vel)
    && nh.getParam("pan_nsteps", params.pan.nsteps)
    && nh.getParam("tilt_min", params.tilt.min)
    && nh.getParam("tilt_max", params.tilt.max)
    && nh.getParam("tilt_vel", params.tilt.vel)
    && nh.getParam("tilt_nsteps", params.tilt.nsteps);
  // clang-format on

  using Type = AcquisitionNode::Type;

  if (type == "continuous")
    params.type = Type::CONTINUOUS;
  else if (type == "point2point")
    params.type = Type::POINT2POINT;
  else
    throw(std::runtime_error{ "Acquisition Type not specified or not valid" });

  if (got_all)
    return params;
  else
    throw(std::runtime_error{ "no enough params specified" });
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "lemonbot_acquisition_node");

  auto nh = ros::NodeHandle{ "~" };

  auto params = get_params(nh);

  auto opts = AcquisitionNode::Options{};
  opts.pan_limits = AcquisitionNode::PanTiltInterpolation{};
  opts.tilt_limits = AcquisitionNode::PanTiltInterpolation{};
  opts.timeout = ros::Duration{ 5 };
  opts.ptu_topic = ptu_topic;
  opts.laser_out_topic = laser_out_topic;
  opts.laser_in_topic = laser_in_topic;
  opts.camera_out_topic = "acquisition/images";
  opts.camera_in_topic = "/camera/image_color";
  opts.done_topic = done_topic;
  opts.pause = 2000ms;

  AcquisitionNode node(opts);

  node.start(params);

  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  return 0;
}
