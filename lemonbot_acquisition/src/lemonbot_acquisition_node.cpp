#include <lemonbot_acquisition/acquisition_node.h>

using namespace lemonbot;

using namespace std::literals;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "lemonbot_acquisition_node");

  auto params = AcquisitionNode::Params{.min = -90, .max = 0, .nsteps = 100, .vel = 10.0f };

  auto opts = AcquisitionNode::Options{.type = AcquisitionNode::Type::POINT2POINT,
                                       .timeout = ros::Duration(5),
                                       .ptu_topic = "/SetPTUState",
                                       .pause = 300ms,
                                       .max_vel = 40.0f };

  AcquisitionNode node(params, opts);

  node.start();

  return 0;
}
