#include <lemonbot_acquisition/acquisition_node.h>

using namespace lemonbot;

using namespace std::literals;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "lemonbot_acquisition_node");

  auto params = AcquisitionNode::Params{.min = -90, .max = 90, .vel = 5.0f };

  auto opts = AcquisitionNode::Options{
    .type = AcquisitionNode::Type::CONTINUOUS,
    .ptu_topic = "/SetPTUState",
    .max_vel = 30.0f,
    .laser_in_topic = "/laserscan",
    .laser_out_topic = "/capture_laser",
  };

  AcquisitionNode node(params, opts);

  node.start();

  return 0;
}
