#include <lemonbot_acquisition/acquisition_node.h>
#include <lemonbot_acquisition/republishing.h>

#include <std_msgs/Float32.h>

using namespace lemonbot;

using namespace std::literals;

int main(int argc, char* argv[])
{
<<<<<<< HEAD
  ros::init(argc, argv, "lemonbot_acquisition");

  ros::NodeHandle nh;
=======
  ros::init(argc, argv, "lemonbot_acquisition_node");

  auto params = AcquisitionNode::Params{.min = -90, .max = 90, .vel = 5.0f };

  auto opts = AcquisitionNode::Options{
    .type = AcquisitionNode::Type::CONTINUOUS,
    .ptu_topic = "/SetPTUState",
    .max_vel = 30.0f,
    .laser_in_topic = "/laserscan",
    .laser_out_topic = "/capture_laser",
  };
>>>>>>> d46387ecc410540c324e1cd4c350711af3ef2701

  ros::Publisher pub = nh.advertise<std_msgs::Float32>("some_topic", 10);

  republish<std_msgs::Float32>("other_topic", pub, ros::Duration(1));

  return 0;
}
