#include <lemonbot_acquisition/acquisition_node.h>
#include <lemonbot_acquisition/republishing.h>

#include <std_msgs/Float32.h>

using namespace lemonbot;

using namespace std::literals;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "lemonbot_acquisition");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Float32>("some_topic", 10);

  republish<std_msgs::Float32>("other_topic", pub, ros::Duration(1));

  return 0;
}
