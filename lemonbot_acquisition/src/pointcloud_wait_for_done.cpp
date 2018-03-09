#include <lemonbot_acquisition/wait_for_done.h>

#include <sensor_msgs/PointCloud2.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pointcloud_wait_for_done");

  WaitForDone<sensor_msgs::PointCloud2> wfm("input", "output", "done");

  ros::spin();

  return 0;
}
