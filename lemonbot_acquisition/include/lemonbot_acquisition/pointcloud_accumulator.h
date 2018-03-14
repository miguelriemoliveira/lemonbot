#ifndef POINTCLOUD_ACCUMULATOR_H
#define POINTCLOUD_ACCUMULATOR_H

#include <functional>
#include <string>

#include <ros/ros.h>

#include <std_msgs/Bool.h>

#include <pcl_ros/point_cloud.h>

#include <lemonbot_acquisition/transformer.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PointcloudAccumulator : public Transformer<PointCloud, PointCloud>
{
public:
  PointcloudAccumulator(std::string input, std::string output, std::string done_topic, std::string result_topic);

protected:
  PointCloud accumulate(const PointCloud& pc);

  void done(const std_msgs::Bool::ConstPtr&);

private:
  ros::NodeHandle _nh;

  ros::Subscriber _done_sub;
  ros::Publisher _result_pub;

  PointCloud _accumulated;
};

#endif