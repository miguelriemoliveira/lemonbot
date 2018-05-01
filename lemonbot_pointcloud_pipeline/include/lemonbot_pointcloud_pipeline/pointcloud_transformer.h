#ifndef POINTCLOUD_TRANSFORMER_H
#define POINTCLOUD_TRANSFORMER_H

#include <functional>
#include <string>

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>

#include <lemonbot_utils/transformer.h>

#include <tf/transform_listener.h>

namespace lemonbot::pointcloud_pipeline {

using namespace lemonbot::utils;

typedef sensor_msgs::PointCloud2 PointCloud;

class PointcloudTransformer : Transformer<PointCloud, PointCloud>
{
public:
  PointcloudTransformer(std::string input, std::string output, std::string base_link);

protected:
  PointCloud transform(const PointCloud& pc);

private:
  std::string _base_link;
  tf::TransformListener _listener;
};

}

#endif