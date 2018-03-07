#ifndef POINTCLOUD_ACCUMULATOR_H
#define POINTCLOUD_ACCUMULATOR_H

#include <functional>
#include <string>

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>

#include <lemonbot_acquisition/transformer.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PointcloudAccumulator : public Transformer<PointCloud, PointCloud>
{
public:
  PointcloudAccumulator(std::string input, std::string output);

protected:
  PointCloud accumulate(const PointCloud& pc);

private:
  PointCloud _accumulated;
};

#endif