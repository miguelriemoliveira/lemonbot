#include <lemonbot_pointcloud_pipeline/pointcloud_transformer.h>

using namespace lemonbot::pointcloud_pipeline;

PointcloudTransformer::PointcloudTransformer(std::string input, std::string output, std::string base_link)
  : _base_link(base_link)
  , Transformer(input, output, std::bind(&PointcloudTransformer::transform, this, std::placeholders::_1))
{
}

PointCloud PointcloudTransformer::transform(const PointCloud& pc)
{
  _listener.waitForTransform(pc.header.frame_id, _base_link, pc.header.stamp, ros::Duration(1.0));

  PointCloud out;
  pcl_ros::transformPointCloud(_base_link, pc, out, _listener);
  return out;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pointcloud_transformer");

  PointcloudTransformer pointcloud_transformer("input", "output", "lemonbot_base_link");

  ros::spin();
}
