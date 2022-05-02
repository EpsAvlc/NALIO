#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

#include "nalio/feature/feature_extractor.hh"
#include "nalio/feature/loam_feature_extractor.hh"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "nalio_pipeline");
  ros::NodeHandle nh;

  ros::spin();
  return 0;
}
