#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "nalio/nailo.hh"

nalio::System::Ptr system_ptr;
nalio::Dataset::Ptr dataset_ptr;

void dataCallback(const datahub::MessagePackage& package) {
  ROS_INFO_STREAM_FUNC("enter pipeline data cb");
  system_ptr->feedData(package);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "nalio_pipeline");

  system_ptr = nalio::Factory<nalio::System>::produce_unique("LOAMSystem");
  system_ptr->init();

  dataset_ptr = nalio::Factory<nalio::Dataset>::produce_shared("KITTIDataset");
  dataset_ptr->init(true);
  dataset_ptr->registerCallback(dataCallback);

  ros::Rate rate(30);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
