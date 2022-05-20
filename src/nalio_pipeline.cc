#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

#include "nalio/nailo.hh"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "nalio_pipeline");
  // ros::NodeHandle nh;
  
  nalio::System::Ptr system_ptr = nalio::Factory<nalio::System>::produce_unique("LOAMSystem");
  system_ptr->init();

  nalio::Dataset::Ptr dataset_ptr = nalio::Factory<nalio::Dataset>::produce_shared("KITTIDataset");
  // dataset_ptr->init(false);
  
  ros::Rate rate(30);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
    // if (!dataset_ptr->getDataPackage(&data_package)) {
    //   continue;
    // }

    // system_ptr->feedData(data_package);
  }

  return 0;
}
