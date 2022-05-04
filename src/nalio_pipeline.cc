#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

#include "nalio/nailo.hh"


int main(int argc, char* argv[]) {
  ros::init(argc, argv, "nalio_pipeline");
  ros::NodeHandle nh;
  
  nalio::System::Ptr system_ptr = nalio::factory<nalio::System>::produce_unique("LOAMSystem");
  nalio::Dataset::Ptr dataset_ptr = nalio::factory<nalio::Dataset>::produce_shared("KITTIDataset");

  // system_ptr.reset()  

  ros::spin();
  return 0;
}
