#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

#include "nalio/nailo.hh"

nalio::System::Ptr system_ptr;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "nalio_pipeline");
  ros::NodeHandle nh;
  
  system_ptr = nalio::factory<nalio::System>::produce_unique("LOAMSystem");
  // system_ptr.reset()  

  ros::spin();
  return 0;
}
