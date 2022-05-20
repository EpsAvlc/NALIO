#include "nalio/data/kitti_dataset.hh"

#include <pcl_conversions/pcl_conversions.h>

#include "nalio/data/data_converter.hh"
#include "nalio/utils/log_utils.hh"

namespace nalio {
bool KITTIDataset::init(bool online) {
  online_ = online;
  if (online_) {
    if (!ros::isInitialized()) {
      ROS_ERROR_STREAM_FUNC("ROS has not been inited. Init failed.");
      return false;
    }
    lidar_sub_ = nh_.subscribe("/velodyne_points", 1,
                               &KITTIDataset::lidarCallback, this);
  }

  buffer_.registerMessage("/velodyne_points", 20);
  return true;
}

void KITTIDataset::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  Message::Ptr lidar_msg = toNalioMessage(msg, "/velodyne_points");
  buffer_.receiveMessage(lidar_msg);
}

};  // namespace nalio
