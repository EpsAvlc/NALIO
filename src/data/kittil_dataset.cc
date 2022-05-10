#include "nalio/data/kitti_dataset.hh"

#include <pcl_conversions/pcl_conversions.h>

#include "nalio/utils/log_utils.hh"

namespace nalio {
bool KITTIDataset::init(bool online) {
  online_ = online;
  if (online_) {
    if (!ros::isInitialized()) {
      ROS_ERROR_STREAM_FUNC("ROS has not been inited. Init failed.");
      return false;
    }
    lidar_sub_ =
        nh_.subscribe("/velodyne_points", 1, KITTIDataset::lidarCallback, this);
  }
}

void KITTIDataset::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  lidar_msg_list_.push_back(msg);
}

bool KITTIDataset::getDataPackage(DataPackage* dp) {
  if ( 0 == lidar_msg_list_.size() || nullptr == dp) {
    return false;
  }
  pcl::PointCloud<pcl::PointXYZI> pc;
  pcl::fromROSMsg(*lidar_msg_list_.front(), pc);
  dp->lidar_meas.resize(pc.size());
  for (int i = 0; i < pc.size(); ++i) {
    dp->lidar_meas[i].x = pc[i].x;
    dp->lidar_meas[i].y = pc[i].y;
    dp->lidar_meas[i].z = pc[i].z;
    dp->lidar_meas[i].intensity = pc[i].intensity;
  }

  return true;
}

};  // namespace nalio
