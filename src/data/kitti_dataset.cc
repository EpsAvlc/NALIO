#include "nalio/data/kitti_dataset.hh"

#include <pcl_conversions/pcl_conversions.h>

#include "nalio/data/data_converter.hh"
#include "nalio/utils/log_utils.hh"

namespace nalio {
bool KITTIDataset::init(bool online) {
  online_ = online;
  if (online_) {
    if (!ros::isInitialized()) {
      NLOG_ERROR_STREAM("ROS has not been inited. Init failed.");
      return false;
    }
    lidar_sub_ = nh_.subscribe("/velodyne_points", 1,
                               &KITTIDataset::lidarCallback, this);
  }

  buffer_.registerMessage("/velodyne_points", 20);

  std::vector<std::string> topic_names{"/velodyne_points"};
  std::vector<datahub::DataSyncer::SyncType> sync_types{
      datahub::DataSyncer::SyncType::kNearest};

  syncer_ = buffer_.createDataSyncer(topic_names, sync_types);
  return true;
}

void KITTIDataset::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  datahub::Message::Ptr lidar_msg = nalio::toDatahubMessage(msg, "/velodyne_points");
  buffer_.receiveMessage(lidar_msg);
}

};  // namespace nalio
