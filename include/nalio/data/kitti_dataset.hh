#ifndef NALIO_DATA_KITTI_DATASET_HH__
#define NALIO_DATA_KITTI_DATASET_HH__

#include <list>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "nalio/data/dataset.hh"
#include "nalio/factory/factory.hh"


namespace nalio {
class KITTIDataset : public Dataset {
 public:
  KITTIDataset() {}
  bool init(bool online) override;
  bool getDataPackage(DataPackage* data) override;
  ~KITTIDataset() override {}
 private:
  void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  mutable std::mutex lidar_msg_list_mutex_;

  ros::NodeHandle nh_;
  ros::Subscriber lidar_sub_;
  std::list<sensor_msgs::PointCloud2ConstPtr> lidar_msg_list_;
};

REGISTER_NALIO(Dataset, KITTIDataset, "LOAMSystem")
}  // namespace nalio

#endif  // NALIO_DATA_KITTI_DATASET_HH__
