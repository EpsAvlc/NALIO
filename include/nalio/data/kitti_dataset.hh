#ifndef NALIO_DATA_KITTI_DATASET_HH__
#define NALIO_DATA_KITTI_DATASET_HH__

#include <list>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "nalio/data/datahub.hh"
#include "nalio/data/dataset.hh"
#include "nalio/data/message.hh"
#include "nalio/factory/factory.hh"
#include "nalio/param/param_server.hh"

namespace nalio {
class KITTIDataset : public Dataset {
 public:
  KITTIDataset() {}
  bool init(bool online) override;
  ~KITTIDataset() override {}

 private:
  void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber lidar_sub_;
  std::list<sensor_msgs::PointCloud2ConstPtr> lidar_msg_list_;
};

REGISTER_NALIO(Dataset, KITTIDataset, "KITTIDataset")
}  // namespace nalio

#endif  // NALIO_DATA_KITTI_DATASET_HH__
