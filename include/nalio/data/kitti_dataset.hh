#ifndef NALIO_DATA_KITTI_DATASET_HH__
#define NALIO_DATA_KITTI_DATASET_HH__

#include "nalio/data/dataset.hh"
#include "nalio/factory/factory.hh"

namespace nalio {
class KITTIDataset : public Dataset {
 public:
  KITTIDataset() : imu_topic_("/imu"), lidar_topic_("/velodyne_points") {}
  void init() override;
};

REGISTER_NALIO(Dataset, KITTIDataset, "LOAMSystem")
}  // namespace nalio

#endif  // NALIO_DATA_KITTI_DATASET_HH__
