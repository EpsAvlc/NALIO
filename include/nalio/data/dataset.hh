#ifndef NALIO_DATA_DATASET_HH__
#define NALIO_DATA_DATASET_HH__

#include <string>
#include <memory>

#include "nalio/data/data.hh"

namespace nalio {

class Dataset {
public:
  using Ptr = std::shared_ptr<Dataset>;
  virtual void init() {};
  virtual std::string getIMUTopic() { return imu_topic_; };
  virtual std::string getLiDARTopic() { return lidar_topic_; };
  virtual int offlineReadData() {};

protected:
  std::string imu_topic_;
  std::string lidar_topic_;
};
}

#endif // NALIO_DATA_DATASET_HH__
