#ifndef NALIO_DATA_DATASET_HH__
#define NALIO_DATA_DATASET_HH__

#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "datahub/datahub.hh"
#include "nalio/global.hh"

namespace nalio {

class Dataset {
 public:
  NALIO_MAKE_TYPE(Dataset);

  Dataset() {}
  virtual bool init(bool online) { return false; }
  void registerCallback(const datahub::DatahubCallback& cb) { syncer_->registerCallback(cb); }

  virtual ~Dataset() {}

 protected:
  bool online_;
  datahub::DataBuffer buffer_;
  datahub::DataSyncer::Ptr syncer_;
};
}  // namespace nalio

#endif  // NALIO_DATA_DATASET_HH__
