#ifndef NALIO_DATA_DATASET_HH__
#define NALIO_DATA_DATASET_HH__

#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "nalio/data/data.hh"
#include "nalio/data/datahub.hh"
#include "nalio/data/message.hh"

namespace nalio {

class Dataset {
 public:
  using Ptr = std::shared_ptr<Dataset>;
  Dataset() {}
  virtual bool init(bool online) { return false; }
  void registerCallback(const NalioCallback& cb) {
    syncer_->registerCallback(cb);
  }

  virtual ~Dataset() {}

 protected:
  bool online_;
  DataBuffer buffer_;
  DataSyncer::Ptr syncer_;
};
}  // namespace nalio

#endif  // NALIO_DATA_DATASET_HH__
