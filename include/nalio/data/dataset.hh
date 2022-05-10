#ifndef NALIO_DATA_DATASET_HH__
#define NALIO_DATA_DATASET_HH__

#include <memory>
#include <string>

#include "nalio/data/data.hh"

namespace nalio {

class Dataset {
 public:
  using Ptr = std::shared_ptr<Dataset>;
  virtual bool init(bool online) { return false; }
  virtual bool getDataPackage(DataPackage* data){};

 protected:
  bool online_;
};
}  // namespace nalio

#endif  // NALIO_DATA_DATASET_HH__
