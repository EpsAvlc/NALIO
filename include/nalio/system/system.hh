#ifndef NALIO_SYSTEM_SYSTEM_HH__
#define NALIO_SYSTEM_SYSTEM_HH__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

#include "nalio/data/data.hh"

namespace nalio {
class System {
 public:
  using Ptr = std::unique_ptr<System>;
  virtual void init(){};
  virtual void feedData(const DataPackage& data){};
  virtual void stop(){};

 protected:
  virtual void propagate(){};
  virtual void update(){};
  virtual Eigen::Isometry3d getEstimated(){ return Eigen::Isometry3d(); };
};
}  // namespace nalio

#endif
