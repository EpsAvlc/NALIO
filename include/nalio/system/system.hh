#ifndef NALIO_SYSTEM_SYSTEM_HH__
#define NALIO_SYSTEM_SYSTEM_HH__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

#include "datahub/message.hh"
#include "nalio/utils/transformation.hh"
#include "nalio/visualizer/visualizer.hh"
namespace nalio {

class System {
 public:
  using Ptr = std::unique_ptr<System>;
  virtual void init(){};
  virtual void feedData(const datahub::MessagePackage& data, VisInfo::Ptr const& ptr_deb_info = nullptr){};
  virtual void stop(){};

 protected:
  virtual void propagate(){};
  virtual void update(){};
  virtual TransformationD getEstimated() { return TransformationD(); };
};
}  // namespace nalio

#endif
