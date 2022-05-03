#ifndef NALIO_SYSTEM_SYSTEM_HH__
#define NALIO_SYSTEM_SYSTEM_HH__

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace nalio {
class System {
 public:
  using Ptr = std::unique_ptr<System>;
  virtual void init() {};
  virtual void stop() {};

 protected:
  virtual void propagate() {};
  virtual void update() {};
  virtual Eigen::Isometry3d getEstimated();


};
}  // namespace nalio

#endif
