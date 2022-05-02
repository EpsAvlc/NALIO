#ifndef NALIO_SYSTEM_SYSTEM_HH__
#define NALIO_SYSTEM_SYSTEM_HH__

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace nalio {
class system {
 public:
  virtual void init() = 0;
  virtual void stop() = 0;

 protected:
  virtual void propagate() = 0;
  virtual void update() = 0;
  virtual Eigen::Isometry3d getEstimated();


};
}  // namespace nalio

#endif
