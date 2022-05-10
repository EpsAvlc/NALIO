#ifndef NALIO_WS_PROPAGATOR_PROPAGATOR_HH__
#define NALIO_WS_PROPAGATOR_PROPAGATOR_HH__

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace nalio {
class Propagator {
 public:
  virtual Eigen::Isometry3d propagate() {};
};
}  // namespace nalio

#endif  // NALIO_WS_PROPAGATOR_PROPAGATOR_HH__
