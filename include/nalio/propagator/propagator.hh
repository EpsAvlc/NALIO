#ifndef NALIO_WS_PROPAGATOR_PROPAGATOR_HH__
#define NALIO_WS_PROPAGATOR_PROPAGATOR_HH__

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace nalio {
template <typename InputT, typename OutputT>
class Propagator {
 public:
  virtual OutputT propagate() { return OutputT(); };
  virtual bool update(const InputT& input) { return true;};
};
}  // namespace nalio

#endif  // NALIO_WS_PROPAGATOR_PROPAGATOR_HH__
