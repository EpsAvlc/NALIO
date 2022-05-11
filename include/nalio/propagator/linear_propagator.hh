#ifndef NALIO_PROPAGATOR_LINEAR_PROPAGATOR_HH__
#define NALIO_PROPAGATOR_LINEAR_PROPAGATOR_HH__

#include <list>
#include "nalio/propagator/propagator.hh"

namespace nalio {
class LinearPropagator
    : public Propagator<Eigen::Isometry3d, Eigen::Isometry3d> {
 public:
  Eigen::Isometry3d propagate() override;
  bool update(const Eigen::Isometry3d& new_pose);

 private:
  std::list<Eigen::Isometry3d> pose_list_;
};
}  // namespace nalio

#endif
