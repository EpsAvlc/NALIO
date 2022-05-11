#include "nalio/system/loam_system.hh"

namespace nalio {
void LOAMSystem::feedData(const DataPackage& data) {
  propagate();
  
  update();
}

void LOAMSystem::propagate() {
  Eigen::Isometry3d propagated_pose = propagator_.propagate();
  state_.set(eigenToState(propagated_pose));
}

LOAMState::StateT LOAMSystem::eigenToState(const Eigen::Isometry3d& pose) {
  LOAMState::StateT ret;
  ret.head<3>() = pose.translation();
  Eigen::Quaterniond q(pose.linear());
  ret.tail<4>() = q.coeffs();
  return ret;
}

Eigen::Isometry3d LOAMSystem::stateToEigen(const LOAMState::StateT& state) {
  Eigen::Quaterniond q(&state_.get().data()[3]);
  Eigen::Vector3d t(state_.get().data());
  Eigen::Isometry3d ret;
  ret.linear() = q.toRotationMatrix();
  ret.translation() = t;
  return ret;
}
}  // namespace nalio
