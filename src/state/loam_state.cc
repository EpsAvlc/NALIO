#include "nalio/state/loam_state.hh"

#include <Eigen/Geometry>
namespace nalio {

void LOAMState::oplus(const InputT& input) {
  state_.head<3>() += input.head<3>();
  Eigen::Quaterniond q(state_.tail<4>());
  Eigen::Quaterniond in_q;
  in_q.w() = 1;
  in_q.x() = 0.5 * input(3);
  in_q.y() = 0.5 * input(4);
  in_q.z() = 0.5 * input(5);
  q = q * in_q;
  state_.tail<4>() = q.coeffs();
}

void LOAMState::reset() {
  state_.setZero();
  state_(4) = 0;
}

};
