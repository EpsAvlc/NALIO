#include "nalio/propagator/linear_propagator.hh"
#include "nalio/state/loam_state.hh"
#include "nalio/state/state.hh"

#include <gtest/gtest.h>

TEST(NALIO, LOAM_STATE) {
  nalio::LOAMState state;
  state.reset();
  nalio::LOAMState::InputT input;
  input << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
  state.oplus(input);

  nalio::LOAMState::StateT expect_state;
  expect_state << 0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 1;
  EXPECT_EQ(state.get(), expect_state);
}

TEST(NALIO, LINEAR_PROPAGATOR) {
  nalio::LinearPropagator propagator;
  Eigen::Isometry3d p0, p1, p2;
  p0.setIdentity();
  EXPECT_TRUE(propagator.propagate().isApprox(p0));
  propagator.update(p0);
  propagator.propagate();
  EXPECT_TRUE(propagator.propagate().isApprox(p0));

  p1.linear() =
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  p1.translation() = Eigen::Vector3d(10, 10, 10);
  propagator.update(p1);
  Eigen::Isometry3d est_prop1;
  est_prop1.linear() =
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  est_prop1.translation() = Eigen::Vector3d(20, 20, 20);
  EXPECT_TRUE(propagator.propagate().isApprox(est_prop1));
}
