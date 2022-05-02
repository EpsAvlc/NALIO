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
  // EXPECT_EQ(state.get(), expect_state);
}
