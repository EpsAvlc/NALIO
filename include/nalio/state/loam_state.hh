#ifndef NALIO_STATE_LOAM_STATE_HH__
#define NALIO_STATE_LOAM_STATE_HH__

#include "nalio/state/state.hh"

namespace nalio {
class LOAMState final : public state<7, 6> {
 public:
  void oplus(const InputT& input) override;
  void reset() override;
};
}  // namespace nalio

#endif  // NALIO_STATE_LOAM_STATE_HH__
