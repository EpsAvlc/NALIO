#ifndef NALIO_SYSTEM_LOAM_SYSTEM_HH__
#define NALIO_SYSTEM_LOAM_SYSTEM_HH__

#include "nalio/factory/factory.hh"
#include "nalio/feature/loam_feature_extractor.hh"
#include "nalio/system/system.hh"
#include "nalio/state/loam_state.hh"

namespace nalio {
class LOAMSystem final : public System {
 public:
  using Ptr = std::unique_ptr<LOAMSystem>;
  void init() override;
  void stop() override;
  void feedData(const DataPackage& data);
  

 private:
  void propagate() override;
  void update() override;
  Eigen::Isometry3d getEstimated() override;

  LOAMFeatureExtractor feature_extractor_;
  LOAMState state_;
};

REGISTER_NALIO(System, LOAMSystem, "LOAMSystem")
}  // namespace nalio

#endif
