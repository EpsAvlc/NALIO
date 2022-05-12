#ifndef NALIO_SYSTEM_LOAM_SYSTEM_HH__
#define NALIO_SYSTEM_LOAM_SYSTEM_HH__

#ifdef NALIO_DEBUG
#include <ros/ros.h>
#endif

#include "nalio/system/system.hh"
#include "nalio/factory/factory.hh"
#include "nalio/feature/loam_feature_extractor.hh"
#include "nalio/state/loam_state.hh"
#include "nalio/propagator/linear_propagator.hh"

namespace nalio {
class LOAMSystem final : public System {
 public:
  using Ptr = std::unique_ptr<LOAMSystem>;
  using PointCloudT = DataPackage::PointCloudT;
  void init() override;
  void stop() override;
  void feedData(const DataPackage& data);
  

 private:
  void propagate() override;
  void update() override;

  Eigen::Isometry3d getEstimated() override;
  LOAMState::StateT eigenToState(const Eigen::Isometry3d& eigen);
  Eigen::Isometry3d stateToEigen(const LOAMState::StateT& state);

  LinearPropagator propagator_;
  LOAMFeatureExtractor feature_extractor_;
  LOAMState state_;

#ifdef NALIO_DEBUG
  ros::NodeHandle nh_;
  ros::Publisher sharp_feature_pub_;
  ros::Publisher less_sharp_feature_pub_;
  ros::Publisher flat_feature_pub_;
  ros::Publisher less_flat_feature_pub_;
#endif
};



REGISTER_NALIO(System, LOAMSystem, "LOAMSystem")
}  // namespace nalio

#endif
