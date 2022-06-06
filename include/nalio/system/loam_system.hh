#ifndef NALIO_SYSTEM_LOAM_SYSTEM_HH__
#define NALIO_SYSTEM_LOAM_SYSTEM_HH__

#ifdef NALIO_DEBUG
#include <ros/ros.h>
#endif

#include <condition_variable>
#include <queue>
#include <shared_mutex>
#include <thread>

#include <Eigen/Geometry>

#include "nalio/data/message.hh"
#include "nalio/factory/factory.hh"
#include "nalio/feature/loam_feature_extractor.hh"
#include "nalio/propagator/linear_propagator.hh"
#include "nalio/system/system.hh"

#ifdef USE_UNOS
#include "unos/manifold/manifold.hh"
#endif

namespace nalio {
class LOAMSystem final : public System {
 public:
  using Ptr = std::unique_ptr<LOAMSystem>;
  using PointCloudT = pcl::PointCloud<NalioPoint>;
  using PointT = NalioPoint;

  LOAMSystem();
  ~LOAMSystem();

  void init() override;
  void stop() override;
  void feedData(const MessagePackage& data);

 private:
  struct EdgePair {
    PointT ori_pt;
    PointT neigh_pt[2];
  };

  struct PlanePair {
    PointT ori_pt;
    PointT neigh_pt[3];
  };
  void propagate() override;
  void update() override;
  void associate(const LOAMFeaturePackage::Ptr& prev_feature,
                 const LOAMFeaturePackage::Ptr& curr_feature,
                 std::vector<EdgePair>* edge_pairs,
                 std::vector<PlanePair>* plane_pairs);

  Eigen::Isometry3d getEstimated() override;

  LinearPropagator propagator_;
  LOAMFeatureExtractor<64> feature_extractor_;
#ifdef USE_UNOS
  unos::Manifold state_;
#else
  // x, y, z, qx, qy, qz, qw
  double last_to_curr_[7];
#endif

  bool initialized_, running_;
  std::queue<LOAMFeaturePackage::Ptr> feature_package_list_;
  std::condition_variable feature_package_list_cv_;
  std::mutex feature_package_list_mutex_;
  std::shared_mutex state_mutex_;
  std::thread update_thread_;
  Eigen::Isometry3d last_state_transform_;
  Eigen::Isometry3d curr_state_transform_;

  const double kNearbyScan = 2.5;

#ifdef NALIO_DEBUG
  ros::NodeHandle nh_;
  ros::Publisher sharp_feature_pub_;
  ros::Publisher less_sharp_feature_pub_;
  ros::Publisher flat_feature_pub_;
  ros::Publisher less_flat_feature_pub_;
  ros::Publisher associate_pub_;
#endif
};

REGISTER_NALIO(System, LOAMSystem, "LOAMSystem")
}  // namespace nalio

#endif
