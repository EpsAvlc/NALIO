#ifndef NALIO_SYSTEM_LOAM_SYSTEM_HH__
#define NALIO_SYSTEM_LOAM_SYSTEM_HH__

#include <nav_msgs/Path.h>
#include <cstddef>
#include <memory>
#ifdef NALIO_DEBUG
#endif

#include <atomic>
#include <condition_variable>
#include <queue>
#include <shared_mutex>
#include <thread>

#include <Eigen/Geometry>

#define PCL_NO_PRECOMPILE
#include "datahub/datahub.hh"
#include "nalio/ceres/loam_factors.hh"
#include "nalio/factory/factory.hh"
#include "nalio/feature/loam_feature_extractor.hh"
// #include "nalio/propagator/linear_propagator.hh"
#include "nalio/common/sliding_grid/auto_sliding_grid_map.hh"
#include "nalio/system/system.hh"
#include "nalio/utils/transformation.hh"

#ifdef USE_UNOS
#include "unos/manifold/manifold.hh"
#endif

namespace nalio {

class LOAMSystem final : public System {
 public:
  using Ptr = std::unique_ptr<LOAMSystem>;
  using PointCloudT = pcl::PointCloud<NalioPoint>;
  using PointT = NalioPoint;
  using SlidingGridMapT = AutoSlidingGridMap<3, std::array<pcl::PointCloud<PointT>::Ptr, 2>>;

  LOAMSystem();
  ~LOAMSystem();

  void init() override;
  void stop() override;
  void feedData(const datahub::MessagePackage& data, VisInfo::Ptr const& p_deb_info = nullptr) override;

 private:
  void initSlidingGridMap();

  void propagate() override;

  void update() override;

  void associateFromLast(const LOAMFeaturePackage::Ptr& prev_feature, const LOAMFeaturePackage::Ptr& curr_feature,
                         std::vector<LOAMEdgePair>* edge_pairs, std::vector<LOAMPlanePair>* plane_pairs);

  void associateFromMap(const LOAMFeaturePackage::Ptr& curr_feature, std::vector<LOAMEdgePair>* edge_pairs,
                        std::vector<LOAMPlanePair>* plane_pairs);

  bool odometryOptimize(const std::vector<LOAMEdgePair>& edge_pair, const std::vector<LOAMPlanePair>& plane_pair);

  void transformPointToLastFrame(const NalioPoint& curr_p, const Eigen::Quaterniond& curr2last_q,
                                 const Eigen::Vector3d& curr2last_t, NalioPoint* last_p);
  TransformationD getEstimated() override;

  void odometryUpdate(LOAMFeaturePackage::Ptr const& feature_package, VisInfo::Ptr const& p_deb_info = nullptr);

  void mappingUpdate(LOAMFeaturePackage::Ptr const& feature_package, VisInfo::Ptr const& p_deb_info = nullptr);

  // LinearPropagator propagator_;
  LOAMFeatureExtractor<64> feature_extractor_;
#ifdef USE_UNOS
  unos::Manifold::Ptr state_;
#else
  // qx, qy, qz, qw, x, y, z
  TransformationD body2odom_, body2map_;
  Eigen::Quaterniond curr2last_q_;
  Eigen::Vector3d curr2last_t_;
#endif

  std::atomic<bool> flg_initialized_, flg_running_;
  std::shared_mutex srd_mtx_state_;
  bool use_async_;

  nav_msgs::Path msg_odom_path_;

  typename SlidingGridMapT::Ptr ptr_sliding_grid_map_;
};

REGISTER_NALIO(System, LOAMSystem, "LOAMSystem");
}  // namespace nalio

#endif
