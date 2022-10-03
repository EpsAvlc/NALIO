#ifndef NALIO_SYSTEM_LOAM_SYSTEM_HH__
#define NALIO_SYSTEM_LOAM_SYSTEM_HH__

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#ifdef NALIO_DEBUG
#endif

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
  void feedData(const datahub::MessagePackage& data) override;
  // only for debug
  void feedData(const LOAMFeaturePackage::Ptr& feature_package);

 private:
  void propagate() override;
  void update() override;
  void associate(const LOAMFeaturePackage::Ptr& prev_feature,
                 const LOAMFeaturePackage::Ptr& curr_feature,
                 std::vector<LOAMEdgePair>* edge_pairs,
                 std::vector<LOAMPlanePair>* plane_pairs);
  bool optimize(const std::vector<LOAMEdgePair>& edge_pair,
                const std::vector<LOAMPlanePair>& plane_pair);
  void transformPointToLastFrame(const NalioPoint& curr_p,
                            const Eigen::Quaterniond& curr2last_q,
                            const Eigen::Vector3d& curr2last_t, NalioPoint* last_p);

  Eigen::Isometry3d getEstimated() override;

  void processOdometry(const LOAMFeaturePackage::Ptr& feature);

  void processMapping(const LOAMFeaturePackage::Ptr& feature);

  LinearPropagator propagator_;
  LOAMFeatureExtractor<64> feature_extractor_;
#ifdef USE_UNOS
  unos::Manifold::Ptr state_;
#else
  /* for odometry */
  Eigen::Quaterniond q_odom_;
  Eigen::Vector3d t_odom_;
  Eigen::Quaterniond q_curr2last_;
  Eigen::Vector3d t_curr2last_;

  /* for mapping */
  Eigen::Quaterniond q_map_;
  Eigen::Vector3d t_map_;
  Eigen::Quaterniond q_odom2map_;
  Eigen::Vector3d t_odom2map_;
#endif

  bool initialized_, running_;
  std::queue<LOAMFeaturePackage::Ptr> feature_package_list_;
  std::condition_variable feature_package_list_cv_;
  std::mutex feature_package_list_mutex_;
  std::shared_mutex state_mutex_;
  std::thread update_thread_;
  Eigen::Isometry3d last_state_transform_;
  Eigen::Isometry3d curr_state_transform_;


#ifdef NALIO_DEBUG
  ros::Publisher sharp_feature_pub_;
  ros::Publisher less_sharp_feature_pub_;
  ros::Publisher flat_feature_pub_;
  ros::Publisher less_flat_feature_pub_;
  ros::Publisher associate_pub_;
  ros::Publisher sharp_curvature_pub_;
#endif
  ros::NodeHandle nh_;
  ros::Publisher path_pub_;
  nav_msgs::Path path_msg_;
};

REGISTER_NALIO(System, LOAMSystem, "LOAMSystem")
}  // namespace nalio

#endif
