#include "nalio/visualizer/loam_visualizer.hh"
#include <memory>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "nalio/utils/nalio_conversion.hh"
#include "nalio/visualizer/visualizer.hh"
#include "nav_msgs/Path.h"
#include "ros/init.h"
#include "ros/publisher.h"

namespace nalio {

void initPathMsg(nav_msgs::Path* path) { path->header.frame_id = "boot"; }

LOAMVisualizer::LOAMVisualizer() : nh_("LOAMVisualizer") {
  pub_map_path_ = nh_.advertise<nav_msgs::Path>("map_path", 1);
  pub_odom_path_ = nh_.advertise<nav_msgs::Path>("odom_path", 1);

  initPathMsg(&path_odom_);
  initPathMsg(&path_map_);
}

void LOAMVisualizer::feed(VisInfo::Ptr const& p_vis_info) {
  auto p_loam_vis_info = VIS_DOWN_CAST<LOAMVisInfo*>(p_vis_info.get());

  geometry_msgs::PoseStamped pose_odom = transform2GeometryPose(p_loam_vis_info->body2odom);
  path_odom_.poses.push_back(pose_odom);
  pub_odom_path_.publish(path_odom_);

  geometry_msgs::PoseStamped pose_map = transform2GeometryPose(p_loam_vis_info->body2map);
  path_map_.poses.push_back(pose_map);
  pub_map_path_.publish(path_map_);

  // static int cnt = 0;
  // static TransformationD last_robot2odom;
  // pcl::console::TicToc tt;
  // tt.tic();
  // NLOG_INFO("wait msg elapse for %lf ms.", tt.toc());
  // ++cnt;
  // if (cnt < 5) {
  //   return;
  // }
  // cnt = 0;

  // TransformationD robot2map_est = robot2map_ * last_robot2odom.inverse() * robot2odom_;
  // NLOG_INFO_STREAM("robot2odom: " << std::endl
  //                                 << robot2odom_.matrix() << std::endl
  //                                 << "robot2map: " << std::endl
  //                                 << robot2map_est.matrix());
  // NLOG_INFO_STREAM("odom process one frame.");
  // last_robot2odom = robot2odom_;

  //   #ifdef NALIO_DEBUG
  //   NLOG_INFO_STREAM("Feed data.");

  //   // display sharp curvatures.
  //   size_t sharp_size = feature_package->sharp_cloud->size();
  //   std::vector<std::string> texts(sharp_size);
  //   std::vector<Eigen::Vector3d> positions(sharp_size);
  //   for (size_t si = 0; si < feature_package->sharp_curvatures.size(); ++si) {
  //     texts[si] =
  //         std::to_string(feature_package->sharp_inds[si]) + ", " +
  //         std::to_string(feature_package->sharp_curvatures[si]);
  //     positions[si] = feature_package->sharp_cloud->at(si).getVector3fMap().cast<double>();
  //   }
  //   visualization_msgs::MarkerArray sharp_curvature_msg = rviz_utils::putTexts(texts, positions, "velodyne");
  //   pub_sharp_curvature_.publish(sharp_curvature_msg);
  // #endif

  // #ifdef NALIO_DEBUG
  //   NLOG_INFO_STREAM("Feed data.");

  //   // display sharp curvatures.
  //   size_t sharp_size = feature_package->sharp_cloud->size();
  //   std::vector<std::string> texts(sharp_size);
  //   std::vector<Eigen::Vector3d> positions(sharp_size);
  //   for (size_t si = 0; si < feature_package->sharp_curvatures.size(); ++si) {
  //     texts[si] =
  //         std::to_string(feature_package->sharp_inds[si]) + ", " +
  //         std::to_string(feature_package->sharp_curvatures[si]);
  //     positions[si] = feature_package->sharp_cloud->at(si).getVector3fMap().cast<double>();
  //   }
  //   visualization_msgs::MarkerArray sharp_curvature_msg = rviz_utils::putTexts(texts, positions, "velodyne");
  //   pub_sharp_curvature_.publish(sharp_curvature_msg);
  // #endif
}

void LOAMVisualizer::spin() { ros::spin(); }

VisInfo::Ptr LOAMVisualizer::createVisInfo() { return std::make_unique<LOAMVisInfo>(); }
}  // namespace nalio
