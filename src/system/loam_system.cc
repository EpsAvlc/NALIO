#include "nalio/system/loam_system.hh"

#include "nalio/utils/log_utils.hh"

#ifdef NALIO_DEBUG
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/time.h>
#include "sensor_msgs/PointCloud2.h"
#endif

namespace nalio {

void LOAMSystem::init() {
#ifdef NALIO_DEBUG
  sharp_feature_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("NALIO/sharp_feature", 1);
  less_sharp_feature_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("NALIO/less_sharp_feature", 1);
  flat_feature_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("NALIO/flat_feature", 1);
  less_flat_feature_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("NALIO/less_flat_feature", 1);
#endif
}

void LOAMSystem::stop() {}

void LOAMSystem::feedData(const MessagePackage& msgs) {
  ROS_INFO_STREAM_FUNC("Feed a message pack. Frame id: " << msgs[0][0]->header.frame_id);
  propagate();
  LOAMFeaturePackage feature_package;
  PointCloudData::Ptr pc_data;
  std::string msg_frame_id;
  for (int mi = 0; mi < msgs.size(); ++mi) {
    if (msgs[mi][0]->type.val == Message::Type::kLidar) {
      pc_data = std::dynamic_pointer_cast<PointCloudData>(msgs[mi][0]->data);
      msg_frame_id = msgs[mi][0]->header.frame_id;
      break;
    }
  }
#ifdef NALIO_DEBUG
  pcl::console::TicToc tt;
  tt.tic();
#endif
  if (!feature_extractor_.extract(pc_data->point_cloud, &feature_package)) {
    ROS_ERROR_STREAM_FUNC("Failed to extract features.");
  }
#ifdef NALIO_DEBUG
  double fe_time = tt.toc();
  ROS_INFO_STREAM_FUNC("feature extractor elapse time: " << fe_time << "ms.");
#endif

#ifdef NALIO_DEBUG
  ROS_INFO_STREAM_FUNC("Feed data.");

  sensor_msgs::PointCloud2 flat_pc_msg, less_flat_pc_msg, sharp_pc_msg,
      less_sharp_pc_msg;
  pcl::toROSMsg(*feature_package.flat_cloud, flat_pc_msg);
  pcl::toROSMsg(*feature_package.less_flat_cloud, less_flat_pc_msg);
  pcl::toROSMsg(*feature_package.sharp_cloud, sharp_pc_msg);
  pcl::toROSMsg(*feature_package.less_sharp_cloud, less_sharp_pc_msg);

  flat_pc_msg.header.frame_id = msg_frame_id;
  less_flat_pc_msg.header.frame_id = msg_frame_id;
  sharp_pc_msg.header.frame_id = msg_frame_id;
  less_sharp_pc_msg.header.frame_id = msg_frame_id;

  flat_feature_pub_.publish(flat_pc_msg);
  less_flat_feature_pub_.publish(less_flat_pc_msg);
  sharp_feature_pub_.publish(sharp_pc_msg);
  less_sharp_feature_pub_.publish(less_sharp_pc_msg);
#endif

  update();
}

void LOAMSystem::propagate() {
  Eigen::Isometry3d propagated_pose = propagator_.propagate();
  state_.set(eigenToState(propagated_pose));
}

void LOAMSystem::update() {}

Eigen::Isometry3d LOAMSystem::getEstimated() {
  return stateToEigen(state_.get());
}

LOAMState::StateT LOAMSystem::eigenToState(const Eigen::Isometry3d& pose) {
  LOAMState::StateT ret;
  ret.head<3>() = pose.translation();
  Eigen::Quaterniond q(pose.linear());
  ret.tail<4>() = q.coeffs();
  return ret;
}

Eigen::Isometry3d LOAMSystem::stateToEigen(const LOAMState::StateT& state) {
  Eigen::Quaterniond q(&state_.get().data()[3]);
  Eigen::Vector3d t(state_.get().data());
  Eigen::Isometry3d ret;
  ret.linear() = q.toRotationMatrix();
  ret.translation() = t;
  return ret;
}

}  // namespace nalio
