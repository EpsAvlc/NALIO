#include "nalio/system/loam_system.hh"

#include "nalio/utils/log_utils.hh"

#ifdef NALIO_DEBUG
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
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

void LOAMSystem::stop() {
  
}

void LOAMSystem::feedData(const MessagePackage& msgs) {
  propagate();
  std::vector<LOAMFeature> features;
  // if (!feature_extractor_.extract(data.lidar_meas, &features)) {
  //   ROS_ERROR_STREAM_FUNC("Failed to extract features.");
  // }

#ifdef NALIO_DEBUG
  PointCloudT flat_pc, less_flat_pc, sharp_pc, less_sharp_pc;
  for (int fi = 0; fi < features.size(); ++fi) {
    switch (features[fi].type.val) {
      case LOAMFeature::Type::kSharp:
        sharp_pc.push_back(features[fi].pt);
        break;
      case LOAMFeature::Type::kLessSharp:
        less_sharp_pc.push_back(features[fi].pt);
        break;
      case LOAMFeature::Type::kFlat:
        flat_pc.push_back(features[fi].pt);
        break;
      case LOAMFeature::Type::kLessFlat:
        less_flat_pc.push_back(features[fi].pt);
        break;
      default:
        break;
    }
  }

  sensor_msgs::PointCloud2 flat_pc_msg, less_flat_pc_msg, sharp_pc_msg, less_sharp_pc_msg;
  pcl::toROSMsg(flat_pc, flat_pc_msg);
  pcl::toROSMsg(less_flat_pc, less_flat_pc_msg);
  pcl::toROSMsg(sharp_pc, sharp_pc_msg);
  pcl::toROSMsg(less_sharp_pc, less_sharp_pc_msg);

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

void LOAMSystem::update() {
  
}

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
