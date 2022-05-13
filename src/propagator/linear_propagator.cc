#include "nalio/propagator/linear_propagator.hh"

#include "nalio/utils/log_utils.hh"

namespace nalio {
Eigen::Isometry3d LinearPropagator::propagate() {
  if (pose_list_.size() == 0) {
    // throw(std::logic_error(std::string(__STR_FUNCTION__) +
    //                        "Can not propaget before update."));
    ROS_WARN("%s : Can not propagate before update.", __STR_FUNCTION__);
    return Eigen::Isometry3d::Identity();
  }
  if (pose_list_.size() == 1) {
    return pose_list_.front();
  }

  Eigen::Isometry3d& prev = pose_list_.front();
  Eigen::Isometry3d& curr = pose_list_.back();

  Eigen::Vector3d dt = curr.translation() - prev.translation();
  Eigen::Matrix3d dr = curr.rotation() * prev.rotation().transpose();
  Eigen::Isometry3d ret = pose_list_.back();
  ret.linear() = dr * curr.rotation();
  ret.translation() = curr.translation() + dt;
  return ret;
 }

bool LinearPropagator::update(const Eigen::Isometry3d& new_pose) {
  pose_list_.push_back(new_pose);
  if (pose_list_.size() > 2) {
    pose_list_.pop_front();
  }
  return true;
}

}  // namespace nalio
