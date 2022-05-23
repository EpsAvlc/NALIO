#include "nalio/system/loam_system.hh"

#include "nalio/utils/log_utils.hh"

#ifdef NALIO_DEBUG
#include <pcl/console/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#endif

namespace nalio {

LOAMSystem::LOAMSystem()
    : state_(unos::SO3(), unos::Vec3()), initialized_(false), running_(true) {}

LOAMSystem::~LOAMSystem() {
  running_ = false;
  if (update_thread_.joinable()) {
    feature_package_list_cv_.notify_all();
    update_thread_.join();
  }
}

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
  update_thread_ = std::thread(&LOAMSystem::update, this);
  initialized_ = true;
}

void LOAMSystem::stop() {}

void LOAMSystem::feedData(const MessagePackage& msgs) {
  if (!initialized_) {
    ROS_ERROR_STREAM_FUNC(" System has not been initialized.");
    return;
  }
  ROS_INFO_STREAM_FUNC(
      "Feed a message pack. Frame id: " << msgs[0][0]->header.frame_id);
  propagate();
  LOAMFeaturePackage::Ptr feature_package(new LOAMFeaturePackage);
  PointCloudData::Ptr pc_data;
  std::string msg_frame_id;
  for (int mi = 0; mi < msgs.size(); ++mi) {
    if (msgs[mi][0]->type.val == Message::Type::kLidar) {
      pc_data = std::dynamic_pointer_cast<PointCloudData>(msgs[mi][0]->data);
      msg_frame_id = msgs[mi][0]->header.frame_id;
      break;
    }
  }

  pcl::console::TicToc tt;
  tt.tic();
  if (!feature_extractor_.extract(pc_data->point_cloud, feature_package)) {
    ROS_ERROR_STREAM_FUNC("Failed to extract features.");
  }
  double fe_time = tt.toc();
  ROS_INFO_STREAM_FUNC("feature extractor elapse time: " << fe_time << "ms.");

  feature_package_list_mutex_.lock();
  feature_package_list_.push(feature_package);
  feature_package_list_mutex_.unlock();
  feature_package_list_cv_.notify_all();

#ifdef NALIO_DEBUG
  ROS_INFO_STREAM_FUNC("Feed data.");

  sensor_msgs::PointCloud2 flat_pc_msg, less_flat_pc_msg, sharp_pc_msg,
      less_sharp_pc_msg;
  pcl::toROSMsg(*feature_package->flat_cloud, flat_pc_msg);
  pcl::toROSMsg(*feature_package->less_flat_cloud, less_flat_pc_msg);
  pcl::toROSMsg(*feature_package->sharp_cloud, sharp_pc_msg);
  pcl::toROSMsg(*feature_package->less_sharp_cloud, less_sharp_pc_msg);

  flat_pc_msg.header.frame_id = msg_frame_id;
  less_flat_pc_msg.header.frame_id = msg_frame_id;
  sharp_pc_msg.header.frame_id = msg_frame_id;
  less_sharp_pc_msg.header.frame_id = msg_frame_id;

  flat_feature_pub_.publish(flat_pc_msg);
  less_flat_feature_pub_.publish(less_flat_pc_msg);
  sharp_feature_pub_.publish(sharp_pc_msg);
  less_sharp_feature_pub_.publish(less_sharp_pc_msg);
#endif
}

void LOAMSystem::propagate() {
  // Eigen::Isometry3d propagated_pose = propagator_.propagate();
  // state_.set(eigenToState(propagated_pose));
}

void LOAMSystem::update() {
  while (running_) {
    std::unique_lock<std::mutex> lock(feature_package_list_mutex_);
    while (feature_package_list_.empty() && running_) {
      feature_package_list_cv_.wait(lock);
    }
    ROS_INFO_STREAM_FUNC("enter update.");
    feature_package_list_.pop();
  }
}

Eigen::Isometry3d LOAMSystem::getEstimated() {
  // return stateToEigen(state_.get());
}

}  // namespace nalio
