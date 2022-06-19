#include "nalio/system/loam_system.hh"

#include <pcl/kdtree/kdtree_flann.h>
#include "nalio/utils/log_utils.hh"

#ifdef NALIO_DEBUG
#include <pcl/console/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#endif

#include "nalio/data/nalio_data.hh"

namespace nalio {

LOAMSystem::LOAMSystem()
    :
#ifdef USE_UNOS
      state_(unos::SO3(), unos::Vec3()),
#endif
      initialized_(false),
      running_(true) {
}

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
  associate_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("NALIO/associate", 1);
#endif

#ifdef USE_UNOS
  throw(std::logic_error("not implement yet."));
#else
  for (size_t i = 0; i < 7; ++i) {
    last_to_curr_[i] = 0;
  }
  last_to_curr_[6] = 1;
#endif

  update_thread_ = std::thread(&LOAMSystem::update, this);
  initialized_ = true;
}

void LOAMSystem::stop() {}

void LOAMSystem::feedData(const datahub::MessagePackage& msgs) {
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
    if (msgs[mi][0]->type.val == datahub::Message::Type::kLidar) {
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
  ROS_INFO_STREAM_FUNC(
      "sharp feature size: " << feature_package->sharp_cloud->size());
  ROS_INFO_STREAM_FUNC(
      "flat feature size: " << feature_package->flat_cloud->size());

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
  static LOAMFeaturePackage::Ptr prev_feature_package;
  while (running_) {
    std::unique_lock<std::mutex> lock(feature_package_list_mutex_);
    while (feature_package_list_.empty() && running_) {
      feature_package_list_cv_.wait(lock);
    }
    ROS_INFO_STREAM_FUNC("enter update.");
    auto& curr_feature_package = feature_package_list_.front();
    if (prev_feature_package) {
      std::vector<EdgePair> edge_pairs;
      std::vector<PlanePair> plane_pairs;
      associate(prev_feature_package, curr_feature_package, &edge_pairs,
                &plane_pairs);
    }
    prev_feature_package = curr_feature_package;
    feature_package_list_.pop();
  }
}

void LOAMSystem::associate(const LOAMFeaturePackage::Ptr& prev_feature,
                           const LOAMFeaturePackage::Ptr& curr_feature,
                           std::vector<EdgePair>* edge_pairs,
                           std::vector<PlanePair>* plane_pairs) {
  const double kMaxAssociateDistanceSq = 25;
  if (!edge_pairs || !plane_pairs) {
    throw(std::invalid_argument("edge_pairs or plane_pairs is nullptr!"));
  }
  edge_pairs->clear();
  plane_pairs->clear();
  //* Edge feature association.
  static pcl::KdTreeFLANN<PointT> edge_kdtree;
  edge_kdtree.setInputCloud(prev_feature->less_sharp_cloud);
  std::vector<int> pt_search_inds;
  std::vector<float> pt_search_dists;
  for (size_t ei = 0; ei < curr_feature->sharp_cloud->size(); ++ei) {
    const PointT& pt_sel = curr_feature->sharp_cloud->at(ei);

    edge_kdtree.nearestKSearch(pt_sel, 1, pt_search_inds, pt_search_dists);
    if (pt_search_dists[0] > kMaxAssociateDistanceSq) {
      continue;
    }
    const PointT& closest_pt =
        (*prev_feature->less_sharp_cloud)[pt_search_inds[0]];
    int second_closest_pt_ind = -1;
    double second_closest_pt_dist = kMaxAssociateDistanceSq;
    for (size_t ej = pt_search_inds[0] + 1;
         ej < prev_feature->less_sharp_cloud->size(); ++ej) {
      if ((*prev_feature->less_sharp_cloud)[ej].line <= closest_pt.line) {
        continue;
      }

      if ((*prev_feature->less_sharp_cloud)[ej].line >
          closest_pt.line + kNearbyScan) {
        break;
      }

      double distance =
          ((*prev_feature->less_sharp_cloud)[ej].getVector3fMap() -
           pt_sel.getVector3fMap())
              .squaredNorm();
      if (distance < second_closest_pt_dist) {
        second_closest_pt_ind = ej;
        second_closest_pt_dist = distance;
      }
    }

    for (int32_t ej = pt_search_inds[0] - 1; ej > 0; --ej) {
      if ((*prev_feature->less_sharp_cloud)[ej].line >= closest_pt.line) {
        continue;
      }

      if ((*prev_feature->less_sharp_cloud)[ej].line <
          closest_pt.line - kNearbyScan) {
        break;
      }
      double distance =
          ((*prev_feature->less_sharp_cloud)[ej].getVector3fMap() -
           pt_sel.getVector3fMap())
              .squaredNorm();
      if (distance < second_closest_pt_dist) {
        second_closest_pt_ind = ej;
        second_closest_pt_dist = distance;
      }
    }

    if (second_closest_pt_ind > 0) {
      EdgePair edge_pair;
      edge_pair.ori_pt = pt_sel;
      edge_pair.neigh_pt[0] = closest_pt;
      edge_pair.neigh_pt[1] =
          (*prev_feature->less_sharp_cloud)[second_closest_pt_ind];
      edge_pairs->push_back(std::move(edge_pair));
    }
  }

  static pcl::KdTreeFLANN<PointT> plane_kdtree;
  plane_kdtree.setInputCloud(prev_feature->less_flat_cloud);
  ROS_INFO_STREAM_FUNC("Current flat cloud size: " << curr_feature->flat_cloud->size());
  for (size_t pi = 0; pi < curr_feature->flat_cloud->size(); ++pi) {
    const PointT& pt_sel = curr_feature->flat_cloud->at(pi);
    plane_kdtree.nearestKSearch(pt_sel, 1, pt_search_inds, pt_search_dists);
    if (pt_search_dists[0] > kMaxAssociateDistanceSq) {
      continue;
    }

    const PointT& closest_pt =
        (*prev_feature->less_flat_cloud)[pt_search_inds[0]];
    int l_ind = -1, m_ind = -1;
    double l_distance = kMaxAssociateDistanceSq;
    double m_distance = kMaxAssociateDistanceSq;

    for (size_t pj = pt_search_inds[0] + 1;
         pj < prev_feature->less_flat_cloud->size(); ++pj) {
      if ((*prev_feature->less_flat_cloud)[pj].line <= closest_pt.line) {
        continue;
      }
      if ((*prev_feature->less_flat_cloud)[pj].line >
          closest_pt.line + kNearbyScan) {
        break;
      }
      double distance = ((*prev_feature->less_flat_cloud)[pj].getVector3fMap() -
                         closest_pt.getVector3fMap())
                            .squaredNorm();
      if (distance < l_distance) {
        l_ind = pj;
        l_distance = distance;
      }
    }

    for (int32_t pj = pt_search_inds[0] - 1; pj >= 0; --pj) {
      if ((*prev_feature->less_flat_cloud)[pj].line >= closest_pt.line) {
        continue;
      }
      if ((*prev_feature->less_flat_cloud)[pj].line <
          closest_pt.line - kNearbyScan) {
        break;
      }
      double distance = ((*prev_feature->less_flat_cloud)[pj].getVector3fMap() -
                         closest_pt.getVector3fMap())
                            .squaredNorm();
      if (distance < m_distance) {
        m_ind = pj;
        m_distance = distance;
      }
    }

    if (l_ind >= 0 && m_ind >= 0) {
      PlanePair plane_pair;
      plane_pair.ori_pt = pt_sel;
      plane_pair.neigh_pt[0] = closest_pt;
      plane_pair.neigh_pt[1] = (*prev_feature->less_flat_cloud)[l_ind];
      plane_pair.neigh_pt[2] = (*prev_feature->less_flat_cloud)[m_ind];
      plane_pairs->push_back(std::move(plane_pair));
    }
  }

#ifdef NALIO_DEBUG
  visualization_msgs::MarkerArray associate_marker_array;
  visualization_msgs::Marker edge_line_list, plane_line_list;
  edge_line_list.header.frame_id = "camera_init";
  edge_line_list.header.stamp = ros::Time::now();
  edge_line_list.action = visualization_msgs::Marker::ADD;
  edge_line_list.id = 1;
  edge_line_list.type = visualization_msgs::Marker::LINE_LIST;
  edge_line_list.ns = "edge_lines";
  edge_line_list.scale.x = 0.15;
  edge_line_list.scale.y = 0.15;
  edge_line_list.scale.z = 0.15;
  edge_line_list.color.r = 1;
  edge_line_list.color.g = 0;
  edge_line_list.color.b = 0;
  edge_line_list.color.a = 1;
  edge_line_list.pose.orientation.w = 1;

  for (size_t ei = 0; ei < edge_pairs->size(); ++ei) {
    geometry_msgs::Point ori_pt;
    ori_pt.x = (*edge_pairs)[ei].ori_pt.x;
    ori_pt.y = (*edge_pairs)[ei].ori_pt.y;
    ori_pt.z = (*edge_pairs)[ei].ori_pt.z;

    for (size_t ni = 0; ni < 2; ++ni) {
      geometry_msgs::Point neigh_pt;
      neigh_pt.x = (*edge_pairs)[ei].neigh_pt[ni].x;
      neigh_pt.y = (*edge_pairs)[ei].neigh_pt[ni].y;
      neigh_pt.z = (*edge_pairs)[ei].neigh_pt[ni].z;

      edge_line_list.points.push_back(ori_pt);
      edge_line_list.points.push_back(std::move(neigh_pt));
    }
  }
  associate_marker_array.markers.push_back(edge_line_list);

  plane_line_list.header.frame_id = "camera_init";
  plane_line_list.header.stamp = ros::Time::now();
  plane_line_list.action = visualization_msgs::Marker::ADD;
  plane_line_list.id = 2;
  plane_line_list.type = visualization_msgs::Marker::LINE_LIST;
  plane_line_list.ns = "plane_lines";
  plane_line_list.scale.x = 0.15;
  plane_line_list.scale.y = 0.15;
  plane_line_list.scale.z = 0.15;
  plane_line_list.color.r = 0;
  plane_line_list.color.g = 0;
  plane_line_list.color.b = 1;
  plane_line_list.color.a = 1;
  plane_line_list.pose.orientation.w = 1;
  for (size_t pi = 0; pi < plane_pairs->size(); ++pi) {
    geometry_msgs::Point ori_pt;
    ori_pt.x = (*plane_pairs)[pi].ori_pt.x;
    ori_pt.y = (*plane_pairs)[pi].ori_pt.y;
    ori_pt.z = (*plane_pairs)[pi].ori_pt.z;

    for (size_t ni = 0; ni < 3; ++ni) {
      geometry_msgs::Point neigh_pt;
      neigh_pt.x = (*plane_pairs)[pi].neigh_pt[ni].x;
      neigh_pt.y = (*plane_pairs)[pi].neigh_pt[ni].y;
      neigh_pt.z = (*plane_pairs)[pi].neigh_pt[ni].z;

      plane_line_list.points.push_back(ori_pt);
      plane_line_list.points.push_back(std::move(neigh_pt));
    }
  }
  associate_marker_array.markers.push_back(plane_line_list);
  associate_pub_.publish(associate_marker_array);
#endif
}

Eigen::Isometry3d LOAMSystem::getEstimated() {
  Eigen::Isometry3d ret;
  return curr_state_transform_;
}

}  // namespace nalio
