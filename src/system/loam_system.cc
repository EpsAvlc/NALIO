#include "nalio/system/loam_system.hh"

#include "nalio/utils/log_utils.hh"

#ifdef NALIO_DEBUG
#include <pcl/console/time.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#endif

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
      nh_.advertise<visualization_msgs::Marker>("NALIO/associate", 1);
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
  //* Edge feature association.
  static pcl::KdTreeFLANN<PointT> edge_kdtree;
  edge_kdtree.setInputCloud(prev_feature->less_sharp_cloud);
  std::vector<int> pt_search_inds;
  std::vector<float> pt_search_dists;
  for (size_t ei = 0; ei < curr_feature->sharp_cloud->size(); ++ei) {
    const PointT& pt_sel = curr_feature->sharp_cloud->at(ei);

    edge_kdtree.nearestKSearch(pt_sel, 1, pt_search_inds, pt_search_dists);
    const PointT& closest_pt =
        (*prev_feature->less_sharp_cloud)[pt_search_inds[0]];
    int second_closest_pt_ind = -1;
    double second_closest_pt_dist = std::numeric_limits<double>::max();
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

    for (size_t ej = pt_search_inds[0] - 1; ej > 0; --ej) {
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

  static pcl::KdTreeFLANN<PointT, PointT> plane_kdtree;
  plane_kdtree.setInputCloud(prev_feature->less_flat_cloud);
  for (size_t pi = 0; pi < curr_feature->flat_cloud->size(); ++pi) {
    const PointT& pt_sel = curr_feature->flat_cloud->at(pi);
    plane_kdtree.nearestKSearch(pt_sel, 1, pt_search_inds, pt_search_dists);
    const PointT& closest_pt =
        (*prev_feature->less_flat_cloud)[pt_search_inds[0]];
    int l_ind = -1, m_ind = -1;
    double l_distance = std::numeric_limits<double>::max();
    double m_distance = std::numeric_limits<double>::max();

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

    for (size_t pj = pt_search_inds[0] - 1; pj > 0; --pj) {
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
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "map";
  line_list.header.stamp = ros::Time::now();
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.id = 2;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.ns = "lines";
  line_list.scale.x = 0.15;
  line_list.scale.y = 0.15;
  line_list.scale.z = 0.15;
  line_list.color.r = 1;
  line_list.color.g = 0;
  line_list.color.b = 0;
  line_list.color.a = 1;

  for (size_t ei = 0; ei < edge_pairs->size(); ++ei) {
    geometry_msgs::Point ori_pt;
    ori_pt.x = (*edge_pairs)[ei].ori_pt.x;
    ori_pt.y = (*edge_pairs)[ei].ori_pt.x;
    ori_pt.z = (*edge_pairs)[ei].ori_pt.x;

    for (size_t ni = 0; ni < edge_pairs->size(); ++ni) {
      geometry_msgs::Point neigh_pt;
      neigh_pt.x = (*edge_pairs)[ei].neigh_pt[ni].x;
      neigh_pt.y = (*edge_pairs)[ei].neigh_pt[ni].y;
      neigh_pt.z = (*edge_pairs)[ei].neigh_pt[ni].z;

      line_list.points.push_back(ori_pt);
      line_list.points.push_back(neigh_pt);
    }
  }
  associate_pub_.publish(line_list);
#endif
}

Eigen::Isometry3d LOAMSystem::getEstimated() {
  Eigen::Isometry3d ret;
  return curr_state_transform_;
}

}  // namespace nalio
