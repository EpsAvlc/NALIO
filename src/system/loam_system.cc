#include "nalio/system/loam_system.hh"

#include <pcl/kdtree/kdtree_flann.h>
#include "nalio/utils/log_utils.hh"

#ifdef NALIO_DEBUG
#include <pcl/console/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include "nalio/utils/rviz_utils.hh"
#endif

#include "nalio/data/nalio_data.hh"

namespace nalio {

LOAMSystem::LOAMSystem()
    :
#ifdef USE_UNOS
      state_(unos::SO3(), unos::Vec3()),
#endif
      flg_initialized_(false),
      flg_running_(true),
      nh_("NALIO_LOAM"),
      status_(0) {
}

LOAMSystem::~LOAMSystem() {
  flg_running_ = false;
  if (odometry_thread_.joinable()) {
    // feature_package_list_cv_.notify_all();
    odometry_thread_.join();
  }

  if (mapping_thread_.joinable()) {
    mapping_thread_.join();
  }
}

void LOAMSystem::init() {
#ifdef NALIO_DEBUG
  pub_sharp_feature_ = nh_.advertise<sensor_msgs::PointCloud2>("sharp_feature", 1);
  pub_less_sharp_feature_ = nh_.advertise<sensor_msgs::PointCloud2>("less_sharp_feature", 1);
  pub_flat_feature_ = nh_.advertise<sensor_msgs::PointCloud2>("flat_feature", 1);
  pub_less_flat_feature_ = nh_.advertise<sensor_msgs::PointCloud2>("less_flat_feature", 1);
  pub_associate_ = nh_.advertise<visualization_msgs::MarkerArray>("associate", 1);
  pub_sharp_curvature_ = nh_.advertise<visualization_msgs::MarkerArray>("NALIO/sharp_curvature", 1);
#endif
  pub_odom_path_ = nh_.advertise<nav_msgs::Path>("odom_path", 1);

#ifdef USE_UNOS
  throw(std::logic_error("not implement yet."));
#else
  robot2odom_.reset();
  robot2map_.reset();
#endif

  initSlidingGridMap();

  odometry_thread_ = std::thread(&LOAMSystem::odometryThread, this);
  mapping_thread_ = std::thread(&LOAMSystem::mappingThread, this);
  flg_initialized_ = true;
}

void LOAMSystem::stop() {}

void LOAMSystem::feedData(const datahub::MessagePackage& msgs) {
  if (!flg_initialized_) {
    NLOG_ERROR_STREAM(" System has not been initialized.");
    return;
  }
  NLOG_INFO_STREAM("Feed a message pack. Frame id: " << msgs[0][0]->header.frame_id);
  propagate();

  LOAMFeaturePackage::Ptr feature_package(new LOAMFeaturePackage);
  PointCloudData::Ptr pc_data;
  std::string msg_frame_id = "velodyne";
  for (int mi = 0; mi < msgs.size(); ++mi) {
    if (msgs[mi][0]->type.val == datahub::Message::Type::kLidar) {
      pc_data = std::dynamic_pointer_cast<PointCloudData>(msgs[mi][0]->data);
      // msg_frame_id = msgs[mi][0]->header.frame_id;
      break;
    }
  }

  pcl::console::TicToc tt;
  tt.tic();
  if (!feature_extractor_.extract(pc_data->point_cloud, feature_package)) {
    NLOG_ERROR_STREAM("Failed to extract features.");
  }
  double fe_time = tt.toc();
  NLOG_INFO_STREAM("feature extractor elapse time: " << fe_time << "ms.");
  NLOG_INFO_STREAM("sharp feature size: " << feature_package->sharp_cloud->size());
  NLOG_INFO_STREAM("flat feature size: " << feature_package->flat_cloud->size());

  {
    std::lock_guard<std::mutex> lock(mtx_deq_feature_package_);
    deq_feature_package_.push_back(feature_package);
  }
  cv_deq_feature_package_.notify_all();

#ifdef NALIO_DEBUG
  NLOG_INFO_STREAM("Feed data.");

  sensor_msgs::PointCloud2 flat_pc_msg, less_flat_pc_msg, sharp_pc_msg, less_sharp_pc_msg;
  pcl::toROSMsg(*feature_package->flat_cloud, flat_pc_msg);
  pcl::toROSMsg(*feature_package->less_flat_cloud, less_flat_pc_msg);
  pcl::toROSMsg(*feature_package->sharp_cloud, sharp_pc_msg);
  pcl::toROSMsg(*feature_package->less_sharp_cloud, less_sharp_pc_msg);

  flat_pc_msg.header.frame_id = msg_frame_id;
  less_flat_pc_msg.header.frame_id = msg_frame_id;
  sharp_pc_msg.header.frame_id = msg_frame_id;
  less_sharp_pc_msg.header.frame_id = msg_frame_id;

  pub_flat_feature_.publish(flat_pc_msg);
  pub_less_flat_feature_.publish(less_flat_pc_msg);
  pub_sharp_feature_.publish(sharp_pc_msg);
  pub_less_sharp_feature_.publish(less_sharp_pc_msg);

  // display sharp curvatures.
  size_t sharp_size = feature_package->sharp_cloud->size();
  std::vector<std::string> texts(sharp_size);
  std::vector<Eigen::Vector3d> positions(sharp_size);
  for (size_t si = 0; si < feature_package->sharp_curvatures.size(); ++si) {
    texts[si] =
        std::to_string(feature_package->sharp_inds[si]) + ", " + std::to_string(feature_package->sharp_curvatures[si]);
    positions[si] = feature_package->sharp_cloud->at(si).getVector3fMap().cast<double>();
  }
  visualization_msgs::MarkerArray sharp_curvature_msg = rviz_utils::putTexts(texts, positions, "velodyne");
  pub_sharp_curvature_.publish(sharp_curvature_msg);
#endif
}

void LOAMSystem::feedData(const LOAMFeaturePackage::Ptr& feature_package) {
  {
    std::lock_guard<std::mutex> lock(mtx_deq_feature_package_);
    deq_feature_package_.push_back(feature_package);
  }
  cv_deq_feature_package_.notify_all();
}

void LOAMSystem::initSlidingGridMap() {
  SlidingGridMapT::Config config;
  config.initial_method = [](size_t const ind) {
    std::array<pcl::PointCloud<PointT>::Ptr, 2> ret;
    for (size_t pi = 0; pi < ret.size(); ++pi) {
      ret[pi] = boost::make_shared<pcl::PointCloud<PointT>>();
    }
    return ret;
  };
  config.origin.setZero();
  config.resolution = 25;
  config.size[0] = config.size[1] = 20;
  config.size[2] = 10;
  config.move_step[0] = config.move_step[1] = config.resolution * 4;
  config.move_step[2] = config.resolution * 2;
  for (size_t ci = 0; ci < 3; ++ci) {
    config.move_threshold[ci] = config.move_step[ci] / 2;
  }
  ptr_sliding_grid_map_.reset(new SlidingGridMapT(config));
}

void LOAMSystem::propagate() {
  // Eigen::Isometry3d propagated_pose = propagator_.propagate();
  // state_.set(eigenToState(propagated_pose));
}

void LOAMSystem::update() {}

void LOAMSystem::associateFromLast(const LOAMFeaturePackage::Ptr& prev_feature,
                                   const LOAMFeaturePackage::Ptr& curr_feature, std::vector<LOAMEdgePair>* edge_pairs,
                                   std::vector<LOAMPlanePair>* plane_pairs) {
  const double kMaxAssociateDistanceSq = 25;
  const double kNearbyScan = 2.5;
  if (!prev_feature->flat_cloud || !prev_feature->less_flat_cloud || !prev_feature->sharp_cloud ||
      !prev_feature->less_sharp_cloud) {
    throw(std::invalid_argument("prev_feature has null ptr!"));
  }
  if (!curr_feature->flat_cloud || !curr_feature->less_flat_cloud || !curr_feature->sharp_cloud ||
      !curr_feature->less_sharp_cloud) {
    throw(std::invalid_argument("curr_feature has null ptr!"));
  }
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
  NLOG_INFO_STREAM("Sharp cloud size: " << curr_feature->sharp_cloud->size());
  for (size_t ei = 0; ei < curr_feature->sharp_cloud->size(); ++ei) {
    const PointT& pt_curr = curr_feature->sharp_cloud->at(ei);
    PointT pt_curr_in_last;
    transformPointToLastFrame(pt_curr, curr2last_q_, curr2last_t_, &pt_curr_in_last);

    edge_kdtree.nearestKSearch(pt_curr_in_last, 1, pt_search_inds, pt_search_dists);
    if (pt_search_dists[0] > kMaxAssociateDistanceSq) {
      continue;
    }
    const PointT& closest_pt = (*prev_feature->less_sharp_cloud)[pt_search_inds[0]];
    int second_closest_pt_ind = -1;
    double second_closest_pt_dist = kMaxAssociateDistanceSq;
    for (size_t ej = pt_search_inds[0] + 1; ej < prev_feature->less_sharp_cloud->size(); ++ej) {
      if ((*prev_feature->less_sharp_cloud)[ej].line <= closest_pt.line) {
        continue;
      }

      if ((*prev_feature->less_sharp_cloud)[ej].line > closest_pt.line + kNearbyScan) {
        break;
      }

      double distance =
          ((*prev_feature->less_sharp_cloud)[ej].getVector3fMap() - pt_curr_in_last.getVector3fMap()).squaredNorm();
      if (distance < second_closest_pt_dist) {
        second_closest_pt_ind = ej;
        second_closest_pt_dist = distance;
      }
    }

    for (int32_t ej = pt_search_inds[0] - 1; ej > 0; --ej) {
      if ((*prev_feature->less_sharp_cloud)[ej].line >= closest_pt.line) {
        continue;
      }

      if ((*prev_feature->less_sharp_cloud)[ej].line < closest_pt.line - kNearbyScan) {
        break;
      }
      double distance =
          ((*prev_feature->less_sharp_cloud)[ej].getVector3fMap() - pt_curr_in_last.getVector3fMap()).squaredNorm();
      if (distance < second_closest_pt_dist) {
        second_closest_pt_ind = ej;
        second_closest_pt_dist = distance;
      }
    }

    if (second_closest_pt_ind >= 0) {
      LOAMEdgePair edge_pair;
      edge_pair.ori_pt = pt_curr;
      edge_pair.neigh_pt[0] = closest_pt;
      edge_pair.neigh_pt[1] = (*prev_feature->less_sharp_cloud)[second_closest_pt_ind];
      edge_pairs->push_back(std::move(edge_pair));
    }
  }

  static pcl::KdTreeFLANN<PointT> plane_kdtree;
  plane_kdtree.setInputCloud(prev_feature->less_flat_cloud);
  NLOG_INFO_STREAM("Current flat cloud size: " << curr_feature->flat_cloud->size());
  for (size_t pi = 0; pi < curr_feature->flat_cloud->size(); ++pi) {
    const PointT& pt_sel = curr_feature->flat_cloud->at(pi);
    PointT pt_curr_in_last;
    transformPointToLastFrame(pt_sel, curr2last_q_, curr2last_t_, &pt_curr_in_last);
    plane_kdtree.nearestKSearch(pt_curr_in_last, 1, pt_search_inds, pt_search_dists);
    if (pt_search_dists[0] > kMaxAssociateDistanceSq) {
      continue;
    }

    const PointT& closest_pt = (*prev_feature->less_flat_cloud)[pt_search_inds[0]];
    int l_ind = -1, m_ind = -1;
    double l_distance = kMaxAssociateDistanceSq;
    double m_distance = kMaxAssociateDistanceSq;

    for (size_t pj = pt_search_inds[0] + 1; pj < prev_feature->less_flat_cloud->size(); ++pj) {
      if ((*prev_feature->less_flat_cloud)[pj].line <= closest_pt.line) {
        continue;
      }
      if ((*prev_feature->less_flat_cloud)[pj].line > closest_pt.line + kNearbyScan) {
        break;
      }
      double distance =
          ((*prev_feature->less_flat_cloud)[pj].getVector3fMap() - closest_pt.getVector3fMap()).squaredNorm();
      if (distance < l_distance) {
        l_ind = pj;
        l_distance = distance;
      }
    }

    for (int32_t pj = pt_search_inds[0] - 1; pj >= 0; --pj) {
      if ((*prev_feature->less_flat_cloud)[pj].line >= closest_pt.line) {
        continue;
      }
      if ((*prev_feature->less_flat_cloud)[pj].line < closest_pt.line - kNearbyScan) {
        break;
      }
      double distance =
          ((*prev_feature->less_flat_cloud)[pj].getVector3fMap() - closest_pt.getVector3fMap()).squaredNorm();
      if (distance < m_distance) {
        m_ind = pj;
        m_distance = distance;
      }
    }

    if (l_ind >= 0 && m_ind >= 0) {
      LOAMPlanePair plane_pair;
      plane_pair.ori_pt = pt_sel;
      plane_pair.neigh_pt[0] = closest_pt;
      plane_pair.neigh_pt[1] = (*prev_feature->less_flat_cloud)[l_ind];
      plane_pair.neigh_pt[2] = (*prev_feature->less_flat_cloud)[m_ind];
      plane_pairs->push_back(std::move(plane_pair));
    }
  }

#ifdef NALIO_DEBUG
  visualization_msgs::MarkerArray associate_marker_array;

  for (size_t ei = 0; ei < edge_pairs->size(); ++ei) {
    visualization_msgs::Marker edge_line_list, plane_line_list;
    edge_line_list.header.frame_id = "camera_init";
    edge_line_list.header.stamp = ros::Time::now();
    edge_line_list.action = visualization_msgs::Marker::ADD;
    edge_line_list.id = ei;
    edge_line_list.type = visualization_msgs::Marker::LINE_LIST;
    edge_line_list.ns = "edge_lines";
    edge_line_list.scale.x = 0.15;
    edge_line_list.scale.y = 0.15;
    edge_line_list.scale.z = 0.15;
    edge_line_list.color.r = rand() % 255 / 255.;
    edge_line_list.color.g = rand() % 255 / 255.;
    edge_line_list.color.b = rand() % 255 / 255.;
    edge_line_list.color.a = 1;
    edge_line_list.pose.orientation.w = 1;

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
    associate_marker_array.markers.push_back(edge_line_list);
  }

  // plane_line_list.header.frame_id = "camera_init";
  // plane_line_list.header.stamp = ros::Time::now();
  // plane_line_list.action = visualization_msgs::Marker::ADD;
  // plane_line_list.id = 2;
  // plane_line_list.type = visualization_msgs::Marker::LINE_LIST;
  // plane_line_list.ns = "plane_lines";
  // plane_line_list.scale.x = 0.15;
  // plane_line_list.scale.y = 0.15;
  // plane_line_list.scale.z = 0.15;
  // plane_line_list.color.r = 0;
  // plane_line_list.color.g = 0;
  // plane_line_list.color.b = 1;
  // plane_line_list.color.a = 1;
  // plane_line_list.pose.orientation.w = 1;
  // for (size_t pi = 0; pi < plane_pairs->size(); ++pi) {
  //   geometry_msgs::Point ori_pt;
  //   ori_pt.x = (*plane_pairs)[pi].ori_pt.x;
  //   ori_pt.y = (*plane_pairs)[pi].ori_pt.y;
  //   ori_pt.z = (*plane_pairs)[pi].ori_pt.z;

  //   for (size_t ni = 0; ni < 3; ++ni) {
  //     geometry_msgs::Point neigh_pt;
  //     neigh_pt.x = (*plane_pairs)[pi].neigh_pt[ni].x;
  //     neigh_pt.y = (*plane_pairs)[pi].neigh_pt[ni].y;
  //     neigh_pt.z = (*plane_pairs)[pi].neigh_pt[ni].z;

  //     plane_line_list.points.push_back(ori_pt);
  //     plane_line_list.points.push_back(std::move(neigh_pt));
  //   }
  // }
  // associate_marker_array.markers.push_back(plane_line_list);
  pub_associate_.publish(associate_marker_array);
#endif
}

bool LOAMSystem::optimize(const std::vector<LOAMEdgePair>& edge_pair, const std::vector<LOAMPlanePair>& plane_pair) {
#ifdef USE_UNOS
  throw(std::logic_error("Has not been implemented."));
#else
  NLOG_INFO_STREAM("Edge pair size: " << edge_pair.size());
  static double curr2last_data_t[3]{0, 0, 0};
  static double curr2last_data_q[4]{0, 0, 0, 1};
  for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter) {
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);
    ceres::EigenQuaternionManifold* q_manifold = new ceres::EigenQuaternionManifold;
    problem.AddParameterBlock(curr2last_data_q, 4, q_manifold);
    problem.AddParameterBlock(curr2last_data_t, 3);
    ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
    for (size_t ei = 0; ei < edge_pair.size(); ++ei) {
      ceres::CostFunction* edge_factor = LOAMEdgeFactor::Create(edge_pair[ei]);
      problem.AddResidualBlock(edge_factor, loss_function, curr2last_data_q, curr2last_data_t);
    }

    for (size_t pi = 0; pi < plane_pair.size(); ++pi) {
      ceres::CostFunction* plane_factor = LOAMPlaneFactor::Create(plane_pair[pi]);
      problem.AddResidualBlock(plane_factor, loss_function, curr2last_data_q, curr2last_data_t);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 4;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
  }

  Eigen::Map<Eigen::Vector3d> curr2last_t(curr2last_data_t);
  Eigen::Map<Eigen::Quaterniond> curr2last_q(curr2last_data_q);

  robot2odom_.translation = robot2odom_.translation + robot2odom_.rotation * curr2last_t;
  robot2odom_.rotation = robot2odom_.rotation * curr2last_q;

#endif

  return true;
}

void LOAMSystem::transformPointToLastFrame(const NalioPoint& curr_p, const Eigen::Quaterniond& curr2last_q,
                                           const Eigen::Vector3d& curr2last_t, NalioPoint* last_p) {
  Eigen::Vector3d curr_pt_eigen(curr_p.x, curr_p.y, curr_p.z);
  Eigen::Vector3d last_pt_eigen = curr2last_q.toRotationMatrix() * curr_pt_eigen + curr2last_t;
  last_p->x = last_pt_eigen.x();
  last_p->y = last_pt_eigen.y();
  last_p->z = last_pt_eigen.z();
  last_p->intensity = curr_p.intensity;
  last_p->rel_time = curr_p.rel_time;
  last_p->line = curr_p.line;
}

TransformationD LOAMSystem::getEstimated() { return robot2map_; }

void LOAMSystem::odometryThread() {
  static LOAMFeaturePackage::Ptr prev_feature_package;
  while (flg_running_) {
    std::unique_lock<std::mutex> lock(mtx_deq_feature_package_);
    while (deq_feature_package_.empty() && flg_running_) {
      cv_deq_feature_package_.wait(lock, [this]() { return this->status_[Status::MSG_PACKAGE_READY]; });
    }
    NLOG_INFO_STREAM("odom process one frame.");
    auto& curr_feature_package = deq_feature_package_.front();
    if (prev_feature_package) {
      std::vector<LOAMEdgePair> edge_pairs;
      std::vector<LOAMPlanePair> plane_pairs;
      pcl::console::TicToc tt;
      tt.tic();
      associateFromLast(prev_feature_package, curr_feature_package, &edge_pairs, &plane_pairs);
      optimize(edge_pairs, plane_pairs);
      NLOG_INFO("odom elapse: %lf ms", tt.toc());
      publishOdom();
    }
    prev_feature_package = curr_feature_package;
    deq_feature_package_.pop_front();
    status_[Status::ODOMETRY_PROCESSED] = true;
    if (true == status_[Status::MAPPING_PROCESSED]) {
      status_[Status::MSG_PACKAGE_READY] = false;
    }
  }
}

void LOAMSystem::mappingThread() {
  int cnt = 0;
  static TransformationD last_robot2odom;
  while (flg_running_) {
    pcl::console::TicToc tt;
    tt.tic();
    std::unique_lock<std::mutex> lock(mtx_deq_feature_package_);
    while (deq_feature_package_.empty() && flg_running_) {
      cv_deq_feature_package_.wait(lock, [this]() { return this->status_[Status::MSG_PACKAGE_READY]; });
    }
    NLOG_INFO("wait msg elapse for %lf ms.", tt.toc());
    // TODO(Check it)
    ++cnt;
    if (cnt < 5) {
      continue;
    }
    cnt = 0;

    TransformationD robot2map_est = robot2map_ * last_robot2odom.inverse() * robot2odom_;
    NLOG_INFO_STREAM("robot2odom: " << std::endl
                                    << robot2odom_.matrix() << std::endl
                                    << "robot2map: " << std::endl
                                    << robot2map_est.matrix());
    NLOG_INFO_STREAM("odom process one frame.");
    last_robot2odom = robot2odom_;

    status_[Status::MAPPING_PROCESSED] = true;
    if (true == status_[Status::ODOMETRY_PROCESSED]) {
      status_[Status::MSG_PACKAGE_READY] = false;
    }
  }
}

void LOAMSystem::publishOdom() {
  msg_odom_path_.header.frame_id = "map";
  msg_odom_path_.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = robot2odom_.translation.x();
  pose.pose.position.y = robot2odom_.translation.y();
  pose.pose.position.z = robot2odom_.translation.z();

  Eigen::Quaterniond curr_q{robot2odom_.rotation};
  pose.pose.orientation.w = curr_q.w();
  pose.pose.orientation.x = curr_q.x();
  pose.pose.orientation.y = curr_q.y();
  pose.pose.orientation.z = curr_q.z();
  msg_odom_path_.poses.push_back(pose);
  pub_odom_path_.publish(msg_odom_path_);
}
}  // namespace nalio
