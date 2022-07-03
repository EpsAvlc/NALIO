#ifndef NALIO_CERES_LOAM_FACTORY_HH
#define NALIO_CERES_LOAM_FACTORY_HH

#include <ceres/ceres.h>

#include "nalio/data/nalio_data.hh"

namespace nalio {
struct LOAMEdgePair {
  using PointT = NalioPoint;
  PointT ori_pt;
  PointT neigh_pt[2];
};

struct LOAMPlanePair {
  using PointT = NalioPoint;
  PointT ori_pt;
  PointT neigh_pt[3];
};

// Paper: LOAM: Lidar Odometry and Mapping in Real-time.
struct LOAMEdgeFactor {
 public:
  LOAMEdgeFactor(const LOAMEdgePair& edge_pair)
      : ori_pt_(edge_pair.ori_pt),
        neigh_pt_a_(edge_pair.neigh_pt[0]),
        neigh_pt_b_(edge_pair.neigh_pt[1]) {}

  template <typename T>
  bool operator()(const T* q, const T* t, T* residual) const {
    Eigen::Matrix<T, 3, 1> ori_pt{T(ori_pt_.x), T(ori_pt_.y), T(ori_pt_.z)};
    Eigen::Matrix<T, 3, 1> neigh_pt_a{T(neigh_pt_a_.x), T(neigh_pt_a_.y),
                                      T(neigh_pt_a_.z)};
    Eigen::Matrix<T, 3, 1> neigh_pt_b{T(neigh_pt_b_.x), T(neigh_pt_b_.y),
                                      T(neigh_pt_b_.z)};
    Eigen::Matrix<T, 3, 1> curr2last_t{T(t[0]), T(t[1]), T(t[2])};
    Eigen::Quaternion<T> curr2last_q{T(q[3]), T(q[0]), T(q[1]), T(q[2])};
    Eigen::Matrix<T, 3, 1> ori_pt_in_last = curr2last_q * ori_pt + curr2last_t;

    Eigen::Matrix<T, 3, 1> nu =
        (ori_pt_in_last - neigh_pt_a).cross(ori_pt_in_last - neigh_pt_b);
    T de = (neigh_pt_a - neigh_pt_b).norm();

    residual[0] = nu(0) / de;
    residual[1] = nu(1) / de;
    residual[2] = nu(2) / de;

    return true;
  }

  static ceres::CostFunction* Create(const LOAMEdgePair& edge_pair) {
    return (new ceres::AutoDiffCostFunction<LOAMEdgeFactor, 3, 4, 3>(
        new LOAMEdgeFactor(edge_pair)));
  }

 private:
  NalioPoint ori_pt_, neigh_pt_a_, neigh_pt_b_;
};

// Paper: LOAM: Lidar Odometry and Mapping in Real-time.
struct LOAMPlaneFactor {
 public:
  LOAMPlaneFactor(const LOAMPlanePair& plane_pair)
      : pt_i_(plane_pair.ori_pt),
        pt_j_(plane_pair.neigh_pt[0]),
        pt_l_(plane_pair.neigh_pt[1]),
        pt_m_(plane_pair.neigh_pt[2]) {}

  template <typename T>
  bool operator()(const T* q, const T* t, T* residual) const {
    Eigen::Matrix<T, 3, 1> pt_i{T(pt_i_.x), T(pt_i_.y), T(pt_i_.z)};
    Eigen::Matrix<T, 3, 1> pt_j{T(pt_j_.x), T(pt_j_.y), T(pt_j_.z)};
    Eigen::Matrix<T, 3, 1> pt_l{T(pt_l_.x), T(pt_l_.y), T(pt_l_.z)};
    Eigen::Matrix<T, 3, 1> pt_m{T(pt_m_.x), T(pt_m_.y), T(pt_m_.z)};
    Eigen::Matrix<T, 3, 1> curr2last_t{T(t[0]), T(t[1]), T(t[2])};
    Eigen::Quaternion<T> curr2last_q{T(q[3]), T(q[0]), T(q[1]), T(q[2])};

    Eigen::Matrix<T, 3, 1> lpt_i = curr2last_q * pt_i + curr2last_t;
    Eigen::Matrix<T, 3, 1> jml = (pt_j - pt_l).cross(pt_j - pt_m);
    jml.normalize();
    residual[0] = (lpt_i - pt_j).dot(jml);

    return true;
  }

  static ceres::CostFunction* Create(const LOAMPlanePair& l) {
    return (new ceres::AutoDiffCostFunction<LOAMPlaneFactor, 1, 4, 3>(
        new LOAMPlaneFactor(l)));
  }

 private:
  NalioPoint pt_i_, pt_j_, pt_l_, pt_m_;
};
};  // namespace nalio

#endif  // LOAM_FACTORY_HH
