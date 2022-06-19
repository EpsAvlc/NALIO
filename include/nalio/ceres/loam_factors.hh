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

struct LOAMEdgeFactor {
 public:
  LOAMEdgeFactor(const LOAMEdgePair& edge_pair)
      : ori_pt_(edge_pair.ori_pt.getVector3fMap().cast<double>()),
        neigh_pt_a_(edge_pair.neigh_pt[0].getVector3fMap().cast<double>()),
        neigh_pt_b_(edge_pair.neigh_pt[1].getVector3fMap().cast<double>()) {}

  template <typename T>
  bool operator()(const T* q, const T* t, T* residual) const {
    Eigen::Matrix<T, 3, 1> ori_pt(T(ori_pt_.x()), T(ori_pt_.y()),
                                  T(ori_pt_.z()));
    Eigen::Matrix<T, 3, 1> neigh_pt_a(T(neigh_pt_a_.x()), T(neigh_pt_a_.y()),
                                      T(neigh_pt_a_.z()));
    Eigen::Matrix<T, 3, 1> neigh_pt_b(T(neigh_pt_b_.x()), T(neigh_pt_b_.y()),
                                      T(neigh_pt_b_.z()));
  }

 private:
  Eigen::Vector3d ori_pt_, neigh_pt_a_, neigh_pt_b_;
};
};  // namespace nalio

#endif  // LOAM_FACTORY_HH
