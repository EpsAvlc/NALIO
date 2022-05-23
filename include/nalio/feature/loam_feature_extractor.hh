#ifndef NALIO_FEATURE_LOAM_FEATURE_EXTACTOR_HH__
#define NALIO_FEATURE_LOAM_FEATURE_EXTACTOR_HH__

#include <algorithm>
#include <array>
#include <queue>
#include <unordered_set>

#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <ros/console.h>

#include "nalio/data/data.hh"
#include "nalio/feature/feature_extractor.hh"
#include "nalio/utils/log_utils.hh"

namespace nalio {

struct LOAMFeaturePackage : public FeaturePackage {
  using Ptr = std::shared_ptr<LOAMFeaturePackage>;

  LOAMFeaturePackage()
      : sharp_cloud(new pcl::PointCloud<NalioPoint>),
        less_sharp_cloud(new pcl::PointCloud<NalioPoint>),
        flat_cloud(new pcl::PointCloud<NalioPoint>),
        less_flat_cloud(new pcl::PointCloud<NalioPoint>) {}

  void clear() {
    sharp_cloud->clear();
    less_sharp_cloud->clear();
    flat_cloud->clear();
    less_flat_cloud->clear();
  }

  pcl::PointCloud<NalioPoint>::Ptr sharp_cloud;
  pcl::PointCloud<NalioPoint>::Ptr less_sharp_cloud;
  pcl::PointCloud<NalioPoint>::Ptr flat_cloud;
  pcl::PointCloud<NalioPoint>::Ptr less_flat_cloud;
};

// Currently it can only be applied to HDL-64.
template <uint16_t N>
class LOAMFeatureExtractor
    : public FeatureExtractor<NalioPoint, LOAMFeaturePackage> {
 public:
  using PointCloudT = pcl::PointCloud<NalioPoint>;
  bool extract(const PointCloudT::ConstPtr&,
               LOAMFeaturePackage::Ptr features) override;

 private:
  struct PointInfo {
    float curvature;
    bool neighbor_selected;  // 是否其邻居点已被选作Feature点
    uint16_t ind;            // 原本的下标
  };

  int splitScans(const PointCloudT& cloud_in,
                 std::array<PointCloudT, N>* scan_pts);
  const double kScanPeriod = 0.1;
  const uint8_t kSections = 6;
  const uint8_t kMaxSharpPts = 2;
  const uint8_t kMaxLessSharpPts = 20;
  const uint8_t kMaxFlatPts = 4;

  std::array<std::array<PointInfo, 6250>, N> point_infos_;
  std::array<PointCloudT, N> scan_pts_;
};

template <uint16_t N>
bool LOAMFeatureExtractor<N>::extract(const PointCloudT::ConstPtr& cloud_in,
                                      LOAMFeaturePackage::Ptr features) {
  if (!cloud_in) {
    throw(std::invalid_argument("cloud_in is empty."));
  }

  if (!features) {
    throw(std::invalid_argument("features is empty."));
  }

  features->clear();

  if (!(0 == splitScans(*cloud_in, &scan_pts_))) {
    ROS_ERROR_STREAM_FUNC("Failed to splitScans");
    return -1;
  }

  for (int line = 0; line <= 50; ++line) {
    const PointCloudT& line_pts = scan_pts_[line];
    for (size_t i = 5; i <= line_pts.size() - 6; ++i) {
      float diff_x = line_pts[i - 5].x + line_pts[i - 4].x + line_pts[i - 3].x +
                     line_pts[i - 2].x + line_pts[i - 1].x -
                     10 * line_pts[i].x + line_pts[i + 1].x +
                     line_pts[i + 2].x + line_pts[i + 3].x + line_pts[i + 4].x +
                     line_pts[i + 5].x;
      float diff_y = line_pts[i - 5].y + line_pts[i - 4].y + line_pts[i - 3].y +
                     line_pts[i - 2].y + line_pts[i - 1].y -
                     10 * line_pts[i].y + line_pts[i + 1].y +
                     line_pts[i + 2].y + line_pts[i + 3].y + line_pts[i + 4].y +
                     line_pts[i + 5].y;
      float diff_z = line_pts[i - 5].z + line_pts[i - 4].z + line_pts[i - 3].z +
                     line_pts[i - 2].z + line_pts[i - 1].z -
                     10 * line_pts[i].z + line_pts[i + 1].z +
                     line_pts[i + 2].z + line_pts[i + 3].z + line_pts[i + 4].z +
                     line_pts[i + 5].z;

      point_infos_[line][i].curvature =
          diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
      point_infos_[line][i].neighbor_selected = false;
      point_infos_[line][i].ind = i;
    }
  }

  for (int line = 0; line < N; ++line) {
    uint16_t num_line_pts = scan_pts_[line].size();
    if (num_line_pts < 6) {
      continue;
    }
    std::unordered_set<uint16_t> neighbor_selected, edge_inds;

    pcl::PointCloud<NalioPoint>::Ptr less_flat_cloud_tmp(
        new pcl::PointCloud<NalioPoint>);
    for (int sec = 0; sec < kSections; ++sec) {
      uint16_t sp = 5 + (num_line_pts - 10) / 6 * sec;
      uint16_t ep = 5 + (num_line_pts - 10) / 6 * (sec + 1) - 1;

      std::sort(point_infos_[line].begin() + sp,
                point_infos_[line].begin() + ep + 1,
                [](const PointInfo& lhs, const PointInfo& rhs) {
                  return lhs.curvature < rhs.curvature;
                });

      for (int pi = ep; pi >= sp; --pi) {
        uint16_t& pt_ind = point_infos_[line][pi].ind;
        if (neighbor_selected.count(pt_ind) > 0) {
          continue;
        }
        if (features->sharp_cloud->size() < kMaxSharpPts) {
          features->sharp_cloud->push_back(scan_pts_[line][pt_ind]);
        }
        features->less_sharp_cloud->push_back(scan_pts_[line][pt_ind]);

        if (features->less_sharp_cloud->size() >= kMaxLessSharpPts) {
          break;
        }
        for (int j = pt_ind - 5; j < pt_ind + 6; ++j) {
          float diff_x = scan_pts_[line][j].x - scan_pts_[line][pt_ind].x;
          float diff_y = scan_pts_[line][j].y - scan_pts_[line][pt_ind].y;
          float diff_z = scan_pts_[line][j].z - scan_pts_[line][pt_ind].z;
          // 将离当前点sqrt 0.05m内的点排除，防止特征点过于密集
          if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > 0.05) {
            break;
          }
          neighbor_selected.insert(j);
        }
        edge_inds.insert(pt_ind);
      }

      for (int pi = sp; pi <= ep; ++pi) {
        uint16_t& pt_ind = point_infos_[line][pi].ind;
        if (neighbor_selected.count(pt_ind) > 0) {
          continue;
        }
        if (features->flat_cloud->size() < kMaxFlatPts) {
          features->flat_cloud->push_back(scan_pts_[line][pt_ind]);
        } else {
          break;
        }
        for (int j = pt_ind - 5; j < pt_ind + 6; ++j) {
          float diff_x = scan_pts_[line][j].x - scan_pts_[line][pt_ind].x;
          float diff_y = scan_pts_[line][j].y - scan_pts_[line][pt_ind].y;
          float diff_z = scan_pts_[line][j].z - scan_pts_[line][pt_ind].z;
          // 将离当前点sqrt 0.05m内的点排除，防止特征点过于密集
          if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z >= 0.05) {
            continue;
          }
          neighbor_selected.insert(j);
        }
      }

      for (int i = sp; i <= ep; ++i) {
        uint pt_ind = point_infos_[line][i].ind;
        if (edge_inds.count(pt_ind) > 0) {
          continue;
        }
        less_flat_cloud_tmp->push_back(scan_pts_[line][pt_ind]);
      }
    }

    //printf("downsample point size %ld \n", less_flat_cloud_tmp->size());
    pcl::PointCloud<NalioPoint> less_flat_cloud_ds;
    pcl::VoxelGrid<NalioPoint> ds_filter;
    ds_filter.setInputCloud(less_flat_cloud_tmp);
    ds_filter.setLeafSize(0.2, 0.2, 0.2);
    ds_filter.filter(less_flat_cloud_ds); // TODO: elapse too much time here.
    *features->less_flat_cloud += less_flat_cloud_ds;
  }

  return true;
}

template <uint16_t N>
int LOAMFeatureExtractor<N>::splitScans(const PointCloudT& cloud_in,
                                        std::array<PointCloudT, N>* scan_pts) {
  NalioPoint pt;
  int num_points = cloud_in.size();
  // atan2 (-pi, pi]
  // 表示当前点与x轴正方向的夹角，并且从x轴逆时针的角度为正，顺时针的角度为负
  float start_ori = -atan2(cloud_in[0].y, cloud_in[1].x);
  float end_ori =
      -atan2(cloud_in[num_points - 1].y, cloud_in[num_points - 1].x);
  if (end_ori - start_ori > 3 * M_PI) {
    end_ori -= 2 * M_PI;
  } else if (end_ori - start_ori < M_PI) {
    end_ori += 2 * M_PI;
  }

  bool half_pass = false;
  for (uint i = 0; i < N; ++i) {
    scan_pts->at(i).clear();
  }
  for (size_t i = 0; i < cloud_in.size(); ++i) {
    pt.x = cloud_in[i].x;
    pt.y = cloud_in[i].y;
    pt.z = cloud_in[i].z;

    float angle = atan(pt.z / sqrt(pt.x * pt.x + pt.y * pt.y)) * 180 / M_PI;
    int line = 0;
    // HDL64 激光雷达以 8.83角度为界限。 以上为upper
    // 的32线激光雷达，以下为lower的32线。
    // 上32的分辨率为0.333，下32的分辨率为0.5
    if (angle >= -8.83)
      line = int((2 - angle) * 3.0 + 0.5);
    else
      line = 64 / 2 + int((-8.83 - angle) * 2.0 + 0.5);

    // use [0 50]  > 50 remove outlies
    if (angle > 2 || angle < -24.33 || line > 50 || line < 0) {
      continue;
    }

    // orientation of each point
    float ori = -atan2(pt.y, pt.x);
    if (!half_pass) {
      if (ori < start_ori - M_PI / 2)  // 起始角度为正的情况
      {
        ori += 2 * M_PI;
      } else if (ori > start_ori + M_PI * 3 / 2)  // 起始角度为负的情况
      {
        ori -= 2 * M_PI;
      }

      if (ori - start_ori > M_PI)  // 角度超过半圈 用end_ori来加角度
      {
        half_pass = true;
      }
    } else {
      ori += 2 * M_PI;
      if (ori < end_ori - M_PI * 3 / 2) {
        ori += 2 * M_PI;
      } else if (ori > end_ori + M_PI / 2) {
        ori -= 2 * M_PI;
      }
    }
    // 给每个点的relative time设置为这个点的时刻的ratio。
    pt.rel_time =
        line + kScanPeriod / (end_ori - start_ori) * (ori - start_ori);
    pt.line = line;
    scan_pts->at(line).push_back(pt);
  }
  return 0;
}

}  // namespace nalio

#endif
