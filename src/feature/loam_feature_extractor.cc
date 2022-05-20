#include "nalio/feature/loam_feature_extractor.hh"

#include <pcl/filters/voxel_grid.h>
#include <ros/console.h>
#include <queue>
#include <unordered_set>

#include "nalio/utils/log_utils.hh"

namespace nalio {

int LOAMFeatureExtractor::extract(const PointCloudT::ConstPtr& cloud_in,
                                  std::vector<LOAMFeature>* features) {
  std::vector<PointCloudT> scan_pts;
  if (!(0 == splitScans(*cloud_in, &scan_pts))) {
    ROS_ERROR_STREAM_FUNC("Failed to splitScans");
    return -1;
  }

  PointInfo point_infos[64][6250];
  for (int line = 0; line <= 50; ++line) {
    const PointCloudT& line_pts = scan_pts[line];
    for (int i = 5; i <= line_pts.size() - 6; ++i) {
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
      // 为每个点设置曲率。
      point_infos[line][i].curvature =
          diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
      point_infos[line][i].neighbor_selected = 0;
      point_infos[line][i].ind = i;
    }
  }

  PointCloudT::Ptr less_flat_cloud;
  for (int line = 0; line <= 50; ++line) {
    uint num_line_pts = scan_pts[line].size();

    std::priority_queue<PointInfo, std::vector<PointInfo>, SharpCmp>
        sharp_queue;
    std::priority_queue<PointInfo, std::vector<PointInfo>, FlatCmp> flat_queue;
    std::unordered_set<uint16_t> neighbor_selected, edge_inds;

    // 分为六个扇区
    for (int sec = 0; sec < 6; ++sec) {
      uint16_t sp = 5 + (num_line_pts - 10) / 6 * sec;
      uint16_t ep = 5 + (num_line_pts - 10) / 6 * (sec + 1) - 1;

      for (int pi = sp; pi <= ep; ++pi) {
        // 保存曲率最大的20个点
        if (sharp_queue.empty() ||
            point_infos[line][pi].curvature > sharp_queue.top().curvature) {
          sharp_queue.push(point_infos[line][pi]);
        }
        if (sharp_queue.size() > 20) {
          sharp_queue.pop();
        }

        // 保存曲率最小的4个点
        if (flat_queue.empty() ||
            point_infos[line][pi].curvature < flat_queue.top().curvature) {
          flat_queue.push(point_infos[line][pi]);
        }
        if (flat_queue.size() > 4) {
          flat_queue.pop();
        }
      }

      while (!sharp_queue.empty()) {
        uint16_t pt_ind = sharp_queue.top().ind;
        if (neighbor_selected.count(pt_ind) > 0) {
          continue;
        }
        if (sharp_queue.size() <= 2) {
          LOAMFeature::Ptr edge_feature(new LOAMFeature);
          edge_feature->type.val = LOAMFeature::Type::kSharp;
          edge_feature->pt = sharp_queue.top().pt;
          features->push_back(*edge_feature);
        }
        LOAMFeature::Ptr weak_edge_feature(new LOAMFeature);
        weak_edge_feature->type.val = LOAMFeature::Type::kLessSharp;
        weak_edge_feature->pt = sharp_queue.top().pt;
        features->push_back(*weak_edge_feature);

        edge_inds.insert(pt_ind);

        for (int j = pt_ind - 5; j < pt_ind + 6; ++j) {
          float diff_x = scan_pts[line][j].x - scan_pts[line][pt_ind].x;
          float diff_y = scan_pts[line][j].y - scan_pts[line][pt_ind].y;
          float diff_z = scan_pts[line][j].z - scan_pts[line][pt_ind].z;
          // 将离当前点sqrt 0.05m内的点排除，防止特征点过于密集
          if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z >= 0.05) {
            continue;
          }
          neighbor_selected.insert(j);
        }

        sharp_queue.pop();
      }

      while (!flat_queue.empty()) {
        uint16_t pt_ind = flat_queue.top().ind;
        if (neighbor_selected.count(pt_ind) > 0) {
          continue;
        }
        LOAMFeature::Ptr plane_feature(new LOAMFeature);
        plane_feature->type.val = LOAMFeature::Type::kFlat;
        plane_feature->pt = flat_queue.top().pt;
        features->push_back(*plane_feature);

        for (int j = pt_ind - 5; j < pt_ind + 6; ++j) {
          float diff_x = scan_pts[line][j].x - scan_pts[line][pt_ind].x;
          float diff_y = scan_pts[line][j].y - scan_pts[line][pt_ind].y;
          float diff_z = scan_pts[line][j].z - scan_pts[line][pt_ind].z;
          // 将离当前点sqrt 0.05m内的点排除，防止特征点过于密集
          if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z >= 0.05) {
            continue;
          }
          neighbor_selected.insert(j);
        }
        flat_queue.pop();
      }
      for (int i = sp; i <= ep; ++i) {
        uint pt_ind = point_infos[line][i].ind;
        if (edge_inds.count(pt_ind) > 0) {
          continue;
        }
        less_flat_cloud->push_back(scan_pts[line][pt_ind]);
      }
    }
  }

  pcl::VoxelGrid<NalioPoint> ds_filter;
  ds_filter.setInputCloud(less_flat_cloud);
  ds_filter.setLeafSize(0.2, 0.2, 0.2);
  ds_filter.filter(*less_flat_cloud);

  for (int pi = 0; pi < less_flat_cloud->size(); ++pi) {
    LOAMFeature::Ptr less_flat_feature(new LOAMFeature);
    less_flat_feature->pt = less_flat_cloud->at(pi);
    less_flat_feature->type.val = LOAMFeature::Type::kLessFlat;
    features->push_back(*less_flat_feature);
  }

  return 0;
}

int LOAMFeatureExtractor::splitScans(const PointCloudT& cloud_in,
                                     std::vector<PointCloudT>* scan_pts) {
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
  scan_pts->clear();
  scan_pts->resize(64);
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
    // 给每个点的intensity设置为这个点的时刻的ratio。
    pt.intensity =
        line + kScanPeriod / (end_ori - start_ori) * (ori - start_ori);
    scan_pts->at(line).push_back(pt);
  }
  return 0;
}

}  // namespace nalio
