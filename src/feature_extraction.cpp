/*
 * Created on Sat Dec 25 2021
 *
 * Copyright (c) 2021
 *
 * Author: EpsAvlc
 */

#include <iostream>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

const std::string lidar_topic = "/velodyne_points";
const double scan_period = 0.1;

using PointCloudT = pcl::PointCloud<pcl::PointXYZI>;

struct PointInfo {
  float curvature;
  float label; // 0 未标签 1 edge 2 plane
  bool neighbor_selected; // 是否其邻居点已被选作Feature点
  // uint16_t ind; // 原本的下标
};

PointInfo point_infos[64][6250];
uint point_ind[64][6250];

int splitScans(const PointCloudT& cloud_in, std::vector<PointCloudT>* scan_pts) {
  pcl::PointXYZI pt;
  int num_points = cloud_in.size();
  // atan2 (-pi, pi] 表示当前点与x轴正方向的夹角，并且从x轴逆时针的角度为正，顺时针的角度为负
  float start_ori = -atan2(cloud_in[0].y, cloud_in[1].x);
  float end_ori = -atan2(cloud_in[num_points - 1].y, cloud_in[num_points-1].x);
  if (end_ori - start_ori > 3 * M_PI) {
    end_ori -= 2 * M_PI;
  } else if (end_ori - start_ori < M_PI) {
    end_ori += 2 * M_PI;
  }

  bool half_pass = false;
  scan_pts->clear();
  scan_pts->resize(64);
  for (uint i = 0; i < cloud_in.size(); ++i) {
    pt.x = cloud_in[i].x;
    pt.y = cloud_in[i].y;
    pt.z = cloud_in[i].z;

    float angle = atan(pt.z / sqrt(pt.x * pt.x + pt.y * pt.y)) * 180 / M_PI;
    int line = 0;
    // HDL64 激光雷达以 8.83角度为界限。 以上为upper 的32线激光雷达，以下为lower的32线。
    // 上32的分辨率为0.333，下32的分辨率为0.5
    if (angle >= -8.83)
        line = int((2 - angle) * 3.0 + 0.5);
    else
        line = 64 / 2 + int((-8.83 - angle) * 2.0 + 0.5);

    // use [0 50]  > 50 remove outlies 
    if (angle > 2 || angle < -24.33 || line > 50 || line < 0)
    {
        continue;
    }

    // orientation of each point
    float ori = -atan2(pt.y, pt.x);
    if (!half_pass)
    { 
        if (ori < start_ori - M_PI / 2) // 起始角度为正的情况
        {
            ori += 2 * M_PI;
        }
        else if (ori > start_ori + M_PI * 3 / 2) // 起始角度为负的情况
        {
            ori -= 2 * M_PI;
        }

        if (ori - start_ori > M_PI) // 角度超过半圈 用end_ori来加角度
        {
            half_pass = true;
        }
    }
    else
    {
        ori += 2 * M_PI;
        if (ori < end_ori - M_PI * 3 / 2)
        {
            ori += 2 * M_PI;
        }
        else if (ori > end_ori + M_PI / 2)
        {
            ori -= 2 * M_PI;
        }
    }
    pt.intensity = line + scan_period / (end_ori - start_ori) * (ori - start_ori);
    scan_pts->at(line).push_back(pt);
  }
  return 0;
}

int extractFeatures(const std::vector<PointCloudT>& scan_pts, PointCloudT* edge_feature,
PointCloudT* secondary_edge_feature, PointCloudT* plane_feature, PointCloudT* secondary_plane_feature) {
  edge_feature->clear();
  plane_feature->clear();
  for (int line = 0; line <= 50; ++line) {
    const PointCloudT& line_pts = scan_pts[line];
    for (int i = 5; i <= line_pts.size() - 6; ++i) {
      float diff_x = line_pts[i - 5].x + line_pts[i - 4].x + line_pts[i - 3].x + line_pts[i - 2].x + line_pts[i - 1].x
      - 10 * line_pts[i].x + line_pts[i + 1].x + line_pts[i + 2].x + line_pts[i + 3].x + line_pts[i + 4].x +
      line_pts[i + 5].x;
      float diff_y = line_pts[i - 5].y + line_pts[i - 4].y + line_pts[i - 3].y + line_pts[i - 2].y + line_pts[i - 1].y
      - 10 * line_pts[i].y + line_pts[i + 1].y + line_pts[i + 2].y + line_pts[i + 3].y + line_pts[i + 4].y +
      line_pts[i + 5].y;
      float diff_z = line_pts[i - 5].z + line_pts[i - 4].z + line_pts[i - 3].z + line_pts[i - 2].z + line_pts[i - 1].z
      - 10 * line_pts[i].z + line_pts[i + 1].z + line_pts[i + 2].z + line_pts[i + 3].z + line_pts[i + 4].z +
      line_pts[i + 5].z;
      point_infos[line][i].curvature = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
      point_infos[line][i].label = 0;
      point_infos[line][i].neighbor_selected = 0;
      point_ind[line][i] = i;
    }
  }

  for (int line = 0; line <= 50; ++line) {
    uint num_line_pts = scan_pts[line].size();
  
    // 分为六个扇区
    for (int sec = 0; sec < 6; ++sec) {
      uint sp = 5 + (num_line_pts - 10) / 6 * sec;
      uint ep = 5 + (num_line_pts - 10) / 6 * (sec + 1) - 1;
      // 从小到大排列
      std::sort(point_ind[line] + sp, point_ind[line] + ep + 1, [&](uint lhs, uint rhs) {
        return point_infos[line][lhs].curvature < point_infos[line][rhs].curvature;
      });     
      // ROS_INFO("-------------------------------");
      // ROS_INFO_STREAM("Max curvature: " << point_infos[line][point_ind[line][ep]].curvature << ", Min curvature: " << point_infos[line][point_ind[line][sp]].curvature);
      uint edge_feature_count = 0;
      for (int i = ep; i >= sp; --i) {
        if (edge_feature_count >= 2) {
          break;
        }
        if (point_infos[line][i].neighbor_selected == true) {
          continue;
        }
        uint pt_ind = point_ind[line][i];
        edge_feature->push_back(scan_pts[line][pt_ind]);
        point_infos[line][i].label = 1;
        for (int j = pt_ind - 5; j < pt_ind + 6; ++j) {
          float diff_x = scan_pts[line][j].x - scan_pts[line][pt_ind].x;
          float diff_y = scan_pts[line][j].y - scan_pts[line][pt_ind].y;
          float diff_z = scan_pts[line][j].z - scan_pts[line][pt_ind].z;
          // 将离当前点sqrt 0.05m内的点排除，防止特征点过于密集
          if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z >= 0.05) {
            continue;
          }
          point_infos[line][j].neighbor_selected = true;
        }
        ++edge_feature_count;
        // ROS_INFO_STREAM("Edge feature curvature: " << point_infos[line][pt_ind].curvature);
      }

      uint plane_feature_count = 0;
      for (int i = sp; i <= ep; ++i) {
        if (plane_feature_count >= 4) {
          break;
        }
        if (point_infos[line][i].neighbor_selected == true) {
          continue;
        }
        uint pt_ind = point_ind[line][i];
        plane_feature->push_back(scan_pts[line][pt_ind]);
        for (int j = pt_ind - 5; j < pt_ind + 6; ++j) {
          float diff_x = scan_pts[line][j].x - scan_pts[line][pt_ind].x;
          float diff_y = scan_pts[line][j].y - scan_pts[line][pt_ind].y;
          float diff_z = scan_pts[line][j].z - scan_pts[line][pt_ind].z;
          // 将离当前点sqrt 0.05m内的点排除，防止特征点过于密集
          if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z >= 0.05) {
            continue;
          }
          point_infos[line][j].neighbor_selected = true;
        }
        point_infos[line][i].label = 2;
        ++plane_feature_count;
        // TODO(caoming): 从对ALOAM代码的输出中可以看出edge 的curvature比较大，Plane的curvarture都是在10-4数量级的。检查一下。
        // ROS_INFO_STREAM("Plane feature curvature: " << point_infos[line][pt_ind].curvature);
      }
    }
  }
}

ros::Publisher edge_feature_pub, plane_feature_pub, secondary_edge_feature_pub;
void lidarCB(const sensor_msgs::PointCloud2ConstPtr pc_msg) {
  PointCloudT pc_pcl;
  pcl::fromROSMsg(*pc_msg, pc_pcl);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(pc_pcl, pc_pcl, indices);
  
  std::vector<PointCloudT> scan_pts;
  splitScans(pc_pcl, &scan_pts);

  PointCloudT edge_feature_pc, secondary_edge_feature_pc, plane_feature_pc, secondary_plane_feature_pc;
  extractFeatures(scan_pts, &edge_feature_pc, &secondary_edge_feature_pc, &plane_feature_pc, &secondary_plane_feature_pc);

  sensor_msgs::PointCloud2 edge_feature_msg, plane_feature_msg;
  pcl::toROSMsg(edge_feature_pc, edge_feature_msg);
  pcl::toROSMsg(plane_feature_pc, plane_feature_msg);
  edge_feature_msg.header = pc_msg->header;
  plane_feature_msg.header = pc_msg->header;

  edge_feature_pub.publish(edge_feature_msg);
  plane_feature_pub.publish(plane_feature_msg);
}

int main(int argc, char *argv[])
{
  /* code */
  ros::init(argc, argv, "lidar_odometry");
  ros::NodeHandle nh;
  ros::Subscriber velo_sub = nh.subscribe(lidar_topic, 1, lidarCB);
  edge_feature_pub = nh.advertise<sensor_msgs::PointCloud2>("edge_feature", 1);
  plane_feature_pub = nh.advertise<sensor_msgs::PointCloud2>("plane_feature", 1);
  ros::spin();
  return 0;
}

