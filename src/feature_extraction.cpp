/*
 * Created on Sat Dec 25 2021
 *
 * Copyright (c) 2021
 *
 * Author: EpsAvlc
 */

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

const std::string lidar_topic = "/velodyne_points";
const double scan_period = 0.1;

using PointCloudT = pcl::PointCloud<pcl::PointXYZI>;

int splitScans(const PointCloudT& cloud_in, std::vector<PointCloudT>& scan_pts) {
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
  scan_pts.clear();
  scan_pts.resize(64);
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
    scan_pts[line].push_back(pt);
  }
  return 0;
}

int extractFeatures(const std::vector<PointCloudT>& scan_pts) {

}

void lidarCB(const sensor_msgs::PointCloud2ConstPtr pc_msg) {
  PointCloudT pc_pcl;
  pcl::fromROSMsg(*pc_msg, pc_pcl);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(pc_pcl, pc_pcl, indices);
  
  std::vector<PointCloudT> scan_pts;
  splitScans(pc_pcl, scan_pts);
}

int main(int argc, char *argv[])
{
  /* code */
  ros::init(argc, argv, "lidar_odometry");
  ros::NodeHandle nh;
  ros::Subscriber velo_sub = nh.subscribe(lidar_topic, 1, lidarCB);
  ros::spin();
  return 0;
}

