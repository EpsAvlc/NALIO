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

std::string lidar_topic = "";

std::vector<ros::Publisher> scan_pubs;

void lidarCB(const sensor_msgs::PointCloud2ConstPtr pc_msg) {
  pcl::PointCloud<pcl::PointXYZI> pc_pcl;
  pcl::fromROSMsg(*pc_msg, pc_pcl);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(pc_pcl, pc_pcl, indices);
  
  std::vector<sensor_msgs::PointCloud2> scan_msgs(16);
  std::vector<pcl::PointCloud<pcl::PointXYZI>> scan_pcs(16);

  int num_points = pc_pcl.size();
  pcl::PointXYZI pt;
  for (int i = 0; i < pc_pcl.size(); ++i) {
    pt.x = pc_pcl[i].x;
    pt.y = pc_pcl[i].y;
    pt.z = pc_pcl[i].z;

    float angle = atan(pt.z / sqrt(pt.x * pt.x + pt.y * pt.y)) * 180 / M_PI;
    int line = 0;
  }
}

int main(int argc, char *argv[])
{
  /* code */
  ros::init(argc, argv, "lidar_odometry");
  ros::NodeHandle nh;
  nh.subscribe(lidar_topic, 1, lidarCB);
  scan_pubs.resize(16);
  for (int i = 0; i < 16; ++i) {
    scan_pubs[i] = nh.advertise<sensor_msgs::PointCloud2>("scan_" + std::to_string(i), 1);
  }
  ros::spin();
  return 0;
}

