/*
 * Created on Sun Jan 09 2022
 *
 * Copyright (c) 2022 
 *
 * Author: EpsAvlc
 */

#include <iostream>
#include <list>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

std::list<sensor_msgs::PointCloud2ConstPtr> edge_feature_msg_list, plane_feature_msg_list;
std::mutex edge_feature_mutex, plane_feature_mutex;

using PointCloudT = pcl::PointCloud<pcl::PointXYZI>;

void EdgeFeatureCB(const sensor_msgs::PointCloud2ConstPtr& edge_feature_msg) {
  std::lock_guard<std::mutex> lock(edge_feature_mutex);
  edge_feature_msg_list.push_back(edge_feature_msg);
}

void PlaneFeatureCB(const sensor_msgs::PointCloud2ConstPtr& plane_feature_msg) {
  std::lock_guard<std::mutex> lock(plane_feature_mutex);
  plane_feature_msg_list.push_back(plane_feature_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_odometry");
  ros::NodeHandle nh;
  ros::Subscriber edge_feature_sub = nh.subscribe<sensor_msgs::PointCloud2>("edge_feature", 1, EdgeFeatureCB);
  ros::Subscriber plane_feature_sub = nh.subscribe<sensor_msgs::PointCloud2>("plane_feature", 1, PlaneFeatureCB);

  ros::Rate rate(20);
  while (ros::ok()) {
    ros::spinOnce();
    if (!edge_feature_msg_list.empty() && !plane_feature_msg_list.empty()) {
      if (edge_feature_msg_list.front()->header.stamp != plane_feature_msg_list.front()->header.stamp) {
        ROS_ERROR_STREAM("Unsync msg");
        ros::shutdown();
      } else {
        PointCloudT edge_feature_pc, plane_feature_pc;
        pcl::fromROSMsg(*edge_feature_msg_list.front(), edge_feature_pc);
        pcl::fromROSMsg(*plane_feature_msg_list.front(), plane_feature_pc);
        edge_feature_msg_list.pop_front();
        plane_feature_msg_list.pop_front();
      }
    }
    rate.sleep();
  }
}
