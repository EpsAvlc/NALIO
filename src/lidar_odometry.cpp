/*
 * Created on Sun Jan 09 2022
 *
 * Copyright (c) 2022 
 *
 * Author: EpsAvlc
 */

#include <iostream>
#include <list>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <mutex>

std::list<sensor_msgs::PointCloud2ConstPtr> edge_feature_msg_list, plane_feature_msg_list;
std::mutex edge_feature_mutex, plane_feature_mutex;

void EdgeFeatureCB(const sensor_msgs::PointCloud2ConstPtr& edge_feature_msg) {
  std::lock_guard<std::mutex> lock(edge_feature_mutex);
  edge_feature_msg_list.push_back(edge_feature_msg);
}

void PlaneFeatureCB(const sensor_msgs::PointCloud2ConstPtr& plane_feature_msg) {
  std::lock_guard<std::mutex> lock(plane_feature_mutex);
  plane_feature_msg_list.push_back(plane_feature_msg)
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_odometry");
  ros::NodeHandle nh;
  ros::Subscriber edge_feature_sub = nh.subscribe<sensor_msgs::PointCloud2>("edge_feature", 1, EdgeFeatureCB);
  ros::Subscriber plane_feature_sub = nh.subscribe<sensor_msgs::PointCloud2>("plane_feature", 1, PlaneFeatureCB);
}
