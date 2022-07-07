#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "nalio/nailo.hh"

// #define USE_ALOAM_FEATURE

nalio::System::Ptr system_ptr;
nalio::Dataset::Ptr dataset_ptr;

#ifdef USE_ALOAM_FEATURE
std::list<pcl::PointCloud<pcl::PointXYZI>> flat_features, less_flat_features,
    sharp_features, less_sharp_features;

std::mutex flat_mutex, less_flat_mutex, sharp_mutex, less_sharp_mutex;

void flatFeatureCb(const sensor_msgs::PointCloud2ConstPtr& flat_msg) {
  std::lock_guard<std::mutex> lock(flat_mutex);
  pcl::PointCloud<pcl::PointXYZI> flat_cloud;
  pcl::fromROSMsg(*flat_msg, flat_cloud);
  flat_features.push_back(flat_cloud);
}

void sharpFeatureCb(const sensor_msgs::PointCloud2ConstPtr& sharp_msg) {
  std::lock_guard<std::mutex> lock(sharp_mutex);
  pcl::PointCloud<pcl::PointXYZI> sharp_cloud;
  pcl::fromROSMsg(*sharp_msg, sharp_cloud);
  sharp_features.push_back(sharp_cloud);
}

void lessFlatFeatureCb(const sensor_msgs::PointCloud2ConstPtr& less_flat_msg) {
  std::lock_guard<std::mutex> lock(less_flat_mutex);
  pcl::PointCloud<pcl::PointXYZI> less_flat_cloud;
  pcl::fromROSMsg(*less_flat_msg, less_flat_cloud);
  less_flat_features.push_back(less_flat_cloud);
}

void lessSharpFeatureCb(
    const sensor_msgs::PointCloud2ConstPtr& less_sharp_msg) {
  std::lock_guard<std::mutex> lock(less_sharp_mutex);
  pcl::PointCloud<pcl::PointXYZI> less_sharp_cloud;
  pcl::fromROSMsg(*less_sharp_msg, less_sharp_cloud);
  less_sharp_features.push_back(less_sharp_cloud);
}
#else
void dataCallback(const datahub::MessagePackage& package) {
  ROS_INFO_STREAM_FUNC("enter pipeline data cb");
  system_ptr->feedData(package);
}
#endif

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "nalio_pipeline");

  system_ptr = nalio::Factory<nalio::System>::produce_unique("LOAMSystem");
  system_ptr->init();

#ifdef USE_ALOAM_FEATURE
  ros::NodeHandle nh;
  ros::Subscriber flat_sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "laser_cloud_flat", 1, flatFeatureCb);
  ros::Subscriber less_flat_sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "laser_cloud_less_flat", 1, lessFlatFeatureCb);
  ros::Subscriber sharp_sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "laser_cloud_sharp", 1, sharpFeatureCb);
  ros::Subscriber less_sharp_sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "laser_cloud_less_sharp", 1, lessSharpFeatureCb);
  pcl::PointCloud<pcl::PointXYZI> flat_feat, less_flat_feat, sharp_feat,
      less_sharp_feat;
  pcl::PointCloud<NalioPoint>::Ptr flat_feat_nalio, less_flat_feat_nalio,
      sharp_feat_nalio, less_sharp_feat_nalio;
  while (ros::ok()) {
    ros::spinOnce();
    if (flat_features.size() == 0 || less_flat_features.size() == 0 ||
        sharp_features.size() == 0 || less_sharp_features.size() == 0) {
      // ROS_INFO_STREAM("Invalid input. flat size: "
      //                 << flat_features.size()
      //                 << ", less flat size: " << less_flat_features.size()
      //                 << ", sharp size: " << sharp_features.size()
      //                 << ", less sharp size: " << less_sharp_features.size());
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(flat_mutex);
      flat_feat.swap(flat_features.front());
      flat_features.pop_front();
      NalioPoint pt;
      flat_feat_nalio.reset(new pcl::PointCloud<NalioPoint>);
      for (size_t ii = 0; ii < flat_feat.size(); ++ii) {
        pt.x = flat_feat[ii].x;
        pt.y = flat_feat[ii].y;
        pt.z = flat_feat[ii].z;
        pt.line = static_cast<uint16_t>(flat_feat[ii].intensity);
        pt.rel_time = flat_feat[ii].intensity - pt.line;
        flat_feat_nalio->push_back(pt);
      }
    }
    {
      std::lock_guard<std::mutex> lock(less_flat_mutex);
      less_flat_feat.swap(less_flat_features.front());
      less_flat_features.pop_front();
      NalioPoint pt;
      less_flat_feat_nalio.reset(new pcl::PointCloud<NalioPoint>);
      for (size_t ii = 0; ii < less_flat_feat.size(); ++ii) {
        pt.x = less_flat_feat[ii].x;
        pt.y = less_flat_feat[ii].y;
        pt.z = less_flat_feat[ii].z;
        pt.line = static_cast<uint16_t>(less_flat_feat[ii].intensity);
        pt.rel_time = less_flat_feat[ii].intensity - pt.line;
        less_flat_feat_nalio->push_back(pt);
      }
    }
    {
      std::lock_guard<std::mutex> lock(sharp_mutex);
      sharp_feat.swap(sharp_features.front());
      sharp_features.pop_front();
      NalioPoint pt;
      sharp_feat_nalio.reset(new pcl::PointCloud<NalioPoint>);
      for (size_t ii = 0; ii < sharp_feat.size(); ++ii) {
        pt.x = sharp_feat[ii].x;
        pt.y = sharp_feat[ii].y;
        pt.z = sharp_feat[ii].z;
        pt.line = static_cast<uint16_t>(sharp_feat[ii].intensity);
        pt.rel_time = sharp_feat[ii].intensity - pt.line;
        sharp_feat_nalio->push_back(pt);
      }
    }
    {
      std::lock_guard<std::mutex> lock(less_sharp_mutex);
      less_sharp_feat.swap(less_sharp_features.front());
      less_sharp_features.pop_front();
      less_sharp_feat_nalio.reset(new pcl::PointCloud<NalioPoint>);
      NalioPoint pt;
      for (size_t ii = 0; ii < less_sharp_feat.size(); ++ii) {
        pt.x = less_sharp_feat[ii].x;
        pt.y = less_sharp_feat[ii].y;
        pt.z = less_sharp_feat[ii].z;
        pt.line = static_cast<uint16_t>(less_sharp_feat[ii].intensity);
        pt.rel_time = less_sharp_feat[ii].intensity - pt.line;
        less_sharp_feat_nalio->push_back(pt);
      }
    }
    nalio::LOAMFeaturePackage::Ptr feature_package(
        new nalio::LOAMFeaturePackage);
    feature_package->flat_cloud = flat_feat_nalio;
    feature_package->less_flat_cloud = less_flat_feat_nalio;
    feature_package->sharp_cloud = sharp_feat_nalio;
    feature_package->less_sharp_cloud = less_sharp_feat_nalio;

    ROS_INFO_STREAM_FUNC("flat_size: "
                         << flat_feat_nalio->size()
                         << ", less_flat_size: " << less_flat_feat_nalio->size()
                         << ", sharp size: " << sharp_feat_nalio->size()
                         << ", less_sharp size: "
                         << less_sharp_feat_nalio->size());
    static_cast<nalio::LOAMSystem&>(*system_ptr).feedData(feature_package);
  }
#else
  dataset_ptr = nalio::Factory<nalio::Dataset>::produce_shared("KITTIDataset");
  dataset_ptr->init(true);
  dataset_ptr->registerCallback(dataCallback);
  ros::Rate rate(30);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
#endif

  return 0;
}
