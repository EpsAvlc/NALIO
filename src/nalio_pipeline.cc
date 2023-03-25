#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "nalio/factory/factory.hh"
#include "nalio/global.hh"
#include "nalio/nailo.hh"
#include "nalio/utils/log_utils.hh"
#include "nalio/visualizer/visualizer.hh"

nalio::System::Ptr p_system;
nalio::Dataset::Ptr p_dataset;
nalio::Visualizer::Ptr p_visualizer;

std::string system_name = "LOAMSystem";
std::string visualizer_name = "LOAMVisualizer";
std::string dataset_name = "KITTIDataset";

void dataCallback(const datahub::MessagePackage& package) {
  NLOG_INFO("Enter callback");
  auto vis_info = p_visualizer->createVisInfo();
  p_system->feedData(package, vis_info);
  p_visualizer->feed(vis_info);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "nalio_pipeline");

  p_system = nalio::Factory<nalio::System>::produce_unique(system_name);
  p_system->init();

  p_visualizer = nalio::Factory<nalio::Visualizer>::produce_unique(visualizer_name);

  p_dataset = nalio::Factory<nalio::Dataset>::produce_unique(dataset_name);
  NALIO_EXPECT_TRUE(p_dataset->init(true));
  p_dataset->registerCallback(dataCallback);

  p_visualizer->spin();
  return 0;
}
