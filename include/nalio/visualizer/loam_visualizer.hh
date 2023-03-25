#ifndef VISUALIZER_LOAM_VISUALIZER_HH__
#define VISUALIZER_LOAM_VISUALIZER_HH__

#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "nalio/factory/factory.hh"
#include "nalio/feature/loam_feature_extractor.hh"
#include "nalio/global.hh"
#include "nalio/utils/transformation.hh"
#include "nalio/visualizer/visualizer.hh"

namespace nalio {
struct LOAMVisInfo : public VisInfo {
 public:
  NALIO_MAKE_TYPE(LOAMVisInfo);

  LOAMFeaturePackage::Ptr p_feat_pkg;
  TransformationD body2map;
  TransformationD body2odom;
};

REGISTER_NALIO(VisInfo, LOAMVisInfo, "LOAMVisInfo");

class LOAMVisualizer : public Visualizer {
 public:
  NALIO_MAKE_TYPE(LOAMVisualizer);
  LOAMVisualizer();

  void feed(VisInfo::Ptr const& vis_info) override;
  void spin() override;
  VisInfo::Ptr createVisInfo() override;

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_odom_path_, pub_map_path_;
  nav_msgs::Path path_odom_, path_map_;
};

REGISTER_NALIO(Visualizer, LOAMVisualizer, "LOAMVisualizer");
}  // namespace nalio

#endif  //!__LOAM_VISUALIZER__H__
