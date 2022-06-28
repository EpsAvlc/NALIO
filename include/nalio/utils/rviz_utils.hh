#ifndef NALIO_UTILS_RVIZ_UTILS_HH
#define NALIO_UTILS_RVIZ_UTILS_HH

#include <string>

#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>

namespace nalio {
namespace rviz_utils {
visualization_msgs::MarkerArray putTexts(
    const std::vector<std::string>& texts,
    const std::vector<Eigen::Vector3d>& positions,
    const std::string& frame_id, const double size = 0.15) {
  visualization_msgs::MarkerArray ret;
  for (size_t ti = 0; ti < texts.size(); ++ti) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "nalio";
    marker.id = ti;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.z = size;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.pose.position.x = positions[ti].x();
    marker.pose.position.y = positions[ti].y();
    marker.pose.position.z = positions[ti].z();
    marker.pose.orientation.w = 1;
    marker.text = texts[ti];

    ret.markers.push_back(marker);
  }
  return ret;
}
}  // namespace rviz_utils
}  // namespace nalio
#endif  // NALIO_UTILS_RVIZ_UTILS_HH
