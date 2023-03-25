#ifndef UTILS_NALIO_CONVERSION_HH__
#define UTILS_NALIO_CONVERSION_HH__

#include <geometry_msgs/PoseStamped.h>
#include "nalio/utils/transformation.hh"
#include "ros/time.h"

namespace nalio {

template <class T>
geometry_msgs::PoseStamped transform2GeometryPose(Transformation<T> const& trans,
                                                  std::string const& frame_id = "boot") {
  geometry_msgs::PoseStamped ret;
  ret.header.frame_id = frame_id;
  if (trans.timestamp > 0) {
    ret.header.stamp.fromSec(trans.timestamp);
  }

  ret.pose.position.x = trans.translation.x();
  ret.pose.position.y = trans.translation.y();
  ret.pose.position.z = trans.translation.z();

  ret.pose.orientation.x = trans.rotation.x();
  ret.pose.orientation.y = trans.rotation.y();
  ret.pose.orientation.z = trans.rotation.z();
  ret.pose.orientation.w = trans.rotation.w();

  return ret;
}

};  // namespace nalio

#endif
