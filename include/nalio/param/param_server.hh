#ifndef NALIO_PARAM_PARAM_SERVER_HH__
#define NALIO_PARAM_PARAM_SERVER_HH__

#include <ros/ros.h>

namespace nalio {
class ParamServer {
 public:
  ParamServer();
  static ros::NodeHandle& nh();

 private:
  static ros::NodeHandle nh_;
};
}  // namespace nalio

#endif  // NALIO_WS_PARAM_SERVER_PARAM_SERVER_HH__
