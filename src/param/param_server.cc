#include "nalio/param/param_server.hh"

namespace nalio {

ros::NodeHandle& ParamServer::nh() { return nh_; };

ros::NodeHandle ParamServer::nh_;

}
