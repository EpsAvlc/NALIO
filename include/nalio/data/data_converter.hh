#ifndef NALIO_WS_DATA_DATA_CONVERTER_HH__
#define NALIO_WS_DATA_DATA_CONVERTER_HH__

#include <memory>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include "nalio/data/message.hh"

namespace nalio {
void toNalioHeader(const std_msgs::Header& ros_header, nalio::Header* std_header);

std::shared_ptr<nalio::Message> toNalioMessage(const sensor_msgs::ImuConstPtr& imu_msg, const std::string& name);

std::shared_ptr<nalio::Message> toNalioMessage(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg, const std::string& name);
}  // namespace nalio

#endif  // NALIO_WS_DATA_DATA_CONVERTER_HH__
