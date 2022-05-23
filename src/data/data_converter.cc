#include "nalio/data/data_converter.hh"

#include <pcl_conversions/pcl_conversions.h>

namespace nalio {
nalio::Header toNalioHeader(const std_msgs::Header& ros_header) {
  nalio::Header nalio_header;
  nalio_header.frame_id = ros_header.frame_id;
  nalio_header.timestamp = ros_header.stamp.toNSec() / 1000;
  return nalio_header;
}

std::shared_ptr<nalio::Message> toNalioMessage(
    const sensor_msgs::ImuConstPtr& imu_msg, const std::string& name) {
  std::shared_ptr<Message> ret(new Message);
  std::shared_ptr<IMUData> imu_data(new IMUData);
  imu_data->w.x() = imu_msg->angular_velocity.x;
  imu_data->w.y() = imu_msg->angular_velocity.y;
  imu_data->w.z() = imu_msg->angular_velocity.z;
  imu_data->a.x() = imu_msg->linear_acceleration.x;
  imu_data->a.y() = imu_msg->linear_acceleration.y;
  imu_data->a.z() = imu_msg->linear_acceleration.z;
  ret->data = imu_data;
  ret->header = toNalioHeader(imu_msg->header);
  ret->name = name;
  ret->type.val = Message::Type::kImu;
  return ret;
}

std::shared_ptr<nalio::Message> toNalioMessage(
    const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg,
    const std::string& name) {
  std::shared_ptr<Message> ret(new Message);
  std::shared_ptr<PointCloudData> point_cloud_data(new PointCloudData);
  point_cloud_data->point_cloud.reset(new pcl::PointCloud<NalioPoint>);

  pcl::PointCloud<pcl::PointXYZI> ori_pc;
  pcl::fromROSMsg(*point_cloud_msg, ori_pc);

  point_cloud_data->point_cloud->resize(ori_pc.size());
  for (size_t pi = 0; pi < ori_pc.size(); ++pi) {
    memcpy((*point_cloud_data->point_cloud)[pi].data, ori_pc[pi].data,
           sizeof(float) * 4);
    (*point_cloud_data->point_cloud)[pi].intensity = ori_pc[pi].intensity;
  }

  ret->data = std::static_pointer_cast<Data>(point_cloud_data);
  ret->header = toNalioHeader(point_cloud_msg->header);
  ret->name = name;
  ret->type.val = Message::Type::kLidar;
  return ret;
}

}  // namespace nalio
