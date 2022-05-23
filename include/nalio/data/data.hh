#ifndef NALIO_DATA_DATA_HH__
#define NALIO_DATA_DATA_HH__

#include <memory>
#include <vector>

#define PCL_NO_PRECOMPILE
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct NalioPoint {
  PCL_ADD_POINT4D;  // preferred way of adding a XYZ+padding
  float intensity;
  float rel_time;
  uint16_t line;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(
    NalioPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        float, rel_time, rel_time)(uint16_t, line, line))

namespace nalio {

struct Data {
  using Ptr = std::shared_ptr<Data>;
  virtual ~Data() {}
};

struct IMUData : public Data {
  //* angular velocity;
  Eigen::Vector3d w;
  //* linear velocity;
  Eigen::Vector3d a;
};

struct PointCloudData : public Data {
  using Ptr = std::shared_ptr<PointCloudData>;
  pcl::PointCloud<NalioPoint>::Ptr point_cloud;
};

}  // namespace nalio

#endif  // NALIO_DATA_DATA_HH__
