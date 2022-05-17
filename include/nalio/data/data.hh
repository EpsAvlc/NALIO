#ifndef NALIO_DATA_DATA_HH__
#define NALIO_DATA_DATA_HH__

#include <vector>
#include <memory>

#define PCL_NO_PRECOMPILE
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct Data {
  using Ptr = std::shared_ptr<Data>;
};

struct NalioPoint {
  PCL_ADD_POINT4D;  // preferred way of adding a XYZ+padding
  float intensity;
  float rel_time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(NalioPoint,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      float, intensity,
                                      intensity)(float, rel_time, rel_time))

namespace nalio {
struct IMUData : public Data {};

struct DataPackage {
  using Ptr = std::shared_ptr<DataPackage>;
  using PointCloudT = pcl::PointCloud<NalioPoint>;
  PointCloudT::Ptr lidar_meas;
  std::vector<IMUData> imu_meas;
};

}  // namespace nalio

#endif  // NALIO_DATA_DATA_HH__
