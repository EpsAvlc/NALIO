#ifndef NALIO_DATA_NALIO_DATA_HH__
#define NALIO_DATA_NALIO_DATA_HH__

#include <memory>
#include <vector>

#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/impl/accumulators.hpp>

#include "datahub/data.hh"

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

namespace pcl {
namespace detail {
struct AccumulatorLine {
  mutable std::vector<int32_t> lines;
  AccumulatorLine()  { lines.reserve(100); }
  template <typename PointT>
  void add(const PointT& t) {
    lines.push_back(t.line);
  }

  template <typename PointT>
  void get(PointT& t, size_t n) const {
    std::sort(lines.begin(), lines.end());
    // mediam number.
    t.line = lines[lines.size() / 2];
  }
};

template <>
struct Accumulators<NalioPoint> {
  // A Fusion vector with accumulator types that are compatible with given
  // point types
  typedef typename boost::fusion::result_of::as_vector<
      boost::mpl::vector<AccumulatorXYZ, AccumulatorLine>>::type type;
};
}  // namespace detail
}  // namespace pcl

namespace nalio {

struct IMUData : public datahub::Data {
  using Ptr = std::shared_ptr<IMUData>;
  //* angular velocity;
  Eigen::Vector3d w;
  //* linear velocity;
  Eigen::Vector3d a;
};

struct PointCloudData : public datahub::Data {
  using Ptr = std::shared_ptr<PointCloudData>;
  pcl::PointCloud<NalioPoint>::Ptr point_cloud;
};

}  // namespace nalio

#endif  // NALIO_DATA_DATA_HH__
