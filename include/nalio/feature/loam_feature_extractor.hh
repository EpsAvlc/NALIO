#ifndef NALIO_FEATURE_LOAM_FEATURE_EXTACTOR_HH__
#define NALIO_FEATURE_LOAM_FEATURE_EXTACTOR_HH__

#include "nalio/feature/feature_extractor.hh"

#include <pcl/point_types.h>

namespace nalio {

struct LOAMFeature : public Feature<pcl::PointXYZI> {
  using Ptr = std::shared_ptr<LOAMFeature>;
  struct Type {
    uint8_t val;
    enum : uint8_t { kLessSharp = 0, kSharp, kLessFlat, kFlat };
  };

  Type type;
  uint8_t line;
};

class LOAMFeatureExtractor
    : public FeatureExtractor<pcl::PointXYZI, pcl::PointXYZI> {
 public:
  using FeatureT = LOAMFeature;
  using PointCloudT = pcl::PointCloud<pcl::PointXYZI>;
  int extract(const pcl::PointCloud<pcl::PointXYZI> cloud_in,
              std::vector<Feature<pcl::PointXYZI>::Ptr>* features);

 private:
  struct PointInfo {
    float curvature;
    bool neighbor_selected;  // 是否其邻居点已被选作Feature点
    uint16_t ind;            // 原本的下标
    pcl::PointXYZI pt;
  };

  struct SharpCmp {
    bool operator()(PointInfo& lhs, PointInfo& rhs) {
      // 小顶堆
      return lhs.curvature > rhs.curvature;
    }
  };

  struct FlatCmp {
    bool operator()(PointInfo& lhs, PointInfo& rhs) {
      // 大顶堆
      return lhs.curvature < rhs.curvature;
    }
  };

  int splitScans(const PointCloudT& cloud_in,
                 std::vector<PointCloudT>* scan_pts);
  const double kScanPeriod = 0.1;
};
}  // namespace nalio

#endif
