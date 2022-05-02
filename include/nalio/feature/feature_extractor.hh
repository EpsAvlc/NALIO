#ifndef NALIO_FEATURE_FEATURE_EXTRACTOR_HH__
#define NALIO_FEATURE_FEATURE_EXTRACTOR_HH__

#include <memory>

#include <pcl/point_cloud.h>

namespace nalio {

template <typename PointT>
struct Feature {
  using Ptr = std::shared_ptr<Feature>;
  PointT pt;
};

template <typename PointIn, typename PointOut>
class FeatureExtractor {
 public:
  using FeatureT = Feature<PointOut>;
  virtual int extract(const pcl::PointCloud<PointIn> in_cloud, std::vector<typename Feature<PointOut>::Ptr>* features) = 0;
};
}  // namespace nalio

#endif
