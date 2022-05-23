#ifndef NALIO_FEATURE_FEATURE_EXTRACTOR_HH__
#define NALIO_FEATURE_FEATURE_EXTRACTOR_HH__

#include <memory>

#include <pcl/point_cloud.h>

namespace nalio {

struct FeaturePackage {
  using Ptr = std::shared_ptr<FeaturePackage>;
};

template <typename PointIn, typename FeaturePackage>
class FeatureExtractor {
 public:
  virtual bool extract(
      const typename pcl::PointCloud<PointIn>::ConstPtr& cloud_in,
      FeaturePackage* features) = 0;
};
}  // namespace nalio

#endif
