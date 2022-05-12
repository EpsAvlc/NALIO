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

template <typename PointIn, typename FeatureT>
class FeatureExtractor {
 public:
  virtual int extract(
      const typename pcl::PointCloud<PointIn>::ConstPtr& cloud_in,
      std::vector<FeatureT>* features) = 0;
};
}  // namespace nalio

#endif
