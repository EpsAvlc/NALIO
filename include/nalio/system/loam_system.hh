#ifndef NALIO_SYSTEM_LOAM_SYSTEM_HH__
#define NALIO_SYSTEM_LOAM_SYSTEM_HH__

#include "nalio/system/system.hh"
#include "nalio/feature/loam_feature_extractor.hh"

namespace nalio {
class LOAMSystem final : public system {
 public:

 private:
  Eigen::VectorXd state_;
};
}  // namespace nalio

#endif
