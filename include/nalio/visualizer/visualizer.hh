#ifndef NALIO_VISUALIZER_VISUALIZER_HH__
#define NALIO_VISUALIZER_VISUALIZER_HH__

#include <memory>

namespace nalio {
class VisInfo {
 public:
  using Ptr = std::unique_ptr<VisInfo>;
};

class Visualizer {
 public:
  using Ptr = std::unique_ptr<Visualizer>;

  virtual void feed(VisInfo::Ptr const& vis_info) = 0;
  virtual void spin() = 0;
  virtual VisInfo::Ptr createVisInfo() = 0;
};

#define VIS_DOWN_CAST static_cast

};  // namespace nalio

#endif
