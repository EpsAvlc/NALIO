#ifndef NALIO_UTILS_GRID_MAP_HH__
#define NALIO_UTILS_GRID_MAP_HH__

#include <Eigen/Core>
#include <map>

namespace nalio {
template <typename T>
class GridMap {
 public:
  GridMap(const double resolution) : resolution_(resolution) {}

  const T& queryConst(const Eigen::Vector3d& position) const {
    Eigen::Vector3i ind = positionToIndex(position);
    return queryConst(ind);
  }

  T& query(const Eigen::Vector3d& position) {
    Eigen::Vector3i ind = positionToIndex(position);
    return query(ind);
  }

  void insert(const Eigen::Vector3d& position, const T& val) {
    Eigen::Vector3i ind = positionToIndex(position);
    insert(ind, val);
  }

 protected:
  Eigen::Vector3i positionToIndex(const Eigen::Vector3d& position) {
    // TODO: need to check it;
    Eigen::Vector3i ret = position / resolution_;
    return ret;
  }

  const T& queryConst(const Eigen::Vector3i& index) const {
    return map_[index.x()][index.y()][index.z()];
  }

  T& query(const Eigen::Vector3i& index) {
    return map_[index.x()][index.y()][index.z()];
  }

  void insert(const Eigen::Vector3i& index, const T& val) {
    map_[index.x()][index.y()][index.z()] = val;
  }

  std::map<int, std::map<int, std::map<int, T>>> map_;
  double resolution_;
};
}  // namespace nalio

#endif  // NALIO_UTILS_GRID_MAP_HH__
