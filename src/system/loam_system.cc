#include "nalio/system/loam_system.hh"

void nalio::LOAMSystem::feedData(const DataPackage& data) {
  propagate();
}
