#ifndef NALIO_DATA_MESSAGE_HH__
#define NALIO_DATA_MESSAGE_HH__

#include <memory>

#include "nalio/data/data.hh"

namespace nalio {
struct Message {
  using Ptr = std::shared_ptr<Message>;
  int64_t timestamp;
  std::string name;
  std::shared_ptr<Data> data;
};
}

#endif // NALIO_WS_DATA_MESSAGE_HH__
