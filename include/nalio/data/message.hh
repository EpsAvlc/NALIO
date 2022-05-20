#ifndef NALIO_DATA_MESSAGE_HH__
#define NALIO_DATA_MESSAGE_HH__

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "nalio/data/data.hh"

namespace nalio {
class TimeStamp {
 public:
  TimeStamp() {}
  TimeStamp(int64_t us) : us_(us) {}

  void operator=(const double us) { us_ = us; }

  double sec() { return us_ / 1000000.; }

  int64_t usec() { return us_; }

  bool operator<(const TimeStamp& rhs) { return us_ < rhs.us_; }

  bool operator>(const TimeStamp& rhs) { return us_ > rhs.us_; }

 private:
  // time stamp in micro seconds
  int64_t us_;
};

struct Header {
  TimeStamp timestamp;
  std::string frame_id;
};

struct Message {
  using Ptr = std::shared_ptr<Message>;
  Header header;
  std::string name;
  std::shared_ptr<Data> data;
  struct Type {
    uint8_t val;
    enum : uint8_t { kImu, kLidar };
  } type;
};

using MessagePackage = std::vector<std::vector<Message::Ptr>>;

}  // namespace nalio

#endif  // NALIO_WS_DATA_MESSAGE_HH__
