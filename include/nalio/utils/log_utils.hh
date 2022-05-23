#ifndef NALIO_UTILS_LOG_UTILS_HH__
#define NALIO_UTILS_LOG_UTILS_HH__

#include <ros/console.h>
#include <string>

namespace nalio {
std::string cutParenthesesNTail(std::string&& pretty_func);

}

#define __STR_FUNCTION__ \
  nalio::cutParenthesesNTail(std::string(__PRETTY_FUNCTION__)).c_str()
#define ROS_ERROR_STREAM_FUNC(log) \
  ROS_ERROR_STREAM(__STR_FUNCTION__ << ": " << log)
#define ROS_INFO_STREAM_FUNC(log) \
  ROS_INFO_STREAM(__STR_FUNCTION__ << ": " << log)

#endif  // NALIO_UTILS_LOG_UTILS_HH__
