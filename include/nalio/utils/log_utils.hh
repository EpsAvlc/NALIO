#ifndef NALIO_UTILS_LOG_UTILS_HH__
#define NALIO_UTILS_LOG_UTILS_HH__

#include <ros/console.h>
#include <string>

namespace nalio {
std::string cutParenthesesNTail(std::string&& pretty_func);

}

#define __STR_FUNCTION__ \
  nalio::cutParenthesesNTail(std::string(__PRETTY_FUNCTION__)).c_str()
#define NLOG_ERROR_STREAM(log) ROS_ERROR_STREAM(__STR_FUNCTION__ << ": " << log)
#define NLOG_INFO_STREAM(log) ROS_INFO_STREAM(__STR_FUNCTION__ << ": " << log)

#define NLOG_TEMPLATE(LEVEL, FMT, ...) \
  ROS_##LEVEL ("%s " FMT, __STR_FUNCTION__, ##__VA_ARGS__)
#define NLOG_INFO(...) NLOG_TEMPLATE(INFO, ##__VA_ARGS__);
#define NLOG_WARN(...) NLOG_TEMPLATE(WARN, ##__VA_ARGS__);
#define NLOG_ERROR(...) NLOG_TEMPLATE(ERROR, ##__VA_ARGS__);

#endif  // NALIO_UTILS_LOG_UTILS_HH__
