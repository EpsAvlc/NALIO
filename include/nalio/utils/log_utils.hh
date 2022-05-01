#include <string>
#include <ros/console.h>

namespace nalio {
std::string cutParenthesesNTail(std::string&& pretty_func);

#ifndef __STR_FUNCTION__
#define __STR_FUNCTION__ nalio::cutParenthesesNTail(std::string(__PRETTY_FUNCTION__)).c_str()
#define ROS_ERROR_STREAM_FUNC(log) ROS_ERROR_STREAM(__STR_FUNCTION__ << ": " << log)
#endif
}
