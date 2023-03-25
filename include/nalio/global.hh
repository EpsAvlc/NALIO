#ifndef NALIO_GLOBAL_H
#define NALIO_GLOBAL_H

#include <ros/ros.h>
#include <exception>

#include "nalio/utils/log_utils.hh"

#define NALIO_EXPECT_TRUE(expr) \
  if (!(expr)) throw std::runtime_error(#expr " is false")
#define NALIO_STATIC_EXPECT_TRUE(expr) static_assert(expr, #expr " is false")


#define NALIO_MAKE_TYPE(T) using Ptr = std::unique_ptr<T>;

#endif  // UNOS_NALIO_GLOBAL_H
