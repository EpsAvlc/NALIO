#ifndef NALIO_GLOBAL_H
#define NALIO_GLOBAL_H

#include <exception>
#include <ros/ros.h>

#include "nalio/utils/log_utils.hh"


#define NALIO_EXPECT_TRUE(expr) \
  if (!(expr)) throw std::runtime_error(#expr " is false")
#define NALIO_STATIC_EXPECT_TRUE(expr) static_assert(expr, #expr " is false")

#endif // UNOS_NALIO_GLOBAL_H
