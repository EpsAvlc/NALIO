cmake_minimum_required(VERSION 3.0.2)
project(nalio)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "DEBUG")
# -g 包含调试信息 
# -Wall 启动所有的 warning
# -O3 三级优化 最高的优化选项
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nalio
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_executable(feature_extraction
  src/feature_extraction.cpp
)

target_link_libraries(feature_extraction
  ${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

add_executable(lidar_odometry
  src/lidar_odometry.cpp
)

target_link_libraries(lidar_odometry
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(kittiHelper src/kittiHelper.cpp)
target_link_libraries(kittiHelper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
