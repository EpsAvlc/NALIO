cmake_minimum_required(VERSION 3.0.2)
project(nalio)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "DEBUG")
# -g 包含调试信息 
# -Wall 启动所有的 warning
# -O3 三级优化 最高的优化选项
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosunit
  rostest
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

# see here: https://github.com/PointCloudLibrary/pcl/issues/3680
# when this is fixed, we can remove the following 3 lines.
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nalio
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/include/eigen3
)

######################## DEFINITIONS #############
add_definitions(-DNALIO_DEBUG)

######################## LIBS ####################
add_library(${PROJECT_NAME}_utils
  src/utils/log_utils.cc
)
target_link_libraries(${PROJECT_NAME}_utils
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_param
  src/param/param_server.cc
)
target_link_libraries(${PROJECT_NAME}_param
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_state
  src/state/loam_state.cc
)
target_link_libraries(${PROJECT_NAME}_state
)

add_library(${PROJECT_NAME}_feature
  src/feature/loam_feature_extractor.cc
)
target_link_libraries(${PROJECT_NAME}_feature
	${PCL_LIBRARIES}
)

add_library(${PROJECT_NAME}_propagator
  src/propagator/linear_propagator.cc
)
target_link_libraries(${PROJECT_NAME}_propagator
  ${PROJECT_NAME}_utils
)

add_library(${PROJECT_NAME}_system
  src/system/loam_system.cc
)
target_link_libraries(${PROJECT_NAME}_system
	${PCL_LIBRARIES}
  ${PROJECT_NAME}_propagator
)


add_library(${PROJECT_NAME}_data
  src/data/kitti_dataset.cc
)
target_link_libraries(${PROJECT_NAME}_data
	${PCL_LIBRARIES}
)
####################### Nodes ####################
add_executable(${PROJECT_NAME}_pipeline
  src/nalio_pipeline.cc
)
target_link_libraries(${PROJECT_NAME}_pipeline
  ${PROJECT_NAME}_feature
  ${PROJECT_NAME}_utils
  ${PROJECT_NAME}_system
  ${PROJECT_NAME}_data
  ${PROJECT_NAME}_state
)

# add_executable(feature_extraction
#   src/feature_extraction.cpp
# )
# target_link_libraries(feature_extraction
#   ${catkin_LIBRARIES}
# 	${PCL_LIBRARIES}
# )

# add_executable(lidar_odometry
#   src/lidar_odometry.cpp
# )

# target_link_libraries(lidar_odometry
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )

####################### Tests ####################
if(CATKIN_ENABLE_TESTING)
catkin_add_gtest(${PROJECT_NAME}_test
  tests/tests.cc
)
target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}_state
  ${PROJECT_NAME}_propagator
  ${PROJECT_NAME}_data
  ${PROJECT_NAME}_system
  ${catkin_LIBRARIES}
  gtest_main
)
endif()

add_executable(kittiHelper src/kittiHelper.cpp)
target_link_libraries(kittiHelper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
