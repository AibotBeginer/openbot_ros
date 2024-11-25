# Copyright 2024 The OpenRobotic Beginner Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(openbot_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(openbot REQUIRED)
include("${OPENBOT_CMAKE_DIR}/functions.cmake")
set(BUILD_SHARED_LIBS OFF)
option(BUILD_GRPC "build features that require Openbot_ros_msgs gRPC support" false)

google_initialize_openbot_project()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(openbot REQUIRED)
find_package(openbot_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(grid_map_octomap REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(motion_msgs REQUIRED)


include(FindPkgConfig)

if(DEFINED urdfdom_headers_VERSION)
  if(${urdfdom_headers_VERSION} GREATER 0.4.1)
    add_definitions(-DURDFDOM_HEADERS_HAS_SHARED_PTR_DEFS)
  endif()
endif()

include_directories(
  include
  "."
  ${openbot_INCLUDE_DIRS}
  ${octomap_INCLUDE_DIRS}
  ${grid_map_octomap_INCLUDE_DIRS}
  ${octomap_ros_INCLUDE_DIRS}
  ${grid_map_ros_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${openbot_msgs_INCLUDE_DIRS}
  ${shape_msgs_INCLUDE_DIRS}
  ${vision_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${motion_msgs_INCLUDE_DIRS}

)

set(ALL_SRCS
  openbot_ros/global_planner.cpp
  openbot_ros/local_planner.cpp
  openbot_ros/pointcloud_to_occupancy_grid.cpp
  # openbot_ros/messages_conversion/proto_msgs_to_ros.cpp
  # openbot_ros/messages_conversion/ros_msgs_to_proto.cpp
  openbot_ros/messages_conversion/builtin_interfaces_converter.cpp
  openbot_ros/messages_conversion/geometry_msgs_converter.cpp
  openbot_ros/messages_conversion/nav_msgs_converter.cpp
  openbot_ros/messages_conversion/sensor_msgs_converter.cpp
  openbot_ros/messages_conversion/shape_msgs_converter.cpp
  openbot_ros/messages_conversion/std_msgs_converter.cpp
  openbot_ros/messages_conversion/vision_msgs_converter.cpp
  openbot_ros/visualization/global_planner_visualization.cpp
  openbot_ros/visualization/local_planner_visualization.cpp
  openbot_ros/msg_conversion.cpp
  openbot_ros/map_generator.cpp
  openbot_ros/mockamap_generator.cpp
  openbot_ros/node.cpp
  openbot_ros/node_constants.cpp
  openbot_ros/node_options.cpp
  openbot_ros/ros_log_sink.cpp
  openbot_ros/sensor_bridge.cpp
  openbot_ros/tf_bridge.cpp
  openbot_ros/time_conversion.cpp
  openbot_ros/urdf_reader.cpp
)


set(dependencies
  builtin_interfaces
  openbot_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  shape_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_msgs
  tf2_ros
  visualization_msgs
  vision_msgs
  octomap
  octomap_ros
  grid_map_octomap
  grid_map_ros
  pcl_ros
  motion_msgs
)

add_library(${PROJECT_NAME} ${ALL_SRCS})
target_link_libraries(${PROJECT_NAME} PUBLIC openbot)

add_subdirectory("openbot_ros")

# PCL
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})

set(BLACKLISTED_PCL_DEFINITIONS " -march=native -msse4.2 -mfpmath=sse ")
foreach(DEFINITION ${PCL_DEFINITIONS})
  list (FIND BLACKLISTED_PCL_DEFINITIONS "${DEFINITION}" DEFINITIONS_INDEX)
  if (${DEFINITIONS_INDEX} GREATER -1)
    continue()
  endif()
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}")
endforeach()

# URDFDOM
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${urdfdom_headers_INCLUDE_DIRS})

if($ENV{ROS_DISTRO} MATCHES "humble" OR $ENV{ROS_DISTRO} MATCHES "iron")
  target_compile_definitions(${PROJECT_NAME} PRIVATE PRE_JAZZY_SERIALIZED_BAG_MSG_FIELD_NAME)
  # target_compile_definitions(cartographer_rosbag_validate PRIVATE PRE_JAZZY_SERIALIZED_BAG_MSG_FIELD_NAME)
endif()

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

install(DIRECTORY configuration_files urdf launch rviz params
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()