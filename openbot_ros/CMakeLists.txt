cmake_minimum_required(VERSION 3.8)
project(openbot_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(openbot REQUIRED)
find_package(openbot_ros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(nav_msgs REQUIRED)
# find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
)

# Library
add_library(${PROJECT_NAME}
  src/msg_conversion.cpp
  src/node.cpp
  src/node_constants.cpp
  src/node_options.cpp
  src/ros_log_sink.cpp
  src/sensor_bridge.cpp
  src/tf_bridge.cpp
  src/time_conversion.cpp
  src/urdf_reader.cpp
)

set(dependencies
  builtin_interfaces
  openbot
  openbot_ros_msgs
  geometry_msgs
  nav_msgs
  # pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_msgs
  tf2_ros
  visualization_msgs
  rosbag2_cpp
  rosbag2_storage
  urdf
  urdfdom
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} openbot ${PCL_LIBRARIES})

# Executables
add_executable(openbot_node src/node_main.cpp)
target_link_libraries(openbot_node ${PROJECT_NAME})
ament_target_dependencies(openbot_node ${dependencies})

if($ENV{ROS_DISTRO} MATCHES "humble" OR $ENV{ROS_DISTRO} MATCHES "iron")
  target_compile_definitions(${PROJECT_NAME} PRIVATE PRE_JAZZY_SERIALIZED_BAG_MSG_FIELD_NAME)
  # target_compile_definitions(cartographer_rosbag_validate PRIVATE PRE_JAZZY_SERIALIZED_BAG_MSG_FIELD_NAME)
endif()

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(TARGETS
#   cartographer_node
#   cartographer_occupancy_grid_node
#   cartographer_offline_node
#   cartographer_assets_writer
#   cartographer_pbstream_map_publisher
#   cartographer_pbstream_to_ros_map
#   cartographer_rosbag_validate
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY configuration_files urdf launch
  DESTINATION share/${PROJECT_NAME}/
)

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
