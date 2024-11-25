################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(openbot_simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(message_filters REQUIRED)


################################################################################
# Build
################################################################################
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${TINYXML2_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${TINYXML2_INCLUDE_DIR}
)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "message_filters"
  "angles"
)


# openbot_drive
set(EXEC_NAME "openbot_drive")
add_executable(${EXEC_NAME} src/openbot_simulator/openbot_drive.cpp)
ament_target_dependencies(${EXEC_NAME} ${dependencies})

# fake_localization
add_executable(fake_localization src/fake_localization/fake_localization.cpp)
ament_target_dependencies(fake_localization ${dependencies})

# add_library(obstacle1 SHARED models/openbot_dqn_world/obstacle_plugin/obstacle1.cc)
# target_link_libraries(obstacle1 ${GAZEBO_LIBRARIES})

# add_library(obstacle2 SHARED models/openbot_dqn_world/obstacle_plugin/obstacle2.cc)
# target_link_libraries(obstacle2 ${GAZEBO_LIBRARIES})

# add_library(obstacles SHARED models/openbot_dqn_world/obstacle_plugin/obstacles.cc)
# target_link_libraries(obstacles ${GAZEBO_LIBRARIES})

################################################################################
# Install
################################################################################

install(TARGETS 
  ${EXEC_NAME}
  fake_localization
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  ${EXEC_NAME}
  fake_localization
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch models params rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_package()
