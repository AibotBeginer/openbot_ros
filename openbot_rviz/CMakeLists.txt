cmake_minimum_required(VERSION 3.5)
project(openbot_rviz)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rcpputils)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

set(openbot_rviz_headers_to_moc
  include/openbot_rviz/pose_3d_tool.hpp
  include/openbot_rviz/goal_3d_new_tool.hpp
  include/openbot_rviz/goal_pose_updater.hpp
  include/openbot_rviz/goal_common.hpp
  include/openbot_rviz/goal_tool.hpp
  include/openbot_rviz/planning_panel.hpp
  include/openbot_rviz/detection_3d_array.hpp
  include/openbot_rviz/detection_3d.hpp
  include/openbot_rviz/bounding_box_3d.hpp
  include/openbot_rviz/bounding_box_3d_array.hpp
)

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/goal_tool.cpp
  src/planning_panel.cpp
  src/pose_3d_tool.cpp
  src/goal_3d_new_tool.cpp
  src/detection_3d_array.cpp
  src/detection_3d.cpp
  src/bounding_box_3d.cpp
  src/bounding_box_3d_array.cpp
  ${openbot_rviz_headers_to_moc}
)

set(dependencies
  class_loader
  geometry_msgs
  nav_msgs
  pluginlib
  Qt5
  rclcpp
  rcpputils
  rclcpp_lifecycle
  yaml_cpp_vendor
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
  std_msgs
  tf2_geometry_msgs
  vision_msgs
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_include_directories(${library_name} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

target_link_libraries(${library_name}
  rviz_common::rviz_common
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# TODO: Make this specific to this project (not rviz default plugins)
target_compile_definitions(${library_name} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include/
)

# Install Python executables
install(PROGRAMS
  samples/Detection3DArray.py
  samples/Detection3D.py
  samples/BoundingBox3D.py
  samples/BoundingBox3DArray.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch conf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Qt5
  rviz_common
  geometry_msgs
  map_msgs
  nav_msgs
  rclcpp
)

ament_package()
