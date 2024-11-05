cmake_minimum_required(VERSION 3.8)
project(openbot_ros_simulator)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


# We specifically don't turn on CMAKE_AUTOMOC, since it generates one huge
# mocs_compilation.cpp file that takes a lot of memory to compile.  Instead
# we create individual moc files that can be compiled separately.
set(openbot_rviz_plugins_headers_to_moc
  include/openbot_ros_simulator/goal_tool.hpp
  include/openbot_ros_simulator/goal_pose_updater.hpp
  include/openbot_ros_simulator/goal_common.hpp
)
foreach(header "${openbot_rviz_plugins_headers_to_moc}")
  qt5_wrap_cpp(openbot_rviz_plugins_moc_files "${header}")
endforeach()

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/goal_tool.cpp
  ${openbot_rviz_plugins_moc_files}
)

target_include_directories(${library_name} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name} PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_default_plugins::rviz_default_plugins
  rviz_rendering::rviz_rendering
  ${std_msgs_TARGETS}
  tf2_geometry_msgs::tf2_geometry_msgs
  ${visualization_msgs_TARGETS}
)
target_link_libraries(${library_name} PRIVATE
  ament_index_cpp::ament_index_cpp
  pluginlib::pluginlib
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  Qt5
  rclcpp
  rclcpp_action
  rviz_common
  rviz_default_plugins
  rviz_rendering
  std_msgs
  tf2_geometry_msgs
  visualization_msgs
)

ament_package()