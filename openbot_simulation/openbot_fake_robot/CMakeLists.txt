################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(openbot_fake_robot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_msgs"
)


set(EXECUTABLES
  "turtlebot3_fake_node"
)

# Define the list of dependencies shared by both executables
set(DEPENDENCIES ${dependencies})

# Create executables and link dependencies
foreach(executable ${EXECUTABLES})
  add_executable(${executable} src/${executable}.cpp)
  ament_target_dependencies(${executable} ${DEPENDENCIES})
  
  # Install executables
  install(TARGETS ${executable}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

# find_package(ament_cmake_python REQUIRED)
# ament_python_install_package(${PROJECT_NAME}/../src/motion_control)

################################################################################
# Install
################################################################################

install(DIRECTORY launch param urdf maps models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_msgs)
ament_package()
