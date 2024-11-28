cmake_minimum_required(VERSION 3.8)
project(surfel_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(image_transport REQUIRED)
find_package(visualization_msgs REQUIRED)

# include
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# dependency
set(dependencies 
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  pcl_ros
  pcl_conversions
  visualization_msgs
  image_transport)

# sources
set(SRCS
  src/surfel_map.cpp
  src/fusion_functions.cpp
)

set(library_name surfel_map)
add_library(${library_name} SHARED ${SRCS})
ament_target_dependencies(${library_name} ${dependencies})
target_link_libraries(${library_name} ${OpenCV_LIBRARIES})
target_link_libraries(${library_name} ${Boost_LIBRARIES})
target_link_libraries(${library_name} ${dlib_LIBRARIES})
target_link_libraries(${library_name} ${PCL_LIBRARIES})

# executable
set(executable_name surfel_map_node)
add_executable(${executable_name} ${SRCS} src/ros_node.cpp)
ament_target_dependencies(${executable_name} ${dependencies})
target_link_libraries(${executable_name} ${OpenCV_LIBRARIES})
target_link_libraries(${executable_name} ${Boost_LIBRARIES})
target_link_libraries(${executable_name} ${dlib_LIBRARIES})
target_link_libraries(${executable_name} ${PCL_LIBRARIES})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${library_name}
  ${executable_name}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
