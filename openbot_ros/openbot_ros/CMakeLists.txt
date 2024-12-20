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

# Executables(node_main)
set(node_executable_name openbot_node)
add_executable(${node_executable_name} node_main.cpp)
target_link_libraries(${node_executable_name} ${PROJECT_NAME})
ament_target_dependencies(${node_executable_name} ${dependencies})

# Executables(node_main)
set(mockamap_generator_executable_name mockamap_generator_node)
add_executable(${mockamap_generator_executable_name} mockamap_generator_main.cpp)
target_link_libraries(${mockamap_generator_executable_name} ${PROJECT_NAME})
ament_target_dependencies(${mockamap_generator_executable_name} ${dependencies})

# Executables(node_main)
set(pointcloud_to_occupancy_grid_executable_name pointcloud_to_occupancy_grid_node)
add_executable(${pointcloud_to_occupancy_grid_executable_name} pointcloud_to_occupancy_grid_main.cpp)
target_link_libraries(${pointcloud_to_occupancy_grid_executable_name} ${PROJECT_NAME})
ament_target_dependencies(${pointcloud_to_occupancy_grid_executable_name} ${dependencies})

install(TARGETS
  ${node_executable_name}
  ${mockamap_generator_executable_name}
  ${pointcloud_to_occupancy_grid_executable_name}
  DESTINATION lib/${PROJECT_NAME}
)
