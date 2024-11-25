/*
 * Copyright 2024 The OpenRobotic Beginner Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "openbot_ros/pointcloud_to_occupancy_grid.hpp"

using Cell = std::pair<int, int>;
namespace openbot_ros
{
  PointCloudToOccupancyGridNode::PointCloudToOccupancyGridNode() : rclcpp::Node("pointcloud_to_occupancy_grid_node"),
    last_publish_time_(std::chrono::steady_clock::now())
  {
    // Initialize subscriber and publishers
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_in", 10,
        std::bind(&PointCloudToOccupancyGridNode::pointCloudCallback, this, std::placeholders::_1));

    base_frame_id_ = declare_parameter("base_frame_id", "base_footprint");

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    pointcloud_camera_frame_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out", rclcpp::QoS{1});
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&PointCloudToOccupancyGridNode::transformPointCloud, this));

        
    mission_pub_ = this->create_publisher<motion_msgs::msg::MotionCtrl>("/diablo/MotionCmd", 10);
  }

  void PointCloudToOccupancyGridNode::publishStandUpMotionCmd(bool stand_mode)
  {
    auto now = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_publish_time_).count();

    if (elapsed_time < 1000) {
      // Ensure we do not publish more than 2 messages in 500 ms.
      RCLCPP_DEBUG(this->get_logger(), "Throttling message publication to avoid exceeding limit");
      return;
    }
    motion_msgs::msg::MotionCtrl missionMsg;

    missionMsg.value.up = 1.0;
      missionMsg.mode_mark = true;

    if (stand_mode) {
      missionMsg.mode.stand_mode = true;
    } else {
      missionMsg.mode.stand_mode = false;
    }

    mission_pub_->publish(missionMsg);

    // Update the last publish time.
    last_publish_time_ = std::chrono::steady_clock::now();
  }

  bool PointCloudToOccupancyGridNode::isHolePassable(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                    float min_z_threshold, 
                    float max_z_threshold, 
                    float min_width) {

    // Parameters for the grid
    const float grid_size = 2.5f;  // 3x3 area
    const float cell_resolution = 0.1f;  // Grid resolution (smaller means finer grid)

    RCLCPP_DEBUG(this->get_logger(), 
                 "Grid size: %.2f, Cell resolution: %.2f", 
                 grid_size, cell_resolution);

    // Create a grid in the x-y plane
    std::map<std::pair<int, int>, std::vector<pcl::PointXYZ>> grid;
    for (const auto& point : cloud->points) {
        // Restrict points to the grid size
        if (std::fabs(point.x) > grid_size / 2 || std::fabs(point.y) > grid_size / 2) {
            continue;  // Skip points outside the grid_size bounds
        }

        int x_idx = static_cast<int>(std::floor(point.x / cell_resolution));
        int y_idx = static_cast<int>(std::floor(point.y / cell_resolution));

        grid[{x_idx, y_idx}].push_back(point);
    }

    RCLCPP_DEBUG(this->get_logger(), 
                 "Point cloud divided into %lu grid cells.", 
                 grid.size());

    // Check each grid cell for conditions and mark empty cells
    std::set<Cell> empty_cells;
    for (const auto& cell : grid) {
        const auto& points = cell.second;
        bool has_points_at_z = false;
        bool has_points_below_z = false;

        // Analyze points in the cell
        for (const auto& point : points) {
            if (point.z >= min_z_threshold && point.z <= max_z_threshold) {
                has_points_at_z = true;
            }
            if (point.z < min_z_threshold) {
                has_points_below_z = true;
            }
        }

        if (has_points_at_z && !has_points_below_z) {
            empty_cells.insert(cell.first);  // Mark this cell as empty
            RCLCPP_DEBUG(this->get_logger(), 
                         "Cell (%d, %d) marked as empty.", 
                         cell.first.first, cell.first.second);
        }
    }

    RCLCPP_DEBUG(this->get_logger(), 
                "Total empty cells detected: %lu.", 
                empty_cells.size());

    // Flood-fill to group connected empty cells
    std::set<Cell> visited;
    std::vector<std::vector<Cell>> connected_components;
    const std::vector<Cell> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

    for (const auto& cell : empty_cells) {
        if (visited.count(cell)) continue;

        // Start a new connected component
        std::vector<Cell> component;
        std::vector<Cell> stack = {cell};

        while (!stack.empty()) {
            Cell current = stack.back();
            stack.pop_back();

            if (visited.count(current)) continue;
            visited.insert(current);
            component.push_back(current);

            // Check neighbors
            for (const auto& dir : directions) {
                Cell neighbor = {current.first + dir.first, current.second + dir.second};
                if (empty_cells.count(neighbor) && !visited.count(neighbor)) {
                    stack.push_back(neighbor);
                }
            }
        }

        connected_components.push_back(component);
        RCLCPP_DEBUG(this->get_logger(), 
                     "Connected component with %lu cells detected.", 
                     component.size());
    }

    RCLCPP_DEBUG(this->get_logger(), 
                "Total connected components: %lu.", 
                connected_components.size());

    // Check dimensions of connected components
    for (const auto& component : connected_components) {
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();

        for (const auto& cell : component) {
            float cell_x = cell.first * cell_resolution;
            float cell_y = cell.second * cell_resolution;

            min_x = std::min(min_x, cell_x);
            max_x = std::max(max_x, cell_x);
            min_y = std::min(min_y, cell_y);
            max_y = std::max(max_y, cell_y);
        }

        float width = std::fabs(max_x - min_x) + std::fabs(max_y - min_y);

        RCLCPP_DEBUG(this->get_logger(), 
                     "Component dimensions - Width: %.2f, Min_x: %.2f, Max_x: %.2f, Min_y: %.2f, Max_y: %.2f", 
                     width, min_x, max_x, min_y, max_y);

        if (width >= min_width) {
            RCLCPP_INFO(this->get_logger(), 
                        "Passable hole found with width %.2f.", 
                        width);
            return true;  // Passable hole found
        }
    }

    RCLCPP_INFO(this->get_logger(), 
                "No passable hole found in the point cloud.");
    return false;  // No passable hole found
}


  void PointCloudToOccupancyGridNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    cloud_in_ = cloud;
    transformPointCloud();
  }

  void PointCloudToOccupancyGridNode::simulateRgbdCamera(
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
  {
    // Filter points based on FOV
    float horizontal_fov_rad = 68.0 * M_PI / 180.0; // Horizontal FOV in radians
    float vertical_fov_rad = 68.0 * M_PI / 180.0;   // Vertical FOV in radians
    bool publish_message = false;


    pcl::PointCloud<pcl::PointXYZ>::Ptr camera_filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr ultra_sonic_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr standup_sitdown_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    for (const auto &point : cloud->points)
    {
      if (std::abs(point.x) <= 0.4 && std::abs(point.y) <= 0.4)
      {
        standup_sitdown_ptr->points.push_back(point);
        continue;
      }

      if (std::abs(point.x) <= 1 && std::abs(point.y) <= 1)
      {
        ultra_sonic_ptr->points.push_back(point);
        continue;
      }

      // Calculate horizontal and vertical angles relative to the robot's pose
      float angle_h = std::atan2(point.y, point.x); // Horizontal angle using Y and X
      float angle_v = std::atan2(point.z, std::sqrt(point.x * point.x + point.y * point.y)); // Vertical angle

      // Check if point lies within the FOV
      if (std::abs(angle_h) <= (horizontal_fov_rad / 2) && (true ||
          std::abs(angle_v) <= (vertical_fov_rad / 2)))
      {
        camera_filtered_ptr->points.push_back(point);
        standup_sitdown_ptr->points.push_back(point);
      } else {

        // cloud_filtered->points.push_back(point);
      }
    }
    
    if (isHolePassable(standup_sitdown_ptr, 
                    0.3, 0.8,
                    0.3)) {
      publishStandUpMotionCmd(false);
    } else {
      publishStandUpMotionCmd(true);
    }

    for (const auto &point : camera_filtered_ptr->points)
    {

      if (std::abs(point.z) >= 0.8)
      {
        // skip z bigger than 1
        continue;
      }

      if (std::abs(point.z) >= 0.3)
      {
        continue;
      }
      cloud_filtered->points.push_back(point);
    }

    for (const auto &point : ultra_sonic_ptr->points)
    {
      if (std::abs(point.z) >= 0.2)
      {
        continue;
      }
      cloud_filtered->points.push_back(point);
    }

    // Log the counts
    RCLCPP_DEBUG(this->get_logger(),
                "Original Cloud: %lu. Filtered cloud contains %lu points.",
                cloud->points.size(),
                cloud_filtered->points.size());
  }

  void PointCloudToOccupancyGridNode::transformPointCloud()
  {
    if (cloud_in_ == nullptr)
    {
      return;
    }

    geometry_msgs::msg::TransformStamped cloud_to_base_transform_stamped;
    try
    {
      cloud_to_base_transform_stamped = tf2_buffer_->lookupTransform(
          base_frame_id_, cloud_in_->header.frame_id, cloud_in_->header.stamp,
          rclcpp::Duration::from_seconds(1.0));

      RCLCPP_DEBUG(this->get_logger(),
                   "[PointCloudToOccupancyGridNode] Transform lookup successful: base_frame_id_: %s, \n"
                   "Timestamp: [%d.%09d]\n"
                   "Frame: '%s' to '%s'\n"
                   "Translation: [x: %f, y: %f, z: %f]\n"
                   "Rotation (quaternion): [x: %f, y: %f, z: %f, w: %f]",
                   base_frame_id_,
                   cloud_to_base_transform_stamped.header.stamp.sec,
                   cloud_to_base_transform_stamped.header.stamp.nanosec,
                   cloud_to_base_transform_stamped.header.frame_id.c_str(),
                   cloud_to_base_transform_stamped.child_frame_id.c_str(),
                   cloud_to_base_transform_stamped.transform.translation.x,
                   cloud_to_base_transform_stamped.transform.translation.y,
                   cloud_to_base_transform_stamped.transform.translation.z,
                   cloud_to_base_transform_stamped.transform.rotation.x,
                   cloud_to_base_transform_stamped.transform.rotation.y,
                   cloud_to_base_transform_stamped.transform.rotation.z,
                   cloud_to_base_transform_stamped.transform.rotation.w);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_to_publish;
    auto stamp = cloud_in_->header.stamp;
    tf2::doTransform(*cloud_in_, cloud_to_publish, cloud_to_base_transform_stamped);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud_to_publish, *pc_ptr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    // cloud_filtered_ptr = pc_ptr;
    simulateRgbdCamera(pc_ptr, cloud_filtered_ptr);

    pcl::toROSMsg(*cloud_filtered_ptr, cloud_to_publish);
    cloud_to_publish.header.frame_id = base_frame_id_;
    cloud_to_publish.header.stamp = stamp;

    pointcloud_camera_frame_pub_->publish(cloud_to_publish);
    return;

    // todo(lzl): pcl_ros crashes

    // Check if transform is identity
    // pointcloud_to_occupancy_grid_node-15] Translation: [x: 0.002709, y: -0.000001, z: 0.000000]
    // [pointcloud_to_occupancy_grid_node-15] Rotation (quaternion): [x: 0.000000, y: 0.000000, z: -0.000540, w: 1.000000]
    // [ERROR] [pointcloud_to_occupancy_grid_node-15]: process has died [pid 1249919, exit code -11, cmd '/home/hello/codebase/projects/jdt_openbot/install/openbot_ros/lib/openbot_ros/pointcloud_to_occupancy_grid_node --ros-args --log-level debug --ros-args --params-file /tmp/launch_params_fzg40bzp -r /cloud_in:=/global_map -r /cloud_out:=/camera_point_cloud'].
    // pcl::PointCloud<pcl::PointXYZ> transformed_pc;
    // if (cloud_to_base_transform_stamped.transform.translation.x == 0.0 &&
    //     cloud_to_base_transform_stamped.transform.translation.y == 0.0 &&
    //     cloud_to_base_transform_stamped.transform.translation.z == 0.0 &&
    //     cloud_to_base_transform_stamped.transform.rotation.x == 0.0 &&
    //     cloud_to_base_transform_stamped.transform.rotation.y == 0.0 &&
    //     cloud_to_base_transform_stamped.transform.rotation.z == 0.0 &&
    //     cloud_to_base_transform_stamped.transform.rotation.w == 1.0)
    // {
    //   RCLCPP_INFO(this->get_logger(), "Identity transform detected. Skipping point cloud transformation.");
    //   // Publish the input point cloud directly
    //   transformed_pc = pc;
    // }
    // else
    // {

    //   // https://github.com/PointCloudLibrary/pcl/pull/5113
    //   pcl_ros::transformPointCloud(pc, transformed_pc, cloud_to_base_transform_stamped);
    //   // Eigen::Isometry3d transform_isometry;
    //   // tf2::transformToEigen(cloud_to_base_transform_stamped.transform, transform_isometry);

    //   // // Convert to Eigen::Matrix4f for PCL compatibility
    //   // Eigen::Matrix4f transform = transform_isometry.matrix().cast<float>();

    //   // // Transform the point cloud
    //   // pcl::PointCloud<pcl::PointXYZ> transformed_pc;
    //   // pcl::transformPointCloud(pc, transformed_pc, transform);
    // }
  }
} // namespace openbot_ros