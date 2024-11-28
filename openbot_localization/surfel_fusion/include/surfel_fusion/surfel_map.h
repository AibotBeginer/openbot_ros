#include <list>
#include <vector>
#include <set>
#include <iostream>
#include <chrono>
#include <thread>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/interprocess/sync/file_lock.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>

#include <boost/shared_ptr.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>

// #include <sensor_msgs/image_encodings.h>


#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/int16.hpp>

#include "surfel_fusion/elements.h"
#include "surfel_fusion/fusion_functions.h"


typedef pcl::PointXYZI PointType;
typedef pcl::PointXYZRGBNormal RgbPointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<RgbPointType> RgbPointCloud;

using namespace std;

struct PoseElement{
    //pose_index is the index in the vector in the database
    vector<SurfelElement> attached_surfels;
    geometry_msgs::msg::Pose cam_pose;
    geometry_msgs::msg::Pose loop_pose;
    vector<int> linked_pose_index;
    int points_begin_index;
    int points_pose_index;
    rclcpp::Time cam_stamp;
    PoseElement() : points_begin_index(-1), points_pose_index(-1) {}
};

class SurfelMap : public rclcpp::Node
{
public:
    SurfelMap();

    void image_input(const sensor_msgs::msg::Image::ConstPtr &image_input);
    void depth_input(const sensor_msgs::msg::Image::ConstPtr &image_input);
    void color_input(const sensor_msgs::msg::Image::ConstPtr &image_input);
    void path_input(const nav_msgs::msg::Path::ConstPtr &loop_path_input);
    void extrinsic_input(const nav_msgs::msg::Odometry::ConstPtr &ex_input);
    void surfel_cmd_callback(const std_msgs::msg::Int16::ConstPtr &cmd);
    void orb_results_input(
        const sensor_msgs::msg::PointCloud2::ConstPtr &loop_stamp_input,
        const nav_msgs::msg::Path::ConstPtr &loop_path_input,
        const nav_msgs::msg::Odometry::ConstPtr &this_pose_input);
    void save_cloud(string save_path_name);
    void save_mesh(string save_path_name);
    void save_map(const std_msgs::msg::String::ConstPtr &save_map_input);
    void publish_all_pointcloud();

    bool surfel_state;
    void set_map_dir(string str);

  private:
    void synchronize_msgs();
    void fuse_map(cv::Mat image, cv::Mat depth, Eigen::Matrix4f pose_input, int reference_index);

    void move_add_surfels(int reference_index);
    void move_all_surfels();
    bool synchronize_buffer();
    void get_add_remove_poses(int root_index, vector<int> &pose_to_add, vector<int> &pose_to_remove);
    void get_driftfree_poses(int root_index, vector<int> &driftfree_poses, int driftfree_range);

    void pose_ros2eigen(geometry_msgs::msg::Pose &pose, Eigen::Matrix4d &T);
    void pose_eigen2ros(Eigen::Matrix4d &T, geometry_msgs::msg::Pose &pose);

    void render_depth(geometry_msgs::msg::Pose &pose);
    void publish_neighbor_pointcloud(rclcpp::Time pub_stamp, int reference_index);
    void publish_raw_pointcloud(cv::Mat &depth, cv::Mat &reference, geometry_msgs::msg::Pose &pose);
    void publish_pose_graph(rclcpp::Time pub_stamp, int reference_index);
    void calculate_memory_usage();

    // for surfel save into mesh
    void push_a_surfel(vector<float> &vertexs, SurfelElement &this_surfel);

    sensor_msgs::msg::PointCloud2 ToPointCloud2(const PointCloud::Ptr points);

    // image
    cv::Mat debug_image;

    // receive buffer
    std::deque<std::pair<rclcpp::Time, cv::Mat>> image_buffer;
    std::deque<std::pair<rclcpp::Time, cv::Mat>> depth_buffer;
    std::deque<std::pair<rclcpp::Time, cv::Mat>> color_buffer;
    std::deque<std::pair<rclcpp::Time, int>> pose_reference_buffer;

    // camera param
    int cam_width;
    int cam_height;
    float cam_fx, cam_fy, cam_cx, cam_cy;
    Eigen::Matrix4d extrinsic_matrix, T_d2c;
    bool extrinsic_matrix_initialized;

    Eigen::Matrix3d camera_matrix;
    Eigen::Matrix3d imu_cam_rot;
    Eigen::Vector3d imu_cam_tra;

    // fuse param
    float far_dist, near_dist;

    // fusion tools
    FusionFunctions fusion_functions;

    // database
    string save_name;
    vector<SurfelElement> local_surfels;
    vector<PoseElement> poses_database;
    std::set<int> local_surfels_indexs;
    int drift_free_poses;

    // for inactive warp
    std::vector<std::thread> warp_thread_pool;
    int warp_thread_num;
    void warp_surfels();
    void warp_inactive_surfels_cpu_kernel(int thread_i, int step);
    void warp_active_surfels_cpu_kernel(int thread_i, int thread_num, Eigen::Matrix4f transform_m);

    // for fast publish
    PointCloud::Ptr inactive_pointcloud;
    RgbPointCloud::Ptr rgb_inactive_pcd;
    std::vector<int> pointcloud_pose_index;

    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publish;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pointcloud_publish;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rgb_pointcloud_publish;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr loop_path_publish;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr driftfree_path_publish;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr loop_marker_publish;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cam_pose_publish;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sp_img_publish;

    // Subscriptor
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_save_map;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_extrinsic_pose;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_cmd;

    rclcpp::TimerBase::SharedPtr timer_;

    // save map
    string map_dir;

    // for gaofei experiment
    bool is_first_path;
    double pre_path_delete_time;
};