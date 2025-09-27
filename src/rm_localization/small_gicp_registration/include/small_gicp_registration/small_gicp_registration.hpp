#ifndef SMALL_GICP_REGISTRATION_HPP
#define SMALL_GICP_REGISTRATION_HPP

// std
#include <filesystem>
#include <mutex>
#include <memory>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// small_gicp
#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling.hpp>
#include <small_gicp/points/point_cloud.hpp>

namespace small_gicp_localization {

using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;
using PointCloudPtr = PointCloud::Ptr;

/**
 * @brief Small GICP-based point cloud registration node for robot localization
 * 
 * This node uses the high-performance small_gicp library for point cloud registration,
 * providing faster and more accurate localization compared to traditional PCL-based ICP.
 */
class SmallGicpNode : public rclcpp::Node {
public:
  explicit SmallGicpNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~SmallGicpNode();

private:
  /**
   * @brief Callback for incoming point cloud data
   */
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief Callback for initial pose estimation
   */
  void initialPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief Load reference point cloud map from PCD file
   */
  bool loadReferenceMap();

  /**
   * @brief Perform multi-candidate GICP registration
   * @param source Input point cloud to be registered
   * @param init_guess Initial transformation guess
   * @return Best transformation matrix and success flag
   */
  std::pair<Eigen::Isometry3d, bool> performRegistration(
      const PointCloudPtr& source,
      const Eigen::Isometry3d& init_guess);

  /**
   * @brief Generate multiple candidate poses around initial guess
   */
  std::vector<Eigen::Isometry3d> generateCandidatePoses(
      const Eigen::Isometry3d& init_guess);

  /**
   * @brief Publish transformation as TF
   */
  void publishTransform(const Eigen::Isometry3d& transform);

  /**
   * @brief Convert Eigen transform to geometry_msgs transform
   */
  geometry_msgs::msg::TransformStamped eigenToTransformStamped(
      const Eigen::Isometry3d& transform,
      const std::string& frame_id,
      const std::string& child_frame_id,
      const rclcpp::Time& stamp);

  // ROS2 interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_sub_;
  
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Thread safety
  std::mutex mutex_;
  std::unique_ptr<std::thread> tf_publisher_thread_;

  // Point cloud data
  PointCloudPtr current_scan_;
  PointCloudPtr reference_map_;
  
  // Small GICP components
  std::unique_ptr<small_gicp::RegistrationPCL<PointType, PointType>> registration_;
  
  // Transform data
  geometry_msgs::msg::TransformStamped map_to_odom_;
  
  // Configuration parameters
  std::filesystem::path pcd_path_;
  std::string map_frame_id_;
  std::string odom_frame_id_;
  std::string range_odom_frame_id_;
  std::string laser_frame_id_;
  std::string pointcloud_topic_;
  
  // Registration parameters
  double downsampling_resolution_;
  double max_correspondence_distance_;
  int num_threads_;
  double convergence_threshold_;
  int max_iterations_;
  
  // Multi-candidate search parameters
  double xy_search_range_;
  double yaw_search_range_;
  double xy_step_;
  double yaw_step_;
  
  // Initial pose
  geometry_msgs::msg::Pose initial_pose_;
  
  // Status flags
  bool is_initialized_;
  bool map_loaded_;
  bool first_scan_;
  double last_registration_score_;
};

} // namespace small_gicp_localization

#endif // SMALL_GICP_REGISTRATION_HPP
