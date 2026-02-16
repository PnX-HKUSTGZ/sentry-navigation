#include "small_gicp_registration/small_gicp_registration.hpp"
#include <pcl/io/pcd_io.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <rclcpp/qos.hpp>

namespace small_gicp_localization {

SmallGicpNode::SmallGicpNode(const rclcpp::NodeOptions &options)
    : Node("small_gicp_registration", options),
      is_initialized_(false),
      map_loaded_(false),
      first_scan_(true),
      last_registration_score_(std::numeric_limits<double>::max()) {
  
  // 声明参数
  this->declare_parameter("pcd_path", std::string(""));
  this->declare_parameter("map_frame_id", std::string("map"));
  this->declare_parameter("odom_frame_id", std::string("odom"));
  this->declare_parameter("range_odom_frame_id", std::string("lidar_odom"));
  this->declare_parameter("laser_frame_id", std::string("livox_frame"));
  this->declare_parameter("pointcloud_topic", std::string("/livox/lidar/pointcloud"));
  
  // 配准参数
  this->declare_parameter("downsampling_resolution", 0.25);
  this->declare_parameter("max_correspondence_distance", 1.0);
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("convergence_threshold", 0.01);
  this->declare_parameter("max_iterations", 30);
  
  // 多候选搜索参数
  this->declare_parameter("xy_search_range", 0.5);
  this->declare_parameter("yaw_search_range", 30.0); // degrees
  this->declare_parameter("xy_step", 0.1);
  this->declare_parameter("yaw_step", 10.0); // degrees
  
  // 初始位姿
  this->declare_parameter("initial_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  // 获取参数
  pcd_path_ = this->get_parameter("pcd_path").as_string();
  map_frame_id_ = this->get_parameter("map_frame_id").as_string();
  odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
  range_odom_frame_id_ = this->get_parameter("range_odom_frame_id").as_string();
  laser_frame_id_ = this->get_parameter("laser_frame_id").as_string();
  pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
  
  downsampling_resolution_ = this->get_parameter("downsampling_resolution").as_double();
  max_correspondence_distance_ = this->get_parameter("max_correspondence_distance").as_double();
  num_threads_ = this->get_parameter("num_threads").as_int();
  convergence_threshold_ = this->get_parameter("convergence_threshold").as_double();
  max_iterations_ = this->get_parameter("max_iterations").as_int();
  
  xy_search_range_ = this->get_parameter("xy_search_range").as_double();
  yaw_search_range_ = this->get_parameter("yaw_search_range").as_double() * M_PI / 180.0;
  xy_step_ = this->get_parameter("xy_step").as_double();
  yaw_step_ = this->get_parameter("yaw_step").as_double() * M_PI / 180.0;

  // 获取初始位姿
  auto initial_pose_vec = this->get_parameter("initial_pose").as_double_array();
  if (initial_pose_vec.size() >= 6) {
    initial_pose_.position.x = initial_pose_vec[0];
    initial_pose_.position.y = initial_pose_vec[1];
    initial_pose_.position.z = initial_pose_vec[2];
    tf2::Quaternion q;
    q.setRPY(initial_pose_vec[3], initial_pose_vec[4], initial_pose_vec[5]);
    initial_pose_.orientation.x = q.x();
    initial_pose_.orientation.y = q.y();
    initial_pose_.orientation.z = q.z();
    initial_pose_.orientation.w = q.w();
  }

  // 初始化small_gicp配准器
  registration_ = std::make_unique<small_gicp::RegistrationPCL<PointType, PointType>>();
  registration_->setNumThreads(num_threads_);
  registration_->setMaxCorrespondenceDistance(max_correspondence_distance_);
  registration_->setMaximumIterations(max_iterations_);
  registration_->setRegistrationType("GICP");  // 可以选择 "ICP", "PLANE_ICP", "GICP", "VGICP"

  // 初始化TF组件
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  // 加载参考地图
  if (!loadReferenceMap()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load reference map from: %s", pcd_path_.c_str());
    return;
  }

  // 创建订阅者
    auto sensor_qos = rclcpp::SensorDataQoS();
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, sensor_qos,
      std::bind(&SmallGicpNode::pointcloudCallback, this, std::placeholders::_1));

    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
      std::bind(&SmallGicpNode::initialPoseCallback, this, std::placeholders::_1));

    // Continuously publish the latest map->odom TF so downstream nodes can query at "now".
    tf_publisher_thread_ = std::make_unique<std::thread>([this]() {
      rclcpp::Rate rate(100);
      while (rclcpp::ok()) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (is_initialized_ && map_to_odom_.header.frame_id == map_frame_id_ && map_to_odom_.child_frame_id == odom_frame_id_) {
            map_to_odom_.header.stamp = this->now();
            tf_broadcaster_->sendTransform(map_to_odom_);
          }
        }
        rate.sleep();
      }
    });

  RCLCPP_INFO(this->get_logger(), "Small GICP localization node initialized successfully");
  RCLCPP_INFO(this->get_logger(), "  Map file: %s", pcd_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Downsampling resolution: %.3f", downsampling_resolution_);
  RCLCPP_INFO(this->get_logger(), "  Max correspondence distance: %.3f", max_correspondence_distance_);
  RCLCPP_INFO(this->get_logger(), "  Number of threads: %d", num_threads_);
}

SmallGicpNode::~SmallGicpNode() {
  if (tf_publisher_thread_ && tf_publisher_thread_->joinable()) {
    tf_publisher_thread_->join();
  }
}

bool SmallGicpNode::loadReferenceMap() {
  if (pcd_path_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "PCD path not specified");
    return false;
  }

  if (!std::filesystem::exists(pcd_path_)) {
    RCLCPP_ERROR(this->get_logger(), "PCD file does not exist: %s", pcd_path_.c_str());
    return false;
  }

  reference_map_ = std::make_shared<PointCloud>();
  if (pcl::io::loadPCDFile<PointType>(pcd_path_.string(), *reference_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", pcd_path_.c_str());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Loaded reference map with %zu points", reference_map_->size());

  // 对参考地图进行降采样  
  auto downsampled_map = small_gicp::voxelgrid_sampling(*reference_map_, downsampling_resolution_);
  reference_map_ = downsampled_map;
  
  RCLCPP_INFO(this->get_logger(), "Downsampled reference map to %zu points", reference_map_->size());

  // 设置为目标点云
  registration_->setInputTarget(reference_map_);
  
  map_loaded_ = true;
  return true;
}

void SmallGicpNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!map_loaded_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Reference map not loaded yet");
    return;
  }

  // 转换点云格式
  current_scan_ = std::make_shared<PointCloud>();
  pcl::fromROSMsg(*msg, *current_scan_);

  if (current_scan_->empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
    return;
  }

  // Apply cached /initialpose if it arrived before any scan.
  if (has_pending_initialpose_ && pending_initialpose_) {
    initialPoseCallback(pending_initialpose_);
    // initialPoseCallback() will clear the pending flag on success.
    // If it still fails, fall through and try the default initialization path.
  }

  // 如果是第一帧或者未初始化，使用初始位姿
  if (first_scan_ || !is_initialized_) {
    Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();
    init_guess.translation() = Eigen::Vector3d(
        initial_pose_.position.x,
        initial_pose_.position.y,
        initial_pose_.position.z
    );
    init_guess.linear() = Eigen::Quaterniond(
        initial_pose_.orientation.w,
        initial_pose_.orientation.x,
        initial_pose_.orientation.y,
        initial_pose_.orientation.z
    ).toRotationMatrix();

    auto [transform, success] = performRegistration(current_scan_, init_guess);
    
    if (success) {
      publishTransform(transform);
      is_initialized_ = true;
      first_scan_ = false;
      RCLCPP_INFO(this->get_logger(), "Initial localization successful");
    } else {
      RCLCPP_WARN(this->get_logger(), "Initial localization failed");
    }
  } else {
    // 使用上一次的map->odom与当前odom->laser作为初始猜测（提高收敛速度与稳定性）
    Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();
    try {
      // odom -> laser (in tf2 API: target=laser, source=range_odom)
      auto odom_to_laser_msg = tf_buffer_->lookupTransform(
          laser_frame_id_, range_odom_frame_id_, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.2));
      Eigen::Isometry3d odom_to_laser = transformStampedToEigen(odom_to_laser_msg);

      Eigen::Isometry3d map_to_odom = Eigen::Isometry3d::Identity();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        // Only use the last published transform when it's valid.
        if (map_to_odom_.header.frame_id == map_frame_id_ && map_to_odom_.child_frame_id == odom_frame_id_) {
          map_to_odom = transformStampedToEigen(map_to_odom_);
        }
      }

      // map -> laser = (map -> odom) * (odom -> laser)
      // But small_gicp expects a transform that maps source(laser) into target(map): map_from_laser.
      // map_from_laser = map_from_odom * odom_from_laser
      Eigen::Isometry3d odom_from_laser = odom_to_laser.inverse();
      init_guess = map_to_odom * odom_from_laser;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF lookup failed for init guess: %s", ex.what());
    }
    
    auto [transform, success] = performRegistration(current_scan_, init_guess);
    
    if (success) {
      publishTransform(transform);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Registration failed");
    }
  }
}

void SmallGicpNode::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  
  RCLCPP_INFO(this->get_logger(), "Received initial pose, performing relocalization");

  if (!map_loaded_) {
    RCLCPP_WARN(this->get_logger(), "Map not loaded; ignoring /initialpose");
    return;
  }

  // /initialpose may arrive before the first scan (RViz is often faster than sensors).
  if (!current_scan_ || current_scan_->empty()) {
    pending_initialpose_ = msg;
    has_pending_initialpose_ = true;
    // Also store it as the default initial pose for when the first scan arrives.
    initial_pose_ = msg->pose.pose;
    RCLCPP_WARN(this->get_logger(),
                "Received /initialpose but no scan available yet; caching until first scan arrives");
    return;
  }

  // Update the default initial pose for future re-inits.
  initial_pose_ = msg->pose.pose;

  // 构建初始变换
  Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();
  init_guess.translation() = Eigen::Vector3d(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z
  );
  init_guess.linear() = Eigen::Quaterniond(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z
  ).toRotationMatrix();

  auto [transform, success] = performRegistration(current_scan_, init_guess);

  if (success) {
    publishTransform(transform);
    is_initialized_ = true;
    first_scan_ = false;
    pending_initialpose_.reset();
    has_pending_initialpose_ = false;
    RCLCPP_INFO(this->get_logger(), "Relocalization successful with score: %.6f", last_registration_score_);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Relocalization failed");
    // Keep it pending so we can retry on the next scan.
    pending_initialpose_ = msg;
    has_pending_initialpose_ = true;
  }
}

Eigen::Isometry3d SmallGicpNode::transformStampedToEigen(const geometry_msgs::msg::TransformStamped& transform) {
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = Eigen::Vector3d(
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z);
  out.linear() = Eigen::Quaterniond(
      transform.transform.rotation.w,
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z).toRotationMatrix();
  return out;
}

std::pair<Eigen::Isometry3d, bool> SmallGicpNode::performRegistration(
    const PointCloudPtr& source, const Eigen::Isometry3d& init_guess) {
  
  // 对输入点云进行降采样
  auto downsampled_source = small_gicp::voxelgrid_sampling(*source, downsampling_resolution_);
  
  // 生成候选位姿
  auto candidate_poses = generateCandidatePoses(init_guess);
  
  double best_score = std::numeric_limits<double>::max();
  Eigen::Isometry3d best_transform = init_guess;
  bool success = false;

  RCLCPP_DEBUG(this->get_logger(), "Testing %zu candidate poses", candidate_poses.size());

  // 对每个候选位姿进行配准
  for (const auto& candidate : candidate_poses) {
    registration_->setInputSource(downsampled_source);
    
    // 执行配准
    auto aligned = std::make_shared<PointCloud>();
    registration_->align(*aligned, candidate.matrix().cast<float>());
    
    if (registration_->hasConverged()) {
      double score = registration_->getFitnessScore();
      
      if (score < best_score) {
        best_score = score;
        best_transform = Eigen::Isometry3d(registration_->getFinalTransformation().cast<double>());
        success = true;
      }
    }
  }

  last_registration_score_ = best_score;
  
  if (success) {
    RCLCPP_DEBUG(this->get_logger(), "Registration converged with score: %.6f", best_score);
  }

  return {best_transform, success};
}

std::vector<Eigen::Isometry3d> SmallGicpNode::generateCandidatePoses(
    const Eigen::Isometry3d& init_guess) {
  
  std::vector<Eigen::Isometry3d> candidates;
  
  // 提取初始位置和朝向
  Eigen::Vector3d init_position = init_guess.translation();
  Eigen::Matrix3d init_rotation = init_guess.linear();
  
  // 从旋转矩阵中提取欧拉角
  Eigen::Vector3d euler = init_rotation.eulerAngles(0, 1, 2); // Roll, Pitch, Yaw
  double init_yaw = euler(2);

  // 在XY平面和Yaw角度周围生成候选位姿
  for (double dx = -xy_search_range_; dx <= xy_search_range_; dx += xy_step_) {
    for (double dy = -xy_search_range_; dy <= xy_search_range_; dy += xy_step_) {
      for (double dyaw = -yaw_search_range_; dyaw <= yaw_search_range_; dyaw += yaw_step_) {
        
        Eigen::Isometry3d candidate = Eigen::Isometry3d::Identity();
        
        // 设置位置
        candidate.translation() = init_position + Eigen::Vector3d(dx, dy, 0.0);
        
        // 设置朝向（保持roll和pitch，只改变yaw）
        Eigen::AngleAxisd roll_angle(euler(0), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle(euler(1), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw_angle(init_yaw + dyaw, Eigen::Vector3d::UnitZ());
        
        candidate.linear() = (roll_angle * pitch_angle * yaw_angle).toRotationMatrix();
        
        candidates.push_back(candidate);
      }
    }
  }

  return candidates;
}

void SmallGicpNode::publishTransform(const Eigen::Isometry3d& map_to_laser) {
  try {
    // 获取laser到odom的变换
    auto transform_stamped = tf_buffer_->lookupTransform(
        laser_frame_id_, range_odom_frame_id_, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
    
    // 转换为Eigen格式
    Eigen::Isometry3d laser_to_odom = Eigen::Isometry3d::Identity();
    laser_to_odom.translation() = Eigen::Vector3d(
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z
    );
    laser_to_odom.linear() = Eigen::Quaterniond(
        transform_stamped.transform.rotation.w,
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z
    ).toRotationMatrix();

    // 计算map到odom的变换
    Eigen::Isometry3d map_to_odom = map_to_laser * laser_to_odom;

    // 转换并发布TF
    std::lock_guard<std::mutex> lock(mutex_);
    map_to_odom_ = eigenToTransformStamped(
        map_to_odom, map_frame_id_, odom_frame_id_, this->get_clock()->now());
    
    tf_broadcaster_->sendTransform(map_to_odom_);
    
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s", ex.what());
  }
}

geometry_msgs::msg::TransformStamped SmallGicpNode::eigenToTransformStamped(
    const Eigen::Isometry3d& transform,
    const std::string& frame_id,
    const std::string& child_frame_id,
    const rclcpp::Time& stamp) {
  
  geometry_msgs::msg::TransformStamped transform_stamped;
  
  transform_stamped.header.stamp = stamp;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  
  transform_stamped.transform.translation.x = transform.translation().x();
  transform_stamped.transform.translation.y = transform.translation().y();
  transform_stamped.transform.translation.z = transform.translation().z();
  
  Eigen::Quaterniond quaternion(transform.linear());
  transform_stamped.transform.rotation.x = quaternion.x();
  transform_stamped.transform.rotation.y = quaternion.y();
  transform_stamped.transform.rotation.z = quaternion.z();
  transform_stamped.transform.rotation.w = quaternion.w();
  
  return transform_stamped;
}

} // namespace small_gicp_localization

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_localization::SmallGicpNode)
