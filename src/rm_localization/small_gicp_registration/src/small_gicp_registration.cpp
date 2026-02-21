#include "small_gicp_registration/small_gicp_registration.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <rclcpp/qos.hpp>

namespace small_gicp_localization {
namespace {
bool hasPointField(const sensor_msgs::msg::PointCloud2& msg, const char* field_name) {
  for (const auto& field : msg.fields) {
    if (field.name == field_name) {
      return true;
    }
  }
  return false;
}

Eigen::Isometry3d projectToPlanar(const Eigen::Isometry3d& in) {
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d t = in.translation();
  const double yaw = std::atan2(in.linear()(1, 0), in.linear()(0, 0));
  out.translation() = Eigen::Vector3d(t.x(), t.y(), 0.0);
  out.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return out;
}

double planarYaw(const Eigen::Isometry3d& in) {
  return std::atan2(in.linear()(1, 0), in.linear()(0, 0));
}
}  // namespace

SmallGicpNode::SmallGicpNode(const rclcpp::NodeOptions &options)
    : Node("small_gicp_registration", options),
      is_initialized_(false),
      map_loaded_(false),
      first_scan_(true),
      startup_time_(rclcpp::Time(0, 0, RCL_ROS_TIME)),
      last_initialpose_time_(rclcpp::Time(0, 0, RCL_ROS_TIME)),
      last_registration_score_(std::numeric_limits<double>::max()) {
  
  // 声明参数
  this->declare_parameter("pcd_path", std::string(""));
  this->declare_parameter("map_frame_id", std::string("map"));
  this->declare_parameter("odom_frame_id", std::string("odom"));
  this->declare_parameter("range_odom_frame_id", std::string("lidar_odom"));
  this->declare_parameter("laser_frame_id", std::string("livox_frame"));
  this->declare_parameter("pointcloud_topic", std::string("/livox/lidar/pointcloud"));
  this->declare_parameter("registration_type", std::string("GICP"));
  
  // 配准参数
  this->declare_parameter("downsampling_resolution", 0.25);
  this->declare_parameter("source_downsampling_resolution", -1.0);
  this->declare_parameter("max_correspondence_distance", 1.0);
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("convergence_threshold", 0.01);
  this->declare_parameter("max_iterations", 30);
  this->declare_parameter("tf_future_tolerance", 0.2);
  this->declare_parameter("max_acceptable_fitness_score", 0.08);
  this->declare_parameter("initialpose_wait_timeout_sec", 30.0);
  this->declare_parameter("min_initialpose_interval_sec", 0.8);
  this->declare_parameter("bootstrap_skip_scans", 20);
  this->declare_parameter("min_source_points_for_registration", 80);
  this->declare_parameter("planarize_reference_map", true);
  this->declare_parameter("planarize_source_scan", true);
  this->declare_parameter("recovery_trigger_failures", 3);
  this->declare_parameter("recovery_xy_search_range", 0.6);
  this->declare_parameter("recovery_yaw_search_range", 20.0);  // degrees
  this->declare_parameter("recovery_max_correspondence_distance", 4.0);
  this->declare_parameter("recovery_acceptable_fitness_score", 0.2);
  
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
  active_cloud_frame_id_ = laser_frame_id_;
  pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
  registration_type_ = this->get_parameter("registration_type").as_string();
  
  downsampling_resolution_ = this->get_parameter("downsampling_resolution").as_double();
  source_downsampling_resolution_ =
      this->get_parameter("source_downsampling_resolution").as_double();
  if (source_downsampling_resolution_ <= 0.0) {
    source_downsampling_resolution_ = downsampling_resolution_;
  }
  max_correspondence_distance_ = this->get_parameter("max_correspondence_distance").as_double();
  num_threads_ = this->get_parameter("num_threads").as_int();
  convergence_threshold_ = this->get_parameter("convergence_threshold").as_double();
  max_iterations_ = this->get_parameter("max_iterations").as_int();
  tf_future_tolerance_ = this->get_parameter("tf_future_tolerance").as_double();
  max_acceptable_fitness_score_ =
      this->get_parameter("max_acceptable_fitness_score").as_double();
  initialpose_wait_timeout_sec_ =
      this->get_parameter("initialpose_wait_timeout_sec").as_double();
  min_initialpose_interval_sec_ =
      this->get_parameter("min_initialpose_interval_sec").as_double();
  bootstrap_skip_scans_ = this->get_parameter("bootstrap_skip_scans").as_int();
  bootstrap_skip_remaining_ = 0;
  min_source_points_for_registration_ =
      this->get_parameter("min_source_points_for_registration").as_int();
  planarize_reference_map_ = this->get_parameter("planarize_reference_map").as_bool();
  planarize_source_scan_ = this->get_parameter("planarize_source_scan").as_bool();
  recovery_trigger_failures_ = this->get_parameter("recovery_trigger_failures").as_int();
  recovery_xy_search_range_ = this->get_parameter("recovery_xy_search_range").as_double();
  recovery_yaw_search_range_ =
      this->get_parameter("recovery_yaw_search_range").as_double() * M_PI / 180.0;
  recovery_max_correspondence_distance_ =
      this->get_parameter("recovery_max_correspondence_distance").as_double();
  recovery_acceptable_fitness_score_ =
      this->get_parameter("recovery_acceptable_fitness_score").as_double();
  
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
  startup_time_ = this->now();

  // 初始化small_gicp配准器
  registration_ = std::make_unique<small_gicp::RegistrationPCL<PointType, PointType>>();
  registration_->setNumThreads(num_threads_);
  registration_->setMaxCorrespondenceDistance(max_correspondence_distance_);
  registration_->setMaximumIterations(max_iterations_);
  const std::array<std::string, 2> supported_types = {"GICP", "VGICP"};
  if (std::find(supported_types.begin(), supported_types.end(), registration_type_) ==
      supported_types.end()) {
    RCLCPP_WARN(this->get_logger(),
                "registration_type=%s is not supported by this small_gicp build, fallback to GICP",
                registration_type_.c_str());
    registration_type_ = "GICP";
  }
  registration_->setRegistrationType(registration_type_);  // "GICP", "VGICP"

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

    // Keep /initialpose QoS compatible with simulation tooling (BEST_EFFORT + VOLATILE).
    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose",
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      std::bind(&SmallGicpNode::initialPoseCallback, this, std::placeholders::_1));

    // Continuously publish the latest map->odom TF so downstream nodes can query at "now".
    tf_publisher_thread_ = std::make_unique<std::thread>([this]() {
      rclcpp::Rate rate(100);
      while (rclcpp::ok()) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (is_initialized_ && map_to_odom_.header.frame_id == map_frame_id_ && map_to_odom_.child_frame_id == odom_frame_id_) {
            map_to_odom_.header.stamp = this->now() + rclcpp::Duration::from_seconds(tf_future_tolerance_);
            tf_broadcaster_->sendTransform(map_to_odom_);
          }
        }
        rate.sleep();
      }
    });

  RCLCPP_INFO(this->get_logger(), "Small GICP localization node initialized successfully");
  RCLCPP_INFO(this->get_logger(), "  Map file: %s", pcd_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Downsampling resolution: %.3f", downsampling_resolution_);
  RCLCPP_INFO(this->get_logger(), "  Source downsampling resolution: %.3f",
              source_downsampling_resolution_);
  RCLCPP_INFO(this->get_logger(), "  Registration type: %s", registration_type_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Max acceptable fitness score: %.3f",
              max_acceptable_fitness_score_);
  RCLCPP_INFO(this->get_logger(), "  Initialpose wait timeout: %.1fs",
              initialpose_wait_timeout_sec_);
  RCLCPP_INFO(this->get_logger(), "  Min /initialpose interval: %.1fs",
              min_initialpose_interval_sec_);
  RCLCPP_INFO(this->get_logger(), "  Recovery trigger failures: %d",
              recovery_trigger_failures_);
  RCLCPP_INFO(this->get_logger(), "  Planarize map/source: %s / %s",
              planarize_reference_map_ ? "true" : "false",
              planarize_source_scan_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  Recovery search window: xy=%.2fm yaw=%.1fdeg",
              recovery_xy_search_range_, recovery_yaw_search_range_ * 180.0 / M_PI);
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
  if (planarize_reference_map_) {
    for (auto& pt : downsampled_map->points) {
      pt.z = 0.0f;
    }
  }
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
  if (hasPointField(*msg, "intensity")) {
    pcl::fromROSMsg(*msg, *current_scan_);
  } else {
    // Simulation pointcloud may not include intensity; synthesize intensity=0.
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    pcl::fromROSMsg(*msg, cloud_xyz);
    current_scan_->clear();
    current_scan_->reserve(cloud_xyz.size());
    for (const auto& pt : cloud_xyz.points) {
      PointType out{};
      out.x = pt.x;
      out.y = pt.y;
      out.z = pt.z;
      out.intensity = 0.0f;
      current_scan_->push_back(out);
    }
    current_scan_->width = cloud_xyz.width;
    current_scan_->height = cloud_xyz.height;
    current_scan_->is_dense = cloud_xyz.is_dense;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "Pointcloud has no intensity field; using intensity=0 fallback.");
  }

  if (current_scan_->empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
    return;
  }
  std::vector<int> finite_indices;
  pcl::removeNaNFromPointCloud(*current_scan_, *current_scan_, finite_indices);
  if (current_scan_->empty()) {
    RCLCPP_WARN(this->get_logger(), "All points are invalid after NaN filtering");
    return;
  }
  if (planarize_source_scan_) {
    for (auto& pt : current_scan_->points) {
      pt.z = 0.0f;
    }
  }

  if (!msg->header.frame_id.empty() && msg->header.frame_id != active_cloud_frame_id_) {
    RCLCPP_INFO(this->get_logger(),
                "Pointcloud frame switch: %s -> %s",
                active_cloud_frame_id_.c_str(), msg->header.frame_id.c_str());
    active_cloud_frame_id_ = msg->header.frame_id;
  }
  const std::string cloud_frame =
      active_cloud_frame_id_.empty() ? laser_frame_id_ : active_cloud_frame_id_;
  auto compute_initial_map_to_laser = [&]() {
    // Convention: T_A_B transforms coordinates from frame B into frame A.
    Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
    map_to_base.translation() = Eigen::Vector3d(
        initial_pose_.position.x,
        initial_pose_.position.y,
        initial_pose_.position.z
    );
    map_to_base.linear() = Eigen::Quaterniond(
        initial_pose_.orientation.w,
        initial_pose_.orientation.x,
        initial_pose_.orientation.y,
        initial_pose_.orientation.z
    ).toRotationMatrix();
    Eigen::Isometry3d base_to_laser = Eigen::Isometry3d::Identity();
    try {
      // target=base_link, source=cloud -> T_base_laser
      auto tf_msg = tf_buffer_->lookupTransform(
          "base_link", cloud_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.2));
      base_to_laser = transformStampedToEigen(tf_msg);
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF lookup failed for base->laser, fallback identity: %s", ex.what());
    }
    return map_to_base * base_to_laser;
  };

  // Apply cached /initialpose once when the first valid scan becomes available.
  // Clear pending flags up front to avoid relocalization loops on repeated failures.
  if (has_pending_initialpose_ && pending_initialpose_) {
    auto pending_msg = pending_initialpose_;
    pending_initialpose_.reset();
    has_pending_initialpose_ = false;
    initialPoseCallback(pending_msg);
  }

  // 如果是第一帧或者未初始化，使用初始位姿
  if (first_scan_ || !is_initialized_) {
    Eigen::Isometry3d init_guess = compute_initial_map_to_laser();

    // Bootstrap first so map->odom is always available for Nav2 startup.
    publishTransform(init_guess);
    is_initialized_ = true;
    first_scan_ = false;
    consecutive_registration_failures_ = 0;
    bootstrap_skip_remaining_ = std::max(0, bootstrap_skip_scans_);
    RCLCPP_INFO(this->get_logger(),
                "Bootstrap map->odom from initial_pose(base_link x=%.2f, y=%.2f). "
                "Registration refinement starts from next scan.",
                initial_pose_.position.x, initial_pose_.position.y);
    return;
  } else {
    if (!has_received_initialpose_) {
      publishTransform(compute_initial_map_to_laser());
      const double wait_sec = (this->now() - startup_time_).seconds();
      if (initialpose_wait_timeout_sec_ > 0.0 &&
          wait_sec >= initialpose_wait_timeout_sec_) {
        has_received_initialpose_ = true;
        RCLCPP_WARN(this->get_logger(),
                    "No /initialpose received after %.1fs, continue with configured initial_pose.",
                    wait_sec);
      } else {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Waiting for /initialpose before starting registration");
        return;
      }
    }
    if (bootstrap_skip_remaining_ > 0) {
      // Keep map pose anchored to the configured initial_pose during warmup.
      publishTransform(compute_initial_map_to_laser());
      bootstrap_skip_remaining_--;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Skipping registration warmup, remaining scans: %d",
                           bootstrap_skip_remaining_);
      return;
    }

    // Use last map->odom and current odom->base->laser as init guess for map->laser.
    Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();
    try {
      // target=odom, source=base_link -> T_odom_base
      auto tf_odom_base_msg = tf_buffer_->lookupTransform(
          odom_frame_id_, "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.2));
      const Eigen::Isometry3d odom_to_base = transformStampedToEigen(tf_odom_base_msg);

      // target=base_link, source=cloud -> T_base_laser
      auto tf_base_laser_msg = tf_buffer_->lookupTransform(
          "base_link", cloud_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.2));
      const Eigen::Isometry3d base_to_laser = transformStampedToEigen(tf_base_laser_msg);

      Eigen::Isometry3d map_to_odom = Eigen::Isometry3d::Identity();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        // Only use the last published transform when it's valid.
        if (map_to_odom_.header.frame_id == map_frame_id_ && map_to_odom_.child_frame_id == odom_frame_id_) {
          map_to_odom = transformStampedToEigen(map_to_odom_);
        }
      }

      // map->laser = (map->odom) * (odom->base) * (base->laser)
      init_guess = map_to_odom * odom_to_base * base_to_laser;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF lookup failed for init guess: %s", ex.what());
    }
    
    const bool recovery_active =
        recovery_trigger_failures_ > 0 &&
        consecutive_registration_failures_ >= recovery_trigger_failures_;
    double active_xy_search = xy_search_range_;
    double active_yaw_search = yaw_search_range_;
    double active_max_corr = max_correspondence_distance_;
    double active_score_threshold = max_acceptable_fitness_score_;
    if (recovery_active) {
      active_xy_search = std::max(active_xy_search, recovery_xy_search_range_);
      active_yaw_search = std::max(active_yaw_search, recovery_yaw_search_range_);
      active_max_corr = std::max(active_max_corr, recovery_max_correspondence_distance_);
      active_score_threshold =
          std::max(active_score_threshold, recovery_acceptable_fitness_score_);
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Registration recovery mode: fail_streak=%d, xy=%.2fm, yaw=%.1fdeg, corr=%.2f, score<=%.3f",
          consecutive_registration_failures_, active_xy_search,
          active_yaw_search * 180.0 / M_PI, active_max_corr, active_score_threshold);
    }

    registration_->setMaxCorrespondenceDistance(active_max_corr);
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "small-gicp init_guess(map->%s): x=%.2f y=%.2f yaw=%.1fdeg",
        cloud_frame.c_str(), init_guess.translation().x(), init_guess.translation().y(),
        planarYaw(init_guess) * 180.0 / M_PI);
    auto [transform, success] = performRegistration(
        current_scan_, init_guess, active_xy_search, active_yaw_search);
    
    if (success && last_registration_score_ <= active_score_threshold) {
      publishTransform(transform);
      consecutive_registration_failures_ = 0;
    } else {
      if (success && last_registration_score_ > active_score_threshold) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Reject registration update: score %.5f exceeds threshold %.5f",
            last_registration_score_, active_score_threshold);
      }
      consecutive_registration_failures_++;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Registration failed");
      publishPredictedTransformFromOdom();
    }
  }
}

void SmallGicpNode::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  const rclcpp::Time now = this->now();
  if (last_initialpose_time_.nanoseconds() > 0 &&
      min_initialpose_interval_sec_ > 0.0 &&
      (now - last_initialpose_time_).seconds() < min_initialpose_interval_sec_) {
    return;
  }
  last_initialpose_time_ = now;
  
  RCLCPP_INFO(this->get_logger(), "Received initial pose, performing relocalization");
  has_received_initialpose_ = true;

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
  Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
  map_to_base.translation() = Eigen::Vector3d(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z
  );
  map_to_base.linear() = Eigen::Quaterniond(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z
  ).toRotationMatrix();
  Eigen::Isometry3d base_to_laser = Eigen::Isometry3d::Identity();
  const std::string cloud_frame =
      active_cloud_frame_id_.empty() ? laser_frame_id_ : active_cloud_frame_id_;
  try {
    // target=base_link, source=cloud -> T_base_laser
    auto tf_msg = tf_buffer_->lookupTransform(
        "base_link", cloud_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.2));
    base_to_laser = transformStampedToEigen(tf_msg);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "TF lookup failed for base->laser during relocalization: %s", ex.what());
  }
  Eigen::Isometry3d init_guess = map_to_base * base_to_laser;

  // Keep /initialpose handling deterministic and non-blocking:
  // directly bootstrap map->odom here, then let periodic scan callbacks refine.
  publishTransform(init_guess);
  is_initialized_ = true;
  first_scan_ = false;
  consecutive_registration_failures_ = 0;
  bootstrap_skip_remaining_ = std::max(0, bootstrap_skip_scans_);
  pending_initialpose_.reset();
  has_pending_initialpose_ = false;
  RCLCPP_INFO(this->get_logger(),
              "Applied /initialpose bootstrap (x=%.2f, y=%.2f), defer refinement to scan callback.",
              msg->pose.pose.position.x, msg->pose.pose.position.y);
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
    const PointCloudPtr& source,
    const Eigen::Isometry3d& init_guess,
    double xy_search_range,
    double yaw_search_range) {
  const auto start_time = std::chrono::steady_clock::now();
  
  // 对输入点云进行降采样
  auto downsampled_source = small_gicp::voxelgrid_sampling(*source, source_downsampling_resolution_);
  if (static_cast<int>(downsampled_source->size()) < min_source_points_for_registration_) {
    last_registration_score_ = std::numeric_limits<double>::max();
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Skip registration: source too sparse after downsampling (%zu < %d)",
        downsampled_source->size(), min_source_points_for_registration_);
    return {init_guess, false};
  }
  
  // 生成候选位姿
  auto candidate_poses = generateCandidatePoses(init_guess, xy_search_range, yaw_search_range);
  if (candidate_poses.empty()) {
    candidate_poses.push_back(init_guess);
  }
  
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
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time).count();
  
  if (success) {
    RCLCPP_DEBUG(this->get_logger(), "Registration converged with score: %.6f", best_score);
  }
  RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "small-gicp iter: src=%zu ds=%zu candidates=%zu success=%d score=%.5f elapsed=%ldms",
      source->size(), downsampled_source->size(), candidate_poses.size(), success ? 1 : 0,
      success ? best_score : -1.0, elapsed_ms);

  return {best_transform, success};
}

std::vector<Eigen::Isometry3d> SmallGicpNode::generateCandidatePoses(
    const Eigen::Isometry3d& init_guess,
    double xy_search_range,
    double yaw_search_range) {
  
  std::vector<Eigen::Isometry3d> candidates;
  if (xy_search_range <= 0.0 && yaw_search_range <= 0.0) {
    candidates.push_back(init_guess);
    return candidates;
  }
  if (xy_step_ <= 0.0 || yaw_step_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(),
                "Invalid search steps (xy_step=%.3f yaw_step=%.3f); fallback to single candidate",
                xy_step_, yaw_step_);
    candidates.push_back(init_guess);
    return candidates;
  }
  
  // 提取初始位置和朝向
  Eigen::Vector3d init_position = init_guess.translation();
  Eigen::Matrix3d init_rotation = init_guess.linear();
  
  // 从旋转矩阵中提取欧拉角
  Eigen::Vector3d euler = init_rotation.eulerAngles(0, 1, 2); // Roll, Pitch, Yaw
  double init_yaw = euler(2);

  // 在XY平面和Yaw角度周围生成候选位姿
  for (double dx = -xy_search_range; dx <= xy_search_range; dx += xy_step_) {
    for (double dy = -xy_search_range; dy <= xy_search_range; dy += xy_step_) {
      for (double dyaw = -yaw_search_range; dyaw <= yaw_search_range; dyaw += yaw_step_) {
        
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
  const std::string cloud_frame =
      active_cloud_frame_id_.empty() ? laser_frame_id_ : active_cloud_frame_id_;
  Eigen::Isometry3d laser_to_base = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d base_to_odom = Eigen::Isometry3d::Identity();
  bool fresh_base_to_odom = false;

  try {
    // target=cloud, source=base_link -> T_laser_base
    auto tf_laser_base_msg = tf_buffer_->lookupTransform(
        cloud_frame, "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
    laser_to_base = transformStampedToEigen(tf_laser_base_msg);
    // target=base_link, source=odom -> T_base_odom
    auto tf_base_odom_msg = tf_buffer_->lookupTransform(
        "base_link", odom_frame_id_, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
    base_to_odom = transformStampedToEigen(tf_base_odom_msg);
    fresh_base_to_odom = true;
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Transform lookup failed, trying cached fallback: %s", ex.what());
  }

  if (!fresh_base_to_odom) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_last_base_to_odom_) {
      base_to_odom = last_base_to_odom_;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Using cached base->odom fallback for map->odom publishing");
    } else {
      // Bootstrap fallback: even when odom TF is not ready yet, publish a usable map->odom.
      base_to_odom = Eigen::Isometry3d::Identity();
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Using identity base->odom fallback for bootstrap");
    }
  }

  // map->odom = (map->laser) * (laser->base) * (base->odom)
  Eigen::Isometry3d map_to_odom = projectToPlanar(map_to_laser * laser_to_base * base_to_odom);
  geometry_msgs::msg::TransformStamped map_to_odom_msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fresh_base_to_odom) {
      last_base_to_odom_ = base_to_odom;
      has_last_base_to_odom_ = true;
    }
    map_to_odom_ = eigenToTransformStamped(
        map_to_odom,
        map_frame_id_,
        odom_frame_id_,
        this->get_clock()->now() + rclcpp::Duration::from_seconds(tf_future_tolerance_));
    map_to_odom_msg = map_to_odom_;
  }
  RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "publish map->odom: x=%.2f y=%.2f yaw=%.1fdeg (map->%s x=%.2f y=%.2f)",
      map_to_odom.translation().x(), map_to_odom.translation().y(),
      planarYaw(map_to_odom) * 180.0 / M_PI, cloud_frame.c_str(),
      map_to_laser.translation().x(), map_to_laser.translation().y());
  tf_broadcaster_->sendTransform(map_to_odom_msg);
}

void SmallGicpNode::publishPredictedTransformFromOdom() {
  geometry_msgs::msg::TransformStamped map_to_odom_msg;
  bool has_ready_transform = false;

  // Keep map->odom continuous when registration fails.
  // Important: don't recompute from a stale map->laser snapshot, otherwise
  // map pose gets frozen and Nav2 reports "Failed to make progress".
  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_ready_transform =
        is_initialized_ &&
        map_to_odom_.header.frame_id == map_frame_id_ &&
        map_to_odom_.child_frame_id == odom_frame_id_;
    if (!has_ready_transform) {
      return;
    }
    map_to_odom_msg = map_to_odom_;
    map_to_odom_msg.header.stamp =
        this->get_clock()->now() + rclcpp::Duration::from_seconds(tf_future_tolerance_);
    map_to_odom_ = map_to_odom_msg;
  }

  if (!has_ready_transform) {
    return;
  }

  tf_broadcaster_->sendTransform(map_to_odom_msg);
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
