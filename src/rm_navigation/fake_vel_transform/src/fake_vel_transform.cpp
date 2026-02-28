#include "fake_vel_transform/fake_vel_transform.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <string>

#include <tf2/utils.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

namespace fake_vel_transform
{
FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  // Runtime parameters
  this->declare_parameter<float>("spin_speed", -6.0);
  this->declare_parameter<bool>("use_local_plan_transform", true);
  this->declare_parameter<int>("tf_publish_frequency", 20);
  this->declare_parameter<double>("local_plan_timeout_sec", 0.5);
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("fake_base_frame", "base_link_fake");
  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter<std::string>("cmd_vel_out_topic", "/cmd_vel_chassis");
  this->declare_parameter<std::string>("local_plan_topic", "/local_plan");
  this->declare_parameter<bool>("follow_mark_enable", false);
  this->declare_parameter<std::string>("follow_mark_topic", "/chassis/follow_mark");
  this->declare_parameter<std::string>("follow_mark_manual_topic", "/chassis/follow_mark_manual");
  this->declare_parameter<std::string>("follow_mark_hint_topic", "/chassis/follow_mark_hint");
  this->declare_parameter<std::string>("follow_mark_mode", "off");
  this->declare_parameter<double>("follow_mark_input_stale_timeout_sec", 0.3);
  this->declare_parameter<double>("follow_mark_enter_margin_m", 0.35);
  this->declare_parameter<double>("follow_mark_exit_margin_m", 0.55);
  this->declare_parameter<std::vector<double>>("follow_mark_zone_rects", std::vector<double>{});
  this->declare_parameter<std::vector<double>>("follow_mark_zone_polygon_points", std::vector<double>{});
  this->declare_parameter<std::vector<int64_t>>("follow_mark_zone_polygon_sizes", std::vector<int64_t>{});
  this->declare_parameter<int>("follow_mark_default_value", 1);
  this->declare_parameter<int>("follow_mark_zone_value", 1);

  this->get_parameter("spin_speed", spin_speed_);
  this->get_parameter("use_local_plan_transform", use_local_plan_transform_);
  this->get_parameter("tf_publish_frequency", tf_publish_frequency_);
  this->get_parameter("local_plan_timeout_sec", local_plan_timeout_sec_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("fake_base_frame", fake_base_frame_);
  this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
  this->get_parameter("cmd_vel_out_topic", cmd_vel_out_topic_);
  this->get_parameter("local_plan_topic", local_plan_topic_);
  this->get_parameter("follow_mark_enable", follow_mark_enable_);
  this->get_parameter("follow_mark_topic", follow_mark_topic_);
  this->get_parameter("follow_mark_manual_topic", follow_mark_manual_topic_);
  this->get_parameter("follow_mark_hint_topic", follow_mark_hint_topic_);
  this->get_parameter("follow_mark_mode", follow_mark_mode_);
  this->get_parameter("follow_mark_input_stale_timeout_sec", follow_mark_input_stale_timeout_sec_);
  this->get_parameter("follow_mark_enter_margin_m", follow_mark_enter_margin_m_);
  this->get_parameter("follow_mark_exit_margin_m", follow_mark_exit_margin_m_);
  this->get_parameter("follow_mark_zone_rects", follow_mark_zone_rects_);
  this->get_parameter("follow_mark_zone_polygon_points", follow_mark_zone_polygon_points_);
  this->get_parameter("follow_mark_zone_polygon_sizes", follow_mark_zone_polygon_sizes_);
  int follow_mark_default_value_raw = 1;
  int follow_mark_zone_value_raw = 1;
  this->get_parameter("follow_mark_default_value", follow_mark_default_value_raw);
  this->get_parameter("follow_mark_zone_value", follow_mark_zone_value_raw);
  follow_mark_default_value_ = sanitizeMark(follow_mark_default_value_raw, 1U, "follow_mark_default_value");
  follow_mark_zone_value_ =
    sanitizeMark(follow_mark_zone_value_raw, follow_mark_default_value_, "follow_mark_zone_value");

  std::transform(
    follow_mark_mode_.begin(), follow_mark_mode_.end(), follow_mark_mode_.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  if (
    follow_mark_mode_ != "off" &&
    follow_mark_mode_ != "zone" &&
    follow_mark_mode_ != "hint" &&
    follow_mark_mode_ != "zone_and_hint")
  {
    RCLCPP_WARN(
      get_logger(), "Invalid follow_mark_mode='%s', fallback to 'off'", follow_mark_mode_.c_str());
    follow_mark_mode_ = "off";
  }

  if (tf_publish_frequency_ <= 0) {
    RCLCPP_WARN(get_logger(), "tf_publish_frequency <= 0, fallback to 20 Hz");
    tf_publish_frequency_ = 20;
  }
  if (local_plan_timeout_sec_ < 0.0) {
    RCLCPP_WARN(get_logger(), "local_plan_timeout_sec < 0, fallback to 0.5 s");
    local_plan_timeout_sec_ = 0.5;
  }
  if (follow_mark_input_stale_timeout_sec_ < 0.0) {
    RCLCPP_WARN(
      get_logger(), "follow_mark_input_stale_timeout_sec < 0, fallback to 0.3 s");
    follow_mark_input_stale_timeout_sec_ = 0.3;
  }
  if (follow_mark_enter_margin_m_ < 0.0) {
    RCLCPP_WARN(get_logger(), "follow_mark_enter_margin_m < 0, fallback to 0.35 m");
    follow_mark_enter_margin_m_ = 0.35;
  }
  if (follow_mark_exit_margin_m_ < 0.0) {
    RCLCPP_WARN(get_logger(), "follow_mark_exit_margin_m < 0, fallback to 0.55 m");
    follow_mark_exit_margin_m_ = 0.55;
  }
  if (follow_mark_exit_margin_m_ < follow_mark_enter_margin_m_) {
    RCLCPP_WARN(
      get_logger(),
      "follow_mark_exit_margin_m(%.3f) < follow_mark_enter_margin_m(%.3f), clamp to enter margin",
      follow_mark_exit_margin_m_, follow_mark_enter_margin_m_);
    follow_mark_exit_margin_m_ = follow_mark_enter_margin_m_;
  }
  if ((follow_mark_zone_rects_.size() % 4U) != 0U) {
    const size_t valid_size = (follow_mark_zone_rects_.size() / 4U) * 4U;
    RCLCPP_WARN(
      get_logger(),
      "follow_mark_zone_rects length=%zu is not multiple of 4, truncate to %zu",
      follow_mark_zone_rects_.size(), valid_size);
    follow_mark_zone_rects_.resize(valid_size);
  }
  if ((follow_mark_zone_polygon_points_.size() % 2U) != 0U) {
    const size_t valid_size = (follow_mark_zone_polygon_points_.size() / 2U) * 2U;
    RCLCPP_WARN(
      get_logger(),
      "follow_mark_zone_polygon_points length=%zu is not multiple of 2, truncate to %zu",
      follow_mark_zone_polygon_points_.size(), valid_size);
    follow_mark_zone_polygon_points_.resize(valid_size);
  }
  for (const auto size : follow_mark_zone_polygon_sizes_) {
    if (size < 3) {
      RCLCPP_WARN(
        get_logger(),
        "follow_mark_zone_polygon_sizes contains invalid polygon size=%ld (<3), polygons will be rebuilt with available valid data only",
        static_cast<long>(size));
    }
  }
  rebuildZonePolygons();

  last_local_plan_stamp_ = this->now();
  last_follow_mark_manual_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  last_follow_mark_hint_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

  // TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // Create Publisher and Subscriber
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, 1, std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));
  cmd_vel_chassis_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_out_topic_, rclcpp::QoS(rclcpp::KeepLast(1)));
  local_pose_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    local_plan_topic_, 1,
    std::bind(&FakeVelTransform::localPoseCallback, this, std::placeholders::_1));
  if (follow_mark_enable_) {
    const auto follow_mark_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    follow_mark_pub_ =
      this->create_publisher<std_msgs::msg::UInt8>(follow_mark_topic_, follow_mark_qos);
    follow_mark_manual_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      follow_mark_manual_topic_, follow_mark_qos,
      std::bind(&FakeVelTransform::followMarkManualCallback, this, std::placeholders::_1));
    follow_mark_hint_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      follow_mark_hint_topic_, follow_mark_qos,
      std::bind(&FakeVelTransform::followMarkHintCallback, this, std::placeholders::_1));
  }

  // Create a timer to publish the transform regularly
  tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / tf_publish_frequency_),
    std::bind(&FakeVelTransform::publishTransform, this));

  RCLCPP_INFO(
    get_logger(),
    "Frames: %s -> %s, lookup %s -> %s, local_plan_timeout=%.2fs, use_local_plan_transform=%s",
    base_frame_.c_str(), fake_base_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str(),
    local_plan_timeout_sec_, use_local_plan_transform_ ? "true" : "false");
  RCLCPP_INFO(
    get_logger(),
    "FollowMark: enable=%s mode=%s default=%u zone=%u zones=%zu stale=%.2fs",
    follow_mark_enable_ ? "true" : "false", follow_mark_mode_.c_str(), follow_mark_default_value_,
    follow_mark_zone_value_, follow_mark_zone_polygons_.size(),
    follow_mark_input_stale_timeout_sec_);
}

// Get the local pose from planner
void FakeVelTransform::localPoseCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!msg || msg->poses.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Received empty or invalid Path message");
    return;
  }

  // Choose the pose based on the size of the poses array
  size_t index = std::min(msg->poses.size() / 4, msg->poses.size() - 1);
  const geometry_msgs::msg::Pose & selected_pose = msg->poses[index].pose;

  // Update current angle based on the difference between teb_angle and base_link_angle_
  double teb_angle = tf2::getYaw(selected_pose.orientation);
  const double raw_angle = teb_angle - base_link_angle_;
  current_angle_ = std::atan2(std::sin(raw_angle), std::cos(raw_angle));
  has_local_plan_ = true;

  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
    last_local_plan_stamp_ = this->now();
  } else {
    last_local_plan_stamp_ = rclcpp::Time(msg->header.stamp);
  }
}

// Transform the velocity from base_link to base_link_fake
void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  try {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf2_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
    base_link_angle_ = tf2::getYaw(transform_stamped.transform.rotation);

    const auto now = this->now();
    const bool local_plan_fresh =
      use_local_plan_transform_ && has_local_plan_ &&
      ((now - last_local_plan_stamp_).seconds() <= local_plan_timeout_sec_);
    const double angle_diff = local_plan_fresh ? -current_angle_ : 0.0;
    if (use_local_plan_transform_ && !local_plan_fresh) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No fresh local plan, fallback to base frame cmd_vel passthrough");
    }

    geometry_msgs::msg::Twist aft_tf_vel;
    // spin_speed_ != 0 keeps legacy fixed-spin behavior; spin_speed_ == 0 passthroughs planner yaw.
    if (std::abs(spin_speed_) > 1e-6F) {
      aft_tf_vel.angular.z = (msg->angular.z != 0.0F) ? spin_speed_ : 0.0F;
    } else {
      aft_tf_vel.angular.z = msg->angular.z;
    }
    aft_tf_vel.linear.x = msg->linear.x * cos(angle_diff) + msg->linear.y * sin(angle_diff);
    aft_tf_vel.linear.y = -msg->linear.x * sin(angle_diff) + msg->linear.y * cos(angle_diff);

    cmd_vel_chassis_pub_->publish(aft_tf_vel);

    if (follow_mark_enable_ && follow_mark_pub_) {
      std_msgs::msg::UInt8 follow_mark_msg;
      follow_mark_msg.data = computeFollowMark(transform_stamped, now);
      follow_mark_pub_->publish(follow_mark_msg);
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform odom to base_link: %s", ex.what());
  }
}

// Publish transform from base_link to base_link_fake
void FakeVelTransform::publishTransform()
{
  const auto now = this->now();
  const bool local_plan_fresh =
    use_local_plan_transform_ && has_local_plan_ &&
    ((now - last_local_plan_stamp_).seconds() <= local_plan_timeout_sec_);
  const double publish_yaw = local_plan_fresh ? current_angle_ : 0.0;

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now;
  t.header.frame_id = base_frame_;
  t.child_frame_id = fake_base_frame_;
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, publish_yaw);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

void FakeVelTransform::followMarkManualCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  follow_mark_manual_ = sanitizeMark(
    static_cast<int>(msg->data), follow_mark_default_value_, "follow_mark_manual");
  has_follow_mark_manual_ = true;
  last_follow_mark_manual_stamp_ = this->now();
}

void FakeVelTransform::followMarkHintCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  follow_mark_hint_ = sanitizeMark(
    static_cast<int>(msg->data), follow_mark_default_value_, "follow_mark_hint");
  has_follow_mark_hint_ = true;
  last_follow_mark_hint_stamp_ = this->now();
}

uint8_t FakeVelTransform::computeFollowMark(
  const geometry_msgs::msg::TransformStamped & odom_to_base, const rclcpp::Time & now)
{
  if (follow_mark_mode_ == "off") {
    return follow_mark_default_value_;
  }

  if (has_follow_mark_manual_ && isFreshMarkInput(last_follow_mark_manual_stamp_, now)) {
    return follow_mark_manual_;
  }

  if (
    (follow_mark_mode_ == "hint" || follow_mark_mode_ == "zone_and_hint") &&
    has_follow_mark_hint_ && isFreshMarkInput(last_follow_mark_hint_stamp_, now))
  {
    return follow_mark_hint_;
  }

  if (follow_mark_mode_ == "zone" || follow_mark_mode_ == "zone_and_hint") {
    updateZoneLatch(
      odom_to_base.transform.translation.x,
      odom_to_base.transform.translation.y);
    if (follow_mark_zone_latched_) {
      return follow_mark_zone_value_;
    }
  }

  return follow_mark_default_value_;
}

bool FakeVelTransform::isFreshMarkInput(const rclcpp::Time & stamp, const rclcpp::Time & now) const
{
  if (stamp.nanoseconds() == 0) {
    return false;
  }
  return (now - stamp).seconds() <= follow_mark_input_stale_timeout_sec_;
}

bool FakeVelTransform::pointInAnyZone(double x, double y, double margin_m) const
{
  if (follow_mark_zone_polygons_.empty()) {
    return false;
  }

  for (const auto & polygon : follow_mark_zone_polygons_) {
    const double x_min = polygon.min_x - margin_m;
    const double x_max = polygon.max_x + margin_m;
    const double y_min = polygon.min_y - margin_m;
    const double y_max = polygon.max_y + margin_m;
    if (x >= x_min && x <= x_max && y >= y_min && y <= y_max) {
      if (pointInPolygon(x, y, polygon)) {
        return true;
      }
      if (margin_m > 0.0 && pointNearPolygonEdges(x, y, polygon, margin_m)) {
        return true;
      }
    }
  }
  return false;
}

void FakeVelTransform::updateZoneLatch(double x, double y)
{
  if (follow_mark_zone_polygons_.empty()) {
    follow_mark_zone_latched_ = false;
    return;
  }

  if (follow_mark_zone_latched_) {
    follow_mark_zone_latched_ = pointInAnyZone(x, y, follow_mark_exit_margin_m_);
  } else {
    follow_mark_zone_latched_ = pointInAnyZone(x, y, follow_mark_enter_margin_m_);
  }
}

bool FakeVelTransform::pointInPolygon(double x, double y, const ZonePolygon & polygon) const
{
  const size_t n = polygon.vertices.size();
  if (n < 3U) {
    return false;
  }

  bool inside = false;
  for (size_t i = 0U, j = n - 1U; i < n; j = i++) {
    const auto & vi = polygon.vertices[i];
    const auto & vj = polygon.vertices[j];
    const bool crosses = ((vi.y > y) != (vj.y > y));
    if (!crosses) {
      continue;
    }
    const double x_on_edge = (vj.x - vi.x) * (y - vi.y) / (vj.y - vi.y) + vi.x;
    if (x < x_on_edge) {
      inside = !inside;
    }
  }
  return inside;
}

bool FakeVelTransform::pointNearPolygonEdges(
  double x, double y, const ZonePolygon & polygon, double margin_m) const
{
  const size_t n = polygon.vertices.size();
  if (n < 2U || margin_m <= 0.0) {
    return false;
  }
  const double margin_sq = margin_m * margin_m;
  for (size_t i = 0U; i < n; ++i) {
    const auto & a = polygon.vertices[i];
    const auto & b = polygon.vertices[(i + 1U) % n];
    if (pointToSegmentDistanceSquared(x, y, a, b) <= margin_sq) {
      return true;
    }
  }
  return false;
}

double FakeVelTransform::pointToSegmentDistanceSquared(
  double px, double py, const ZoneVertex & a, const ZoneVertex & b) const
{
  const double vx = b.x - a.x;
  const double vy = b.y - a.y;
  const double wx = px - a.x;
  const double wy = py - a.y;
  const double c1 = vx * wx + vy * wy;
  if (c1 <= 0.0) {
    const double dx = px - a.x;
    const double dy = py - a.y;
    return dx * dx + dy * dy;
  }

  const double c2 = vx * vx + vy * vy;
  if (c2 <= 1e-12) {
    const double dx = px - a.x;
    const double dy = py - a.y;
    return dx * dx + dy * dy;
  }

  if (c1 >= c2) {
    const double dx = px - b.x;
    const double dy = py - b.y;
    return dx * dx + dy * dy;
  }

  const double t = c1 / c2;
  const double proj_x = a.x + t * vx;
  const double proj_y = a.y + t * vy;
  const double dx = px - proj_x;
  const double dy = py - proj_y;
  return dx * dx + dy * dy;
}

void FakeVelTransform::rebuildZonePolygons()
{
  follow_mark_zone_polygons_.clear();

  auto append_polygon =
    [this](const std::vector<ZoneVertex> & vertices, const char * source_name) {
      if (vertices.size() < 3U) {
        return;
      }
      ZonePolygon polygon;
      polygon.vertices = vertices;
      polygon.min_x = std::numeric_limits<double>::infinity();
      polygon.max_x = -std::numeric_limits<double>::infinity();
      polygon.min_y = std::numeric_limits<double>::infinity();
      polygon.max_y = -std::numeric_limits<double>::infinity();
      for (const auto & v : polygon.vertices) {
        polygon.min_x = std::min(polygon.min_x, v.x);
        polygon.max_x = std::max(polygon.max_x, v.x);
        polygon.min_y = std::min(polygon.min_y, v.y);
        polygon.max_y = std::max(polygon.max_y, v.y);
      }
      if (
        !std::isfinite(polygon.min_x) || !std::isfinite(polygon.max_x) ||
        !std::isfinite(polygon.min_y) || !std::isfinite(polygon.max_y))
      {
        RCLCPP_WARN(get_logger(), "Skip invalid polygon from %s", source_name);
        return;
      }
      follow_mark_zone_polygons_.push_back(std::move(polygon));
    };

  bool polygon_param_loaded = false;
  if (!follow_mark_zone_polygon_sizes_.empty() && !follow_mark_zone_polygon_points_.empty()) {
    size_t required_points = 0U;
    bool sizes_valid = true;
    for (const auto size : follow_mark_zone_polygon_sizes_) {
      if (size < 3) {
        sizes_valid = false;
        break;
      }
      required_points += static_cast<size_t>(size) * 2U;
    }

    if (!sizes_valid) {
      RCLCPP_WARN(
        get_logger(),
        "follow_mark_zone_polygon_sizes contains value <3, skip polygon parameter set");
    } else if (required_points > follow_mark_zone_polygon_points_.size()) {
      RCLCPP_WARN(
        get_logger(),
        "follow_mark_zone_polygon_points not enough: need=%zu actual=%zu, skip polygon parameter set",
        required_points, follow_mark_zone_polygon_points_.size());
    } else {
      polygon_param_loaded = true;
      size_t point_idx = 0U;
      for (const auto size : follow_mark_zone_polygon_sizes_) {
        std::vector<ZoneVertex> vertices;
        vertices.reserve(static_cast<size_t>(size));
        for (int64_t i = 0; i < size; ++i) {
          ZoneVertex vertex;
          vertex.x = follow_mark_zone_polygon_points_[point_idx++];
          vertex.y = follow_mark_zone_polygon_points_[point_idx++];
          vertices.push_back(vertex);
        }
        append_polygon(vertices, "zone_polygons");
      }
      if (required_points < follow_mark_zone_polygon_points_.size()) {
        RCLCPP_WARN(
          get_logger(),
          "follow_mark_zone_polygon_points has trailing values (used=%zu total=%zu), ignored",
          required_points, follow_mark_zone_polygon_points_.size());
      }
    }
  }

  if (!polygon_param_loaded && !follow_mark_zone_rects_.empty()) {
    for (size_t i = 0U; (i + 3U) < follow_mark_zone_rects_.size(); i += 4U) {
      const double x0 = follow_mark_zone_rects_[i];
      const double x1 = follow_mark_zone_rects_[i + 1U];
      const double y0 = follow_mark_zone_rects_[i + 2U];
      const double y1 = follow_mark_zone_rects_[i + 3U];
      std::vector<ZoneVertex> rect_vertices{
        ZoneVertex{std::min(x0, x1), std::min(y0, y1)},
        ZoneVertex{std::max(x0, x1), std::min(y0, y1)},
        ZoneVertex{std::max(x0, x1), std::max(y0, y1)},
        ZoneVertex{std::min(x0, x1), std::max(y0, y1)}
      };
      append_polygon(rect_vertices, "zone_rects");
    }
  }
}

uint8_t FakeVelTransform::sanitizeMark(int value, uint8_t fallback, const char * source) const
{
  if (value == 0 || value == 1) {
    return static_cast<uint8_t>(value);
  }

  RCLCPP_WARN(
    get_logger(), "%s=%d is invalid, fallback to %u", source, value,
    static_cast<unsigned int>(fallback));
  return fallback;
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
