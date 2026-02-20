#include "fake_vel_transform/fake_vel_transform.hpp"

#include <algorithm>
#include <cmath>
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
  this->declare_parameter<int>("tf_publish_frequency", 20);
  this->declare_parameter<double>("local_plan_timeout_sec", 0.5);
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("fake_base_frame", "base_link_fake");
  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter<std::string>("cmd_vel_out_topic", "/cmd_vel_chassis");
  this->declare_parameter<std::string>("local_plan_topic", "/local_plan");

  this->get_parameter("spin_speed", spin_speed_);
  this->get_parameter("tf_publish_frequency", tf_publish_frequency_);
  this->get_parameter("local_plan_timeout_sec", local_plan_timeout_sec_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("fake_base_frame", fake_base_frame_);
  this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
  this->get_parameter("cmd_vel_out_topic", cmd_vel_out_topic_);
  this->get_parameter("local_plan_topic", local_plan_topic_);

  if (tf_publish_frequency_ <= 0) {
    RCLCPP_WARN(get_logger(), "tf_publish_frequency <= 0, fallback to 20 Hz");
    tf_publish_frequency_ = 20;
  }
  if (local_plan_timeout_sec_ < 0.0) {
    RCLCPP_WARN(get_logger(), "local_plan_timeout_sec < 0, fallback to 0.5 s");
    local_plan_timeout_sec_ = 0.5;
  }
  last_local_plan_stamp_ = this->now();

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

  // Create a timer to publish the transform regularly
  tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / tf_publish_frequency_),
    std::bind(&FakeVelTransform::publishTransform, this));

  RCLCPP_INFO(
    get_logger(),
    "Frames: %s -> %s, lookup %s -> %s, local_plan_timeout=%.2fs",
    base_frame_.c_str(), fake_base_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str(),
    local_plan_timeout_sec_);
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
      has_local_plan_ &&
      ((now - last_local_plan_stamp_).seconds() <= local_plan_timeout_sec_);
    const double angle_diff = local_plan_fresh ? -current_angle_ : 0.0;
    if (!local_plan_fresh) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No fresh local plan, fallback to base frame cmd_vel passthrough");
    }

    geometry_msgs::msg::Twist aft_tf_vel;
    aft_tf_vel.angular.z = (msg->angular.z != 0) ? spin_speed_ : 0;
    aft_tf_vel.linear.x = msg->linear.x * cos(angle_diff) + msg->linear.y * sin(angle_diff);
    aft_tf_vel.linear.y = -msg->linear.x * sin(angle_diff) + msg->linear.y * cos(angle_diff);

    cmd_vel_chassis_pub_->publish(aft_tf_vel);
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform odom to base_link: %s", ex.what());
  }
}

// Publish transform from base_link to base_link_fake
void FakeVelTransform::publishTransform()
{
  const auto now = this->now();
  const bool local_plan_fresh =
    has_local_plan_ &&
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

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
