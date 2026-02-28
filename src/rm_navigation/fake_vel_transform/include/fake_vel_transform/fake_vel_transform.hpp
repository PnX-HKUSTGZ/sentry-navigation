#ifndef FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
#define FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_

#include <cstdint>
#include <vector>

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fake_vel_transform
{
class FakeVelTransform : public rclcpp::Node
{
public:
  explicit FakeVelTransform(const rclcpp::NodeOptions & options);

private:
  struct ZoneVertex
  {
    double x{0.0};
    double y{0.0};
  };

  struct ZonePolygon
  {
    std::vector<ZoneVertex> vertices;
    double min_x{0.0};
    double max_x{0.0};
    double min_y{0.0};
    double max_y{0.0};
  };

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  void localPoseCallback(const nav_msgs::msg::Path::SharedPtr msg);

  void publishTransform();

  void followMarkManualCallback(const std_msgs::msg::UInt8::SharedPtr msg);

  void followMarkHintCallback(const std_msgs::msg::UInt8::SharedPtr msg);

  uint8_t computeFollowMark(
    const geometry_msgs::msg::TransformStamped & odom_to_base,
    const rclcpp::Time & now);

  bool isFreshMarkInput(const rclcpp::Time & stamp, const rclcpp::Time & now) const;

  bool pointInAnyZone(double x, double y, double margin_m) const;

  void updateZoneLatch(double x, double y);

  bool pointInPolygon(double x, double y, const ZonePolygon & polygon) const;

  bool pointNearPolygonEdges(
    double x, double y, const ZonePolygon & polygon, double margin_m) const;

  double pointToSegmentDistanceSquared(
    double px, double py, const ZoneVertex & a, const ZoneVertex & b) const;

  void rebuildZonePolygons();

  uint8_t sanitizeMark(int value, uint8_t fallback, const char * source) const;

  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string fake_base_frame_{"base_link_fake"};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string cmd_vel_out_topic_{"/cmd_vel_chassis"};
  std::string local_plan_topic_{"/local_plan"};
  std::string follow_mark_topic_{"/chassis/follow_mark"};
  std::string follow_mark_manual_topic_{"/chassis/follow_mark_manual"};
  std::string follow_mark_hint_topic_{"/chassis/follow_mark_hint"};
  std::string follow_mark_mode_{"off"};

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr follow_mark_manual_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr follow_mark_hint_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr follow_mark_pub_;

  // Broadcast tf from base_link to base_link_fake
  rclcpp::TimerBase::SharedPtr tf_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  geometry_msgs::msg::PoseStamped planner_local_pose_;
  rclcpp::Time last_local_plan_stamp_;
  double current_angle_{0.0};
  double base_link_angle_{0.0};
  double local_plan_timeout_sec_{0.5};
  int tf_publish_frequency_{20};
  bool has_local_plan_{false};
  bool use_local_plan_transform_{true};
  bool follow_mark_enable_{false};
  double follow_mark_input_stale_timeout_sec_{0.3};
  double follow_mark_enter_margin_m_{0.35};
  double follow_mark_exit_margin_m_{0.55};
  std::vector<double> follow_mark_zone_rects_;
  std::vector<double> follow_mark_zone_polygon_points_;
  std::vector<int64_t> follow_mark_zone_polygon_sizes_;
  std::vector<ZonePolygon> follow_mark_zone_polygons_;
  bool follow_mark_zone_latched_{false};
  bool has_follow_mark_manual_{false};
  bool has_follow_mark_hint_{false};
  uint8_t follow_mark_default_value_{1U};
  uint8_t follow_mark_zone_value_{1U};
  uint8_t follow_mark_manual_{1U};
  uint8_t follow_mark_hint_{1U};
  rclcpp::Time last_follow_mark_manual_stamp_;
  rclcpp::Time last_follow_mark_hint_stamp_;
  float spin_speed_{0.0F};
};

}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
