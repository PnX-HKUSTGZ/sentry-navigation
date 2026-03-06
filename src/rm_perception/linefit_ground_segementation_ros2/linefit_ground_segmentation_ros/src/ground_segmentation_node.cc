#include <memory>
#include <chrono>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "ground_segmentation/ground_segmentation.h"

class SegmentationNode : public rclcpp::Node {
public:
  SegmentationNode(const rclcpp::NodeOptions &node_options);
  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  GroundSegmentationParams params_;
  std::shared_ptr<GroundSegmentation> segmenter_;
  std::string gravity_aligned_frame_;
  double warn_input_delay_sec_;
  double drop_input_delay_sec_;
  double warn_processing_time_ms_;
};

SegmentationNode::SegmentationNode(const rclcpp::NodeOptions &node_options)
    : Node("ground_segmentation", node_options) {
  gravity_aligned_frame_ =
      this->declare_parameter("gravity_aligned_frame", "gravity_aligned");

  params_.visualize = this->declare_parameter("visualize", params_.visualize);
  params_.n_bins = this->declare_parameter("n_bins", params_.n_bins);
  params_.n_segments =
      this->declare_parameter("n_segments", params_.n_segments);
  params_.max_dist_to_line =
      this->declare_parameter("max_dist_to_line", params_.max_dist_to_line);
  params_.max_slope = this->declare_parameter("max_slope", params_.max_slope);
  params_.min_slope = this->declare_parameter("min_slope", params_.min_slope);
  params_.long_threshold =
      this->declare_parameter("long_threshold", params_.long_threshold);
  params_.max_long_height =
      this->declare_parameter("max_long_height", params_.max_long_height);
  params_.max_start_height =
      this->declare_parameter("max_start_height", params_.max_start_height);
  params_.sensor_height =
      this->declare_parameter("sensor_height", params_.sensor_height);
  params_.line_search_angle =
      this->declare_parameter("line_search_angle", params_.line_search_angle);
  params_.n_threads = this->declare_parameter("n_threads", params_.n_threads);
  // Params that need to be squared.
  double r_min, r_max, max_fit_error;
  if (this->get_parameter("r_min", r_min)) {
    params_.r_min_square = r_min * r_min;
  }
  if (this->get_parameter("r_max", r_max)) {
    params_.r_max_square = r_max * r_max;
  }
  if (this->get_parameter("max_fit_error", max_fit_error)) {
    params_.max_error_square = max_fit_error * max_fit_error;
  }
  segmenter_ = std::make_shared<GroundSegmentation>(params_);
  std::string ground_topic, obstacle_topic, input_topic;
  ground_topic = this->declare_parameter("ground_output_topic", "ground_cloud");
  obstacle_topic =
      this->declare_parameter("obstacle_output_topic", "obstacle_cloud");
  input_topic = this->declare_parameter("input_topic", "input_cloud");
  warn_input_delay_sec_ =
      this->declare_parameter("warn_input_delay_sec", 0.15);
  drop_input_delay_sec_ =
      this->declare_parameter("drop_input_delay_sec", -1.0);
  warn_processing_time_ms_ =
      this->declare_parameter("warn_processing_time_ms", 15.0);
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&SegmentationNode::scanCallback, this, std::placeholders::_1));
  ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      ground_topic, rclcpp::SensorDataQoS());
  obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      obstacle_topic, rclcpp::SensorDataQoS());
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  RCLCPP_INFO(this->get_logger(), "Segmentation node initialized");
}

void SegmentationNode::scanCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  const auto callback_start = std::chrono::steady_clock::now();
  if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) {
    const auto now = this->get_clock()->now();
    const auto stamp = rclcpp::Time(msg->header.stamp, now.get_clock_type());
    const double input_delay_sec = (now - stamp).seconds();
    if (warn_input_delay_sec_ > 0.0 && input_delay_sec > warn_input_delay_sec_) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "segmentation input delay is high: %.3fs", input_delay_sec);
    }
    if (drop_input_delay_sec_ > 0.0 && input_delay_sec > drop_input_delay_sec_) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "dropping stale segmentation input: delay %.3fs exceeds %.3fs",
          input_delay_sec, drop_input_delay_sec_);
      return;
    }
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  pcl::PointCloud<pcl::PointXYZ> cloud_transformed;

  std::vector<int> labels;

  bool is_original_pc = true;
  if (!gravity_aligned_frame_.empty()) {
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
          gravity_aligned_frame_, msg->header.frame_id, msg->header.stamp);
      // Remove translation part.
      tf_stamped.transform.translation.x = 0;
      tf_stamped.transform.translation.y = 0;
      tf_stamped.transform.translation.z = 0;
      Eigen::Affine3d tf;
      tf.translate(Eigen::Vector3d(0, 0, 0));
      tf.rotate(Eigen::Quaterniond(
          tf_stamped.transform.rotation.w, tf_stamped.transform.rotation.x,
          tf_stamped.transform.rotation.y, tf_stamped.transform.rotation.z));
      // tf::transformMsgToEigen(tf_stamped.transform, tf);
      pcl::transformPointCloud(cloud, cloud_transformed, tf);
      is_original_pc = false;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to transform point cloud into "
                  "gravity frame: %s",
                  ex.what());
    }
  }

  // Trick to avoid PC copy if we do not transform.
  const pcl::PointCloud<pcl::PointXYZ> &cloud_proc =
      is_original_pc ? cloud : cloud_transformed;

  segmenter_->segment(cloud_proc, &labels);
  pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud;
  for (size_t i = 0; i < cloud.size(); ++i) {
    if (labels[i] == 1)
      ground_cloud.push_back(cloud[i]);
    else
      obstacle_cloud.push_back(cloud[i]);
  }
  auto ground_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  auto obstacle_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(ground_cloud, *ground_msg);
  pcl::toROSMsg(obstacle_cloud, *obstacle_msg);
  ground_msg->header = msg->header;
  obstacle_msg->header = msg->header;
  ground_pub_->publish(*ground_msg);
  obstacle_pub_->publish(*obstacle_msg);

  const auto callback_end = std::chrono::steady_clock::now();
  const double callback_cost_ms =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
          callback_end - callback_start)
          .count();
  if (warn_processing_time_ms_ > 0.0 && callback_cost_ms > warn_processing_time_ms_) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "ground_segmentation callback cost %.2f ms", callback_cost_ms);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<SegmentationNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
