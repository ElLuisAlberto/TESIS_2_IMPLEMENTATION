#include "thesis_jaco_perception/shadow_filter_logic.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace thesis_jaco_perception
{
namespace
{
constexpr char kWorldFrame[] = "world";

double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

std::int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

Point3 translation_of(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & value = transform.transform.translation;
  return {value.x, value.y, value.z};
}

void add_value(
  diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(item);
}

sensor_msgs::msg::PointCloud2 cloud_with_indices(
  const sensor_msgs::msg::PointCloud2 & input, const std::vector<std::size_t> & indices)
{
  sensor_msgs::msg::PointCloud2 output = input;
  output.header.frame_id = kWorldFrame;
  output.height = 1U;
  output.width = static_cast<std::uint32_t>(indices.size());
  output.row_step = output.point_step * output.width;
  output.data.clear();
  output.data.reserve(static_cast<std::size_t>(output.row_step));
  for (const std::size_t index : indices) {
    const std::size_t offset = index * input.point_step;
    output.data.insert(
      output.data.end(), input.data.begin() + static_cast<std::ptrdiff_t>(offset),
      input.data.begin() + static_cast<std::ptrdiff_t>(offset + input.point_step));
  }
  return output;
}
}  // namespace

class JacoSelfFilterNode : public rclcpp::Node
{
public:
  JacoSelfFilterNode()
  : Node("jaco_self_filter_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    configuration_.enabled = declare_parameter<bool>("enabled", false);
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/thesis/perception/points_world");
    configuration_.camera_optical_frame = declare_parameter<std::string>(
      "camera_optical_frame", "");
    configuration_.expected_frame = declare_parameter<std::string>("expected_frame", kWorldFrame);
    configuration_.capsule_start_frames = declare_parameter<std::vector<std::string>>(
      "capsule_start_frames", std::vector<std::string>{});
    configuration_.capsule_end_frames = declare_parameter<std::vector<std::string>>(
      "capsule_end_frames", std::vector<std::string>{});
    configuration_.capsule_inner_radii_m = declare_parameter<std::vector<double>>(
      "capsule_inner_radii_m", std::vector<double>{});
    configuration_.epsilon_total_m = declare_parameter<double>("epsilon_total_m", -1.0);
    configuration_.max_cloud_age_sec = declare_parameter<double>("max_cloud_age_sec", 0.5);
    configuration_.tf_timeout_sec = declare_parameter<double>("tf_timeout_sec", 0.05);

    const auto cloud_qos = rclcpp::SensorDataQoS().keep_last(1);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, cloud_qos,
      std::bind(&JacoSelfFilterNode::cloud_callback, this, std::placeholders::_1));
    self_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/thesis/jaco_perception/points_self_confirmed", cloud_qos);
    external_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/thesis/jaco_perception/points_external_candidate", cloud_qos);
    occluded_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/thesis/jaco_perception/points_occluded_unknown", cloud_qos);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/thesis/jaco_perception/diagnostics", rclcpp::QoS(10).reliable());
  }

private:
  void publish_diagnostic(
    const sensor_msgs::msg::PointCloud2 & cloud, std::uint8_t level,
    const std::string & message, const std::string & tf_status,
    const PointPartitions * partitions = nullptr, std::size_t total_points = 0U)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "JACO2 RGB-D self-filter shadow";
    status.hardware_id = "jaco_self_filter_shadow";
    status.level = level;
    status.message = message;
    add_value(status, "enabled", configuration_.enabled ? "true" : "false");
    add_value(status, "configuration_valid", shadow_configuration_error(configuration_).empty() ? "true" : "false");
    add_value(status, "cloud_stamp_age_sec", std::to_string(
      this->now().seconds() - stamp_seconds(cloud.header.stamp)));
    add_value(status, "total_points", std::to_string(total_points));
    add_value(status, "self_confirmed_points", std::to_string(
      partitions == nullptr ? 0U : partitions->self_confirmed_indices.size()));
    add_value(status, "external_confirmed_points", std::to_string(
      partitions == nullptr ? 0U : partitions->external_confirmed_indices.size()));
    add_value(status, "occluded_unknown_points", std::to_string(
      partitions == nullptr ? 0U : partitions->occluded_unknown_indices.size()));
    add_value(status, "tf_status", tf_status);
    diagnostic_msgs::msg::DiagnosticArray report;
    report.header.stamp = this->now();
    report.status.push_back(status);
    diagnostics_publisher_->publish(report);
  }

  bool capsules_at_stamp(
    const builtin_interfaces::msg::Time & stamp, std::vector<Capsule3> & capsules,
    Point3 & camera_origin, std::string & error)
  {
    try {
      const rclcpp::Time exact_time(stamp);
      const auto timeout = rclcpp::Duration::from_seconds(configuration_.tf_timeout_sec);
      camera_origin = translation_of(tf_buffer_.lookupTransform(
        kWorldFrame, configuration_.camera_optical_frame, exact_time, timeout));
      capsules.clear();
      capsules.reserve(configuration_.capsule_start_frames.size());
      for (std::size_t index = 0; index < configuration_.capsule_start_frames.size(); ++index) {
        const auto start = tf_buffer_.lookupTransform(
          kWorldFrame, configuration_.capsule_start_frames[index], exact_time, timeout);
        const auto end = tf_buffer_.lookupTransform(
          kWorldFrame, configuration_.capsule_end_frames[index], exact_time, timeout);
        capsules.push_back({
          translation_of(start), translation_of(end), configuration_.capsule_inner_radii_m[index]});
      }
    } catch (const tf2::TransformException & exception) {
      error = exception.what();
      return false;
    }
    return true;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    if (!configuration_.enabled) {
      publish_diagnostic(*cloud, diagnostic_msgs::msg::DiagnosticStatus::OK,
        "DISABLED", "NOT_REQUESTED");
      return;
    }
    const std::string configuration_error = shadow_configuration_error(configuration_);
    if (!configuration_error.empty()) {
      publish_diagnostic(*cloud, diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "CALIBRATION_OR_CONFIGURATION_INVALID: " + configuration_error, "NOT_REQUESTED");
      return;
    }
    const std::string header_error = cloud_header_error(
      {cloud->header.frame_id, stamp_nanoseconds(cloud->header.stamp)},
      this->now().nanoseconds(), configuration_.max_cloud_age_sec);
    if (!header_error.empty()) {
      publish_diagnostic(*cloud, diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "CLOUD_INVALID: " + header_error, "NOT_REQUESTED");
      return;
    }
    std::vector<Capsule3> capsules;
    Point3 camera_origin;
    std::string tf_error;
    if (!capsules_at_stamp(cloud->header.stamp, capsules, camera_origin, tf_error)) {
      publish_diagnostic(*cloud, diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "TF_AT_STAMP_UNAVAILABLE: " + tf_error, "UNAVAILABLE");
      return;
    }
    std::vector<Point3> points;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x_iterator(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_iterator(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_iterator(*cloud, "z");
      const std::size_t count = static_cast<std::size_t>(cloud->width) * cloud->height;
      points.reserve(count);
      for (std::size_t index = 0; index < count; ++index, ++x_iterator, ++y_iterator, ++z_iterator) {
        points.push_back({*x_iterator, *y_iterator, *z_iterator});
      }
    } catch (const std::runtime_error & exception) {
      publish_diagnostic(*cloud, diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "CLOUD_INVALID: missing or invalid x/y/z fields: " + std::string(exception.what()), "AVAILABLE");
      return;
    }
    try {
      const PointPartitions partitions = partition_observed_points(
        camera_origin, capsules, points, configuration_.epsilon_total_m);
      self_publisher_->publish(cloud_with_indices(*cloud, partitions.self_confirmed_indices));
      external_publisher_->publish(cloud_with_indices(*cloud, partitions.external_confirmed_indices));
      occluded_publisher_->publish(cloud_with_indices(*cloud, partitions.occluded_unknown_indices));
      publish_diagnostic(*cloud, diagnostic_msgs::msg::DiagnosticStatus::OK,
        "SHADOW_CLASSIFICATION_COMPLETE", "AVAILABLE", &partitions, points.size());
    } catch (const std::invalid_argument & exception) {
      publish_diagnostic(*cloud, diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "CALIBRATION_OR_CONFIGURATION_INVALID: " + std::string(exception.what()), "AVAILABLE");
    }
  }

  ShadowFilterConfiguration configuration_;
  std::string input_topic_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr self_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr external_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occluded_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
};
}  // namespace thesis_jaco_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thesis_jaco_perception::JacoSelfFilterNode>());
  rclcpp::shutdown();
  return 0;
}
