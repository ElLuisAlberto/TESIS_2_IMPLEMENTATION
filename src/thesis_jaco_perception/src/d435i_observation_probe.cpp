#include "thesis_jaco_perception/observation_probe_logic.hpp"

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace thesis_jaco_perception
{
namespace
{
constexpr double kLowRateHz = 1.0;

void add_value(
  diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(item);
}
}  // namespace

class D435iObservationProbe : public rclcpp::Node
{
public:
  D435iObservationProbe()
  : Node("d435i_observation_probe")
  {
    pointcloud_topic_ = declare_parameter<std::string>(
      "pointcloud_topic", "/camera/d435i/depth/color/points");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "");
    observation_window_sec_ = declare_parameter<double>("observation_window_sec", 30.0);
    max_range_m_ = declare_parameter<double>("max_range_m", 1.5);
    report_path_ = declare_parameter<std::string>("report_path", "");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/thesis/jaco_perception/d435i_observation_diagnostics");
    validate_probe_parameters(observation_window_sec_, max_range_m_);

    const auto cloud_qos = rclcpp::SensorDataQoS().keep_last(1);
    pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, cloud_qos,
      std::bind(&D435iObservationProbe::pointcloud_callback, this, std::placeholders::_1));
    if (!camera_info_topic_.empty()) {
      camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, cloud_qos,
        std::bind(&D435iObservationProbe::camera_info_callback, this, std::placeholders::_1));
    }
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10).reliable());
    completion_timer_ = create_wall_timer(
      std::chrono::duration<double>(observation_window_sec_),
      std::bind(&D435iObservationProbe::complete, this));
    publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK, "WAITING_FOR_CLOUD");
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    if (complete_) {
      return;
    }
    try {
      observe_pointcloud(*cloud, this->now().seconds(), max_range_m_, metrics_);
      publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK, "COLLECTING");
    } catch (const std::exception & exception) {
      publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "ERROR_CLOUD_FORMAT: " + std::string(exception.what()));
    }
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
  {
    if (!complete_) {
      observe_camera_info(*camera_info, metrics_);
    }
  }

  void publish_diagnostic(std::uint8_t level, const std::string & state)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "D435i passive observation probe";
    status.hardware_id = "d435i_observation_probe";
    status.level = level;
    status.message = state;
    add_value(status, "state", state);
    add_value(status, "pointcloud_topic", pointcloud_topic_);
    add_value(status, "camera_info_requested", camera_info_topic_.empty() ? "false" : "true");
    add_value(status, "camera_info_received", metrics_.camera_info.received ? "true" : "false");
    add_value(status, "clouds_received", std::to_string(metrics_.clouds_received));
    add_value(status, "observed_frequency_hz", std::to_string(metrics_.observed_frequency_hz));
    add_value(status, "last_stamp_age_sec", std::to_string(metrics_.last_stamp_age_sec));
    add_value(status, "null_stamps", std::to_string(metrics_.null_stamps));
    add_value(status, "future_stamps", std::to_string(metrics_.future_stamps));
    add_value(status, "stale_stamps", std::to_string(metrics_.stale_stamps));
    add_value(status, "finite_points", std::to_string(metrics_.finite_points));
    add_value(status, "below_min_range_points", std::to_string(metrics_.below_min_range_points));
    add_value(status, "in_range_points", std::to_string(metrics_.in_range_points));
    add_value(status, "above_max_range_points", std::to_string(metrics_.above_max_range_points));
    diagnostic_msgs::msg::DiagnosticArray report;
    report.header.stamp = this->now();
    report.status.push_back(status);
    diagnostics_publisher_->publish(report);
  }

  void complete()
  {
    if (complete_) {
      return;
    }
    complete_ = true;
    completion_timer_->cancel();
    metrics_.observed_frequency_hz = static_cast<double>(metrics_.clouds_received) /
      observation_window_sec_;
    std::string state = "COMPLETE";
    std::uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    if (metrics_.clouds_received == 0U) {
      state = "ERROR_NO_CLOUD";
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else if (!camera_info_topic_.empty() && !metrics_.camera_info.received) {
      state = "WARN_CAMERA_INFO_UNAVAILABLE";
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else if (metrics_.null_stamps > 0U || metrics_.future_stamps > 0U ||
      metrics_.stale_stamps > 0U)
    {
      state = "WARN_STAMP_QUALITY";
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else if (metrics_.observed_frequency_hz < kLowRateHz) {
      state = "WARN_LOW_RATE";
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }
    try {
      write_json_report_if_requested(report_path_, metrics_);
    } catch (const std::exception & exception) {
      state = "ERROR_REPORT_WRITE: " + std::string(exception.what());
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }
    publish_diagnostic(level, state);
    std::cout << "D435i passive observation complete: state=" << state
              << ", clouds=" << metrics_.clouds_received
              << ", rate_hz=" << metrics_.observed_frequency_hz
              << ", finite_points=" << metrics_.finite_points << std::endl;
  }

  std::string pointcloud_topic_;
  std::string camera_info_topic_;
  double observation_window_sec_{0.0};
  double max_range_m_{0.0};
  std::string report_path_;
  std::string diagnostics_topic_;
  ObservationMetrics metrics_;
  bool complete_{false};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr completion_timer_;
};
}  // namespace thesis_jaco_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thesis_jaco_perception::D435iObservationProbe>());
  rclcpp::shutdown();
  return 0;
}
