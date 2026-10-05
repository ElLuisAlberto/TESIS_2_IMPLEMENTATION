// Copyright 2026 Luis Alberto Munoz Marin
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <chrono>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PerceptionHealth : public rclcpp::Node
{
public:
  PerceptionHealth()
  : Node("perception_health"),
    last_report_(std::chrono::steady_clock::now())
  {
    cloud_topic_ = declare_parameter<std::string>(
      "cloud_topic", "/thesis/perception/points_filtered");
    imu_topic_ = declare_parameter<std::string>(
      "imu_topic", "/camera/d435i/imu");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/thesis/perception/diagnostics");
    hardware_id_ = declare_parameter<std::string>(
      "hardware_id", "Intel RealSense D435i");
    expected_frame_ = declare_parameter<std::string>("expected_frame", "");
    imu_required_ = declare_parameter<bool>("imu_required", false);
    min_cloud_hz_ = declare_parameter<double>("min_cloud_hz", 10.0);
    min_imu_hz_ = declare_parameter<double>("min_imu_hz", 50.0);
    cloud_timeout_s_ = declare_parameter<double>("cloud_timeout_s", 0.5);
    imu_timeout_s_ = declare_parameter<double>("imu_timeout_s", 0.2);
    max_stamp_age_s_ = declare_parameter<double>("max_stamp_age_s", 0.5);
    future_tolerance_s_ = declare_parameter<double>("future_tolerance_s", 0.05);
    min_cloud_points_ = declare_parameter<int>("min_cloud_points", 0);

    if (min_cloud_hz_ <= 0.0 || min_imu_hz_ <= 0.0 ||
      cloud_timeout_s_ <= 0.0 || imu_timeout_s_ <= 0.0 ||
      max_stamp_age_s_ <= 0.0 || future_tolerance_s_ < 0.0 ||
      min_cloud_points_ < 0)
    {
      throw std::runtime_error("invalid perception health parameters");
    }

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(10);
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, sensor_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        cloud_seen_ = true;
        ++cloud_count_;
        last_cloud_ = std::chrono::steady_clock::now();
        last_cloud_frame_ = message->header.frame_id;
        last_cloud_points_ = static_cast<std::size_t>(message->width) *
        static_cast<std::size_t>(message->height);
        last_cloud_stamp_s_ = static_cast<double>(message->header.stamp.sec) +
        static_cast<double>(message->header.stamp.nanosec) * 1.0e-9;
      });
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr) {
        imu_seen_ = true;
        ++imu_count_;
        last_imu_ = std::chrono::steady_clock::now();
      });

    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10).reliable());
    timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&PerceptionHealth::publish_report, this));
  }

private:
  static std::string number(double value)
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2) << value;
    return stream.str();
  }

  static void add_value(
    diagnostic_msgs::msg::DiagnosticStatus & status,
    const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(item);
  }

  void publish_report()
  {
    const auto now_steady = std::chrono::steady_clock::now();
    const double period = std::max(
      1e-6, std::chrono::duration<double>(now_steady - last_report_).count());
    const double cloud_hz = static_cast<double>(cloud_count_) / period;
    const double imu_hz = static_cast<double>(imu_count_) / period;
    const double cloud_age = cloud_seen_ ?
      std::chrono::duration<double>(now_steady - last_cloud_).count() : -1.0;
    const double imu_age = imu_seen_ ?
      std::chrono::duration<double>(now_steady - last_imu_).count() : -1.0;
    const double now_ros_s = this->now().seconds();
    const bool stamp_comparable = cloud_seen_ &&
      last_cloud_stamp_s_ > 0.0 && now_ros_s > 0.0;
    const double stamp_age = stamp_comparable ?
      now_ros_s - last_cloud_stamp_s_ : -1.0;
    const bool frame_valid = expected_frame_.empty() ||
      last_cloud_frame_ == expected_frame_;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "D435i perception pipeline";
    status.hardware_id = hardware_id_;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "OK";

    if (!cloud_seen_ || cloud_age > cloud_timeout_s_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "LOST: nube no disponible";
    } else if (imu_required_ && (!imu_seen_ || imu_age > imu_timeout_s_)) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "LOST: IMU no disponible";
    } else if (!frame_valid) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "LOST: frame de nube inesperado";
    } else if (stamp_comparable && stamp_age > max_stamp_age_s_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "LOST: timestamp de nube vencido";
    } else if (stamp_comparable && stamp_age < -future_tolerance_s_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "LOST: timestamp de nube en el futuro";
    } else if (cloud_hz < min_cloud_hz_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "DEGRADED: frecuencia de nube baja";
    } else if (imu_required_ && imu_hz < min_imu_hz_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "DEGRADED: frecuencia de IMU baja";
    } else if (last_cloud_points_ < static_cast<std::size_t>(min_cloud_points_)) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "DEGRADED: nube con pocos puntos";
    }

    add_value(status, "cloud_topic", cloud_topic_);
    add_value(status, "imu_topic", imu_topic_);
    add_value(status, "cloud_hz", number(cloud_hz));
    add_value(status, "imu_hz", number(imu_hz));
    add_value(status, "cloud_age_s", number(cloud_age));
    add_value(status, "imu_age_s", number(imu_age));
    add_value(status, "cloud_stamp_age_s", number(stamp_age));
    add_value(status, "cloud_frame", last_cloud_frame_);
    add_value(status, "expected_frame", expected_frame_);
    add_value(status, "frame_valid", frame_valid ? "true" : "false");
    add_value(status, "cloud_points", std::to_string(last_cloud_points_));
    add_value(status, "imu_required", imu_required_ ? "true" : "false");

    diagnostic_msgs::msg::DiagnosticArray report;
    report.header.stamp = this->now();
    report.status.push_back(status);
    diagnostics_publisher_->publish(report);

    cloud_count_ = 0;
    imu_count_ = 0;
    last_report_ = now_steady;
  }

  std::string cloud_topic_;
  std::string imu_topic_;
  std::string diagnostics_topic_;
  std::string hardware_id_;
  std::string expected_frame_;
  bool imu_required_;
  double min_cloud_hz_;
  double min_imu_hz_;
  double cloud_timeout_s_;
  double imu_timeout_s_;
  double max_stamp_age_s_;
  double future_tolerance_s_;
  int min_cloud_points_;

  bool cloud_seen_{false};
  bool imu_seen_{false};
  std::size_t cloud_count_{0};
  std::size_t imu_count_{0};
  std::size_t last_cloud_points_{0};
  std::string last_cloud_frame_;
  double last_cloud_stamp_s_{0.0};
  std::chrono::steady_clock::time_point last_report_;
  std::chrono::steady_clock::time_point last_cloud_;
  std::chrono::steady_clock::time_point last_imu_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionHealth>());
  rclcpp::shutdown();
  return 0;
}
