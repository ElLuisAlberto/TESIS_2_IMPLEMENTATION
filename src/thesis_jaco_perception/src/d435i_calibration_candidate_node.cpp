#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{
void add_value(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(item);
}

class CalibrationCandidateNode : public rclcpp::Node
{
public:
  CalibrationCandidateNode() : Node("d435i_calibration_candidate")
  {
    rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/camera/d435i/color/image_raw");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera/d435i/color/camera_info");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "d435i_color_optical_frame");
    candidate_extrinsic_file_ = declare_parameter<std::string>("candidate_extrinsic_file", "");
    detector_available_ = declare_parameter<bool>("detector_available", false);
    session_only_ = declare_parameter<bool>("session_only", true);
    explicit_final_acceptance_ = declare_parameter<bool>("explicit_final_acceptance", false);
    if (!session_only_ || explicit_final_acceptance_) {
      throw std::runtime_error("Only session_only candidates are permitted by this node");
    }
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/thesis/calibration/candidate_diagnostics", rclcpp::QoS(10));
    candidate_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/thesis/calibration/candidate_pose", rclcpp::QoS(1));
    const auto qos = rclcpp::SensorDataQoS();
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      rgb_topic_, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr) { images_seen_++; });
    info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, qos, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
        camera_info_seen_ = !info->header.frame_id.empty();
      });
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publish_status(); });
  }

private:
  void publish_status()
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "D435i temporary calibration candidate";
    status.hardware_id = "d435i_calibration_candidate";
    status.level = detector_available_ ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = detector_available_ ? "DETECTOR_INTERFACE_NOT_IMPLEMENTED" :
      "WAITING_FOR_LOCAL_DETECTOR";
    add_value(status, "session_only", "true");
    add_value(status, "camera_frame", camera_frame_);
    add_value(status, "candidate_extrinsic_file_explicit",
      candidate_extrinsic_file_.empty() ? "false" : "true");
    add_value(status, "images_seen", std::to_string(images_seen_));
    add_value(status, "camera_info_seen", camera_info_seen_ ? "true" : "false");
    add_value(status, "tf_published", "false");
    add_value(status, "candidate_topic", "/thesis/calibration/candidate_pose");
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(status);
    diagnostics_->publish(array);
  }
  std::string rgb_topic_;
  std::string camera_info_topic_;
  std::string camera_frame_;
  std::string candidate_extrinsic_file_;
  bool detector_available_{false};
  bool session_only_{true};
  bool explicit_final_acceptance_{false};
  std::size_t images_seen_{0U};
  bool camera_info_seen_{false};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr candidate_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalibrationCandidateNode>());
  rclcpp::shutdown();
  return 0;
}
