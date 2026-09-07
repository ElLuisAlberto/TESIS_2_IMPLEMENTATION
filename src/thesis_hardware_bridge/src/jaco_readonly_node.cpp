#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "Kinova.API.USBCommandLayerUbuntu.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace
{
constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
}

class JacoReadonlyNode : public rclcpp::Node
{
public:
  JacoReadonlyNode()
  : Node("jaco_readonly_node")
  {
    const double rate_hz = declare_parameter<double>("publish_rate_hz", 20.0);
    topic_ = declare_parameter<std::string>(
      "joint_state_topic", "/jaco/joint_states");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");

    if (rate_hz <= 0.0) {
      throw std::runtime_error("publish_rate_hz debe ser mayor que cero");
    }

    const int period_ms = std::max(
      1, static_cast<int>(1000.0 / rate_hz));

    int result = InitAPI();
    if (result != NO_ERROR_KINOVA) {
      throw std::runtime_error(
        "InitAPI fallo con codigo " + std::to_string(result));
    }
    api_open_ = true;

    KinovaDevice devices[MAX_KINOVA_DEVICE] = {};
    int device_result = 0;
    const int device_count = GetDevices(devices, device_result);

    if (device_result != NO_ERROR_KINOVA || device_count < 1) {
      CloseAPI();
      api_open_ = false;
      throw std::runtime_error(
        "GetDevices fallo: resultado=" +
        std::to_string(device_result));
    }

    result = SetActiveDevice(devices[0]);
    if (result != NO_ERROR_KINOVA) {
      CloseAPI();
      api_open_ = false;
      throw std::runtime_error(
        "SetActiveDevice fallo con codigo " +
        std::to_string(result));
    }

    publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      topic_, rclcpp::QoS(10));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&JacoReadonlyNode::publish_state, this));

    RCLCPP_INFO(
      get_logger(),
      "Puente JACO en solo lectura activo: %s",
      topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "No se usan trayectorias, dedos, HOME ni StartControlAPI");
  }

  ~JacoReadonlyNode() override
  {
    if (api_open_) {
      const int result = CloseAPI();
      RCLCPP_INFO(get_logger(), "CloseAPI retorno %d", result);
      api_open_ = false;
    }
  }

private:
  void publish_state()
  {
    AngularPosition position = {};
    const int result = GetAngularPosition(position);

    if (result != NO_ERROR_KINOVA) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "GetAngularPosition fallo con codigo %d", result);
      return;
    }

    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    message.name = {
      "j2n6s300_joint_1",
      "j2n6s300_joint_2",
      "j2n6s300_joint_3",
      "j2n6s300_joint_4",
      "j2n6s300_joint_5",
      "j2n6s300_joint_6",
      "j2n6s300_joint_finger_1",
      "j2n6s300_joint_finger_tip_1",
      "j2n6s300_joint_finger_2",
      "j2n6s300_joint_finger_tip_2",
      "j2n6s300_joint_finger_3",
      "j2n6s300_joint_finger_tip_3"
    };
    message.position = {
      position.Actuators.Actuator1 * kDegreesToRadians,
      position.Actuators.Actuator2 * kDegreesToRadians,
      position.Actuators.Actuator3 * kDegreesToRadians,
      position.Actuators.Actuator4 * kDegreesToRadians,
      position.Actuators.Actuator5 * kDegreesToRadians,
      position.Actuators.Actuator6 * kDegreesToRadians,

      // Feedback físico de dedos pendiente; solo completan el TF de RViz.
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    };

    publisher_->publish(message);
  }

  bool api_open_{false};
  std::string topic_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<JacoReadonlyNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    std::cerr << "Puente JACO detenido: "
              << exception.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
