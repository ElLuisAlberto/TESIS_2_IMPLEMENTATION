#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include "Kinova.API.USBCommandLayerUbuntu.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "thesis_hardware_bridge/bridge_logic.hpp"
#include "thesis_interfaces/msg/execution_control.hpp"
#include "thesis_interfaces/msg/execution_trajectory.hpp"
#include "thesis_interfaces/msg/joint_command.hpp"
#include "thesis_interfaces/msg/pipeline_timing.hpp"

namespace
{

using namespace std::chrono_literals;
using thesis_hardware_bridge::JointVector;
using thesis_hardware_bridge::kDegreesToRadians;
using thesis_hardware_bridge::kJointCount;
using thesis_hardware_bridge::kJointNames;
using thesis_hardware_bridge::kRadiansToDegrees;

constexpr double kFingerDegreesPerUnit = 80.0 / 6800.0;
constexpr std::size_t kPublishedJointCount = 12;

std::string bool_text(bool value)
{
  return value ? "true" : "false";
}

std::string kinova_device_serial(const KinovaDevice & device)
{
  std::string serial(
    device.SerialNumber,
    strnlen(device.SerialNumber, SERIAL_LENGTH));
  const auto first = serial.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = serial.find_last_not_of(" \t\r\n");
  return serial.substr(first, last - first + 1);
}

diagnostic_msgs::msg::KeyValue key_value(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1.0e-9;
}

}  // namespace

class JacoHardwareNode : public rclcpp::Node
{
public:
  JacoHardwareNode()
  : Node("jaco_hardware_adapter")
  {
    declare_parameter("mock_hardware", false);
    declare_parameter("hardware_output_enabled", false);
    declare_parameter("expected_serial_number", "");
    declare_parameter(
      "usb_lock_file", "/tmp/thesis_kinova_jaco2_usb.lock");
    declare_parameter("state_publish_rate_hz", 20.0);
    declare_parameter("control_rate_hz", 100.0);
    declare_parameter("state_timeout_sec", 0.20);
    declare_parameter("jog_command_timeout_sec", 0.18);
    declare_parameter("execution_control_timeout_sec", 0.35);
    declare_parameter("goal_tolerance_rad", 0.005);
    declare_parameter("goal_timeout_factor", 4.0);
    declare_parameter("goal_timeout_margin_sec", 2.0);
    declare_parameter("maximum_read_failures", 3);
    declare_parameter("initialization_read_attempts", 10);
    declare_parameter("initialization_retry_delay_ms", 200);
    declare_parameter("zero_command_count", 10);

    mock_hardware_ = get_parameter("mock_hardware").as_bool();
    output_permitted_ =
      get_parameter("hardware_output_enabled").as_bool();
    state_publish_rate_hz_ =
      get_parameter("state_publish_rate_hz").as_double();
    control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    state_timeout_sec_ = get_parameter("state_timeout_sec").as_double();
    jog_timeout_sec_ =
      get_parameter("jog_command_timeout_sec").as_double();
    execution_control_timeout_sec_ =
      get_parameter("execution_control_timeout_sec").as_double();
    goal_tolerance_rad_ =
      get_parameter("goal_tolerance_rad").as_double();
    goal_timeout_factor_ =
      get_parameter("goal_timeout_factor").as_double();
    goal_timeout_margin_sec_ =
      get_parameter("goal_timeout_margin_sec").as_double();
    maximum_read_failures_ =
      get_parameter("maximum_read_failures").as_int();
    initialization_read_attempts_ =
      get_parameter("initialization_read_attempts").as_int();
    initialization_retry_delay_ms_ =
      get_parameter("initialization_retry_delay_ms").as_int();
    zero_command_count_ = get_parameter("zero_command_count").as_int();
    usb_lock_file_ = get_parameter("usb_lock_file").as_string();

    validate_parameters();
    configure_ros_interfaces();
    initialize_backend();

    const auto feedback_period = std::chrono::duration<double>(
      1.0 / state_publish_rate_hz_);
    feedback_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(feedback_period),
      std::bind(&JacoHardwareNode::feedback_tick, this));
    diagnostic_timer_ = create_wall_timer(
      500ms, std::bind(&JacoHardwareNode::publish_diagnostics, this));

    publish_connection_state();
    publish_armed_state();
    control_thread_ = std::thread(&JacoHardwareNode::control_loop, this);
    RCLCPP_INFO(
      get_logger(),
      "Adaptador JACO2 preparado: backend=%s, salida_fisica_permitida=%s, "
      "armado=false",
      mock_hardware_ ? "mock" : "USB Kinova",
      output_permitted_ ? "true" : "false");
  }

  ~JacoHardwareNode() override
  {
    control_stop_requested_.store(true);
    if (control_thread_.joinable()) {
      control_thread_.join();
    }
    const std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    shutdown_backend();
  }

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  struct ActiveGoal
  {
    std::string command_id;
    builtin_interfaces::msg::Time intent_stamp;
    builtin_interfaces::msg::Time start_stamp;
    JointVector start{};
    JointVector target{};
    JointVector nominal_velocity{};
    double duration_sec{0.0};
    double speed_scale{1.0};
    bool control_received{false};
    SteadyTime started_at{};
    SteadyTime last_control_at{};
  };

  void validate_parameters()
  {
    const std::array<double, 8> positive_values = {
      state_publish_rate_hz_,
      control_rate_hz_,
      state_timeout_sec_,
      jog_timeout_sec_,
      execution_control_timeout_sec_,
      goal_tolerance_rad_,
      goal_timeout_factor_,
      goal_timeout_margin_sec_,
    };
    if (!std::all_of(
        positive_values.begin(), positive_values.end(),
        [](double value) {return std::isfinite(value) && value > 0.0;}))
    {
      throw std::runtime_error(
              "all rates, timeouts and tolerances must be positive");
    }
    if (std::abs(control_rate_hz_ - 100.0) > 1.0e-6) {
      throw std::runtime_error(
              "control_rate_hz must be 100 Hz for the Kinova DSP");
    }
    if (state_publish_rate_hz_ > control_rate_hz_) {
      throw std::runtime_error(
              "state_publish_rate_hz cannot exceed control_rate_hz");
    }
    if (
      maximum_read_failures_ < 1 || initialization_read_attempts_ < 1 ||
      initialization_retry_delay_ms_ < 1 || zero_command_count_ < 1)
    {
      throw std::runtime_error(
              "failure, initialization and zero-command values must be "
              "positive");
    }
    if (usb_lock_file_.empty()) {
      throw std::runtime_error("usb_lock_file cannot be empty");
    }
  }

  void configure_ros_interfaces()
  {
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10).reliable());
    execution_publisher_ =
      create_publisher<thesis_interfaces::msg::ExecutionTrajectory>(
      "/thesis/execution_trajectory", 10);
    timing_publisher_ =
      create_publisher<thesis_interfaces::msg::PipelineTiming>(
      "/thesis/pipeline_timing", 100);
    diagnostic_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/thesis/hardware/diagnostics", 10);

    auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    armed_publisher_ =
      create_publisher<std_msgs::msg::Bool>(
      "/thesis/hardware/armed", latched_qos);
    connected_publisher_ =
      create_publisher<std_msgs::msg::Bool>(
      "/thesis/hardware/connected", latched_qos);

    command_subscription_ =
      create_subscription<thesis_interfaces::msg::JointCommand>(
      "/thesis/supervised_command", 10,
      std::bind(
        &JacoHardwareNode::command_callback, this,
        std::placeholders::_1));
    jog_subscription_ =
      create_subscription<thesis_interfaces::msg::JointCommand>(
      "/thesis/supervised_jog_command", 10,
      std::bind(
        &JacoHardwareNode::jog_callback, this,
        std::placeholders::_1));
    control_subscription_ =
      create_subscription<thesis_interfaces::msg::ExecutionControl>(
      "/thesis/execution_control", 10,
      std::bind(
        &JacoHardwareNode::execution_control_callback, this,
        std::placeholders::_1));
    arm_service_ = create_service<std_srvs::srv::SetBool>(
      "/thesis/hardware/set_armed",
      std::bind(
        &JacoHardwareNode::arm_service_callback, this,
        std::placeholders::_1, std::placeholders::_2));
  }

  void acquire_usb_lock()
  {
    usb_lock_fd_ = open(
      usb_lock_file_.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0660);
    if (usb_lock_fd_ < 0) {
      throw std::runtime_error(
              "cannot open Kinova USB lock file: " + usb_lock_file_);
    }
    if (flock(usb_lock_fd_, LOCK_EX | LOCK_NB) != 0) {
      close(usb_lock_fd_);
      usb_lock_fd_ = -1;
      throw std::runtime_error(
              "another thesis_hardware_bridge process owns the Kinova USB "
              "lock: " + usb_lock_file_);
    }
    usb_lock_held_ = true;
  }

  void release_usb_lock()
  {
    if (usb_lock_fd_ < 0) {
      return;
    }
    flock(usb_lock_fd_, LOCK_UN);
    close(usb_lock_fd_);
    usb_lock_fd_ = -1;
    usb_lock_held_ = false;
  }

  void initialize_backend()
  {
    if (mock_hardware_) {
      connected_ = true;
      serial_number_ = "MOCK-JACO2";
      serial_reported_ = true;
      device_count_ = 1;
      selected_device_index_ = 0;
      positions_ = {
        0.0,
        180.0 * kDegreesToRadians,
        180.0 * kDegreesToRadians,
        0.0,
        0.0,
        0.0,
      };
      velocities_.fill(0.0);
      fingers_.fill(0.0);
      state_valid_ = true;
      last_state_at_ = std::chrono::steady_clock::now();
      last_mock_update_ = last_state_at_;
      return;
    }

    acquire_usb_lock();

    int result = InitAPI();
    if (result != NO_ERROR_KINOVA) {
      throw std::runtime_error(
              "InitAPI failed with code " + std::to_string(result));
    }
    api_open_ = true;

    KinovaDevice devices[MAX_KINOVA_DEVICE] = {};
    int list_result = 0;
    const int count = GetDevices(devices, list_result);
    if (list_result != NO_ERROR_KINOVA || count < 1) {
      throw std::runtime_error(
              "GetDevices failed: result=" + std::to_string(list_result) +
              ", count=" + std::to_string(count));
    }
    device_count_ = count;

    const std::string expected =
      get_parameter("expected_serial_number").as_string();
    expected_serial_configured_ = !expected.empty();
    int selected = -1;
    if (expected.empty()) {
      if (count != 1) {
        throw std::runtime_error(
                "exactly one JACO is required when expected_serial_number "
                "is empty; found " + std::to_string(count));
      }
      selected = 0;
    } else {
      for (int index = 0; index < count; ++index) {
        const std::string serial = kinova_device_serial(devices[index]);
        if (serial == expected) {
          selected = index;
          break;
        }
      }
      if (selected < 0) {
        throw std::runtime_error(
                "the configured JACO serial number was not found");
      }
    }
    selected_device_index_ = selected;

    result = SetActiveDevice(devices[selected]);
    if (result != NO_ERROR_KINOVA) {
      throw std::runtime_error(
              "SetActiveDevice failed with code " +
              std::to_string(result));
    }
    const std::string reported_serial =
      kinova_device_serial(devices[selected]);
    serial_reported_ = !reported_serial.empty();
    serial_number_ = serial_reported_ ? reported_serial :
      "KINOVA_USB_DEVICE_" + std::to_string(selected);
    if (!serial_reported_) {
      RCLCPP_WARN(
        get_logger(),
        "El SDK Kinova no reportó un número de serie; se seleccionó de "
        "forma segura el único dispositivo USB disponible (índice %d)",
        selected);
    }
    connected_ = true;

    SteadyTime initial_state_at{};
    const bool initialized = thesis_hardware_bridge::retry_operation(
      initialization_read_attempts_,
      [this, &initial_state_at](int attempt) {
        initial_state_at = std::chrono::steady_clock::now();
        if (read_hardware_state(initial_state_at)) {
          return true;
        }
        RCLCPP_WARN(
          get_logger(),
          "Lectura inicial Kinova %d/%d fallida: %s",
          attempt, initialization_read_attempts_, last_error_.c_str());
        return false;
      },
      [this]() {
        std::this_thread::sleep_for(
          std::chrono::milliseconds(initialization_retry_delay_ms_));
      });
    if (!initialized) {
      throw std::runtime_error(
              "the selected JACO did not provide a valid initial state after " +
              std::to_string(initialization_read_attempts_) +
              " attempts: " + last_error_);
    }
    state_valid_ = true;
    last_state_at_ = initial_state_at;
    last_error_.clear();
  }

  bool start_control_api()
  {
    const std::lock_guard<std::recursive_mutex> api_lock(api_mutex_);
    if (mock_hardware_) {
      control_api_started_ = true;
      return true;
    }
    int result = StartControlAPI();
    if (result != NO_ERROR_KINOVA) {
      last_error_ =
        "StartControlAPI failed with code " + std::to_string(result);
      return false;
    }
    control_api_started_ = true;
    result = SetAngularControl();
    if (result != NO_ERROR_KINOVA) {
      last_error_ =
        "SetAngularControl failed with code " + std::to_string(result);
      StopControlAPI();
      control_api_started_ = false;
      return false;
    }
    result = EraseAllTrajectories();
    if (result != NO_ERROR_KINOVA) {
      last_error_ =
        "EraseAllTrajectories failed with code " +
        std::to_string(result);
      StopControlAPI();
      control_api_started_ = false;
      return false;
    }
    return send_zero_burst();
  }

  void stop_control_api()
  {
    const std::lock_guard<std::recursive_mutex> api_lock(api_mutex_);
    if (!control_api_started_) {
      return;
    }
    if (!mock_hardware_) {
      EraseAllTrajectories();
      send_zero_burst();
      StopControlAPI();
    }
    control_api_started_ = false;
  }

  void shutdown_backend()
  {
    const std::lock_guard<std::recursive_mutex> api_lock(api_mutex_);
    if (shutdown_complete_) {
      return;
    }
    shutdown_complete_ = true;
    armed_ = false;
    active_goal_.reset();
    jog_active_ = false;
    stop_control_api();
    if (api_open_) {
      CloseAPI();
      api_open_ = false;
    }
    release_usb_lock();
    connected_ = false;
  }

  bool read_hardware_state(const SteadyTime & sample_time)
  {
    const std::lock_guard<std::recursive_mutex> api_lock(api_mutex_);
    if (mock_hardware_) {
      return true;
    }
    AngularPosition position{};
    const int position_result = GetAngularPosition(position);
    if (position_result != NO_ERROR_KINOVA) {
      last_error_ =
        "GetAngularPosition failed with code " +
        std::to_string(position_result);
      return false;
    }

    const std::array<float, kJointCount> position_values = {
      position.Actuators.Actuator1,
      position.Actuators.Actuator2,
      position.Actuators.Actuator3,
      position.Actuators.Actuator4,
      position.Actuators.Actuator5,
      position.Actuators.Actuator6,
    };
    JointVector measured_positions{};
    for (std::size_t index = 0; index < kJointCount; ++index) {
      measured_positions[index] =
        static_cast<double>(position_values[index]) * kDegreesToRadians;
    }
    fingers_ = {
      static_cast<double>(position.Fingers.Finger1) *
      kFingerDegreesPerUnit * kDegreesToRadians,
      static_cast<double>(position.Fingers.Finger2) *
      kFingerDegreesPerUnit * kDegreesToRadians,
      static_cast<double>(position.Fingers.Finger3) *
      kFingerDegreesPerUnit * kDegreesToRadians,
    };
    if (!thesis_hardware_bridge::finite_vector(measured_positions)) {
      last_error_ = "GetAngularPosition returned non-finite data";
      return false;
    }
    if (state_valid_) {
      const double elapsed = std::chrono::duration<double>(
        sample_time - last_state_at_).count();
      try {
        velocities_ = thesis_hardware_bridge::estimate_velocity(
          positions_, measured_positions, elapsed);
      } catch (const std::exception & exception) {
        last_error_ =
          std::string("velocity estimate failed: ") + exception.what();
        return false;
      }
    } else {
      velocities_.fill(0.0);
    }
    positions_ = measured_positions;
    return true;
  }

  void update_mock_state(const SteadyTime & now)
  {
    if (!mock_hardware_) {
      return;
    }
    const double elapsed =
      std::chrono::duration<double>(now - last_mock_update_).count();
    last_mock_update_ = now;
    if (!armed_ || elapsed <= 0.0 || elapsed > 0.1) {
      velocities_.fill(0.0);
      return;
    }
    velocities_ = last_sent_velocity_;
    for (std::size_t index = 0; index < kJointCount; ++index) {
      positions_[index] += velocities_[index] * elapsed;
    }
  }

  bool send_velocity(const JointVector & radians_per_second)
  {
    const std::lock_guard<std::recursive_mutex> api_lock(api_mutex_);
    if (!control_api_started_) {
      return false;
    }
    if (!thesis_hardware_bridge::finite_vector(radians_per_second)) {
      last_error_ = "refused a non-finite velocity command";
      return false;
    }
    JointVector bounded = radians_per_second;
    for (std::size_t index = 0; index < kJointCount; ++index) {
      bounded[index] = std::clamp(
        bounded[index],
        -thesis_hardware_bridge::kVelocityLimits[index],
        thesis_hardware_bridge::kVelocityLimits[index]);
    }
    last_sent_velocity_ = bounded;
    if (mock_hardware_) {
      return true;
    }

    TrajectoryPoint point;
    point.InitStruct();
    point.Position.Type = ANGULAR_VELOCITY;
    point.Position.HandMode = HAND_NOMOVEMENT;
    point.Position.Actuators.Actuator1 =
      static_cast<float>(bounded[0] * kRadiansToDegrees);
    point.Position.Actuators.Actuator2 =
      static_cast<float>(bounded[1] * kRadiansToDegrees);
    point.Position.Actuators.Actuator3 =
      static_cast<float>(bounded[2] * kRadiansToDegrees);
    point.Position.Actuators.Actuator4 =
      static_cast<float>(bounded[3] * kRadiansToDegrees);
    point.Position.Actuators.Actuator5 =
      static_cast<float>(bounded[4] * kRadiansToDegrees);
    point.Position.Actuators.Actuator6 =
      static_cast<float>(bounded[5] * kRadiansToDegrees);
    point.Position.Actuators.Actuator7 = 0.0F;
    const int result = SendBasicTrajectory(point);
    if (result != NO_ERROR_KINOVA) {
      last_error_ =
        "SendBasicTrajectory failed with code " +
        std::to_string(result);
      return false;
    }
    return true;
  }

  bool send_zero_burst()
  {
    const JointVector zero{};
    if (mock_hardware_) {
      last_sent_velocity_ = zero;
      return true;
    }
    if (!control_api_started_) {
      return true;
    }
    for (int index = 0; index < zero_command_count_; ++index) {
      if (!send_velocity(zero)) {
        return false;
      }
    }
    return true;
  }

  void stop_motion(
    const std::string & reason, bool disarm, const std::string & status)
  {
    const std::lock_guard<std::recursive_mutex> api_lock(api_mutex_);
    if (!mock_hardware_ && control_api_started_) {
      EraseAllTrajectories();
    }
    send_zero_burst();
    if (active_goal_) {
      publish_execution(
        active_goal_->command_id,
        status,
        reason,
        active_goal_->start,
        active_goal_->target,
        active_goal_->duration_sec,
        &active_goal_->start_stamp);
    }
    active_goal_.reset();
    jog_active_ = false;
    point_timing_pending_ = false;
    jog_timing_pending_ = false;
    jog_velocity_.fill(0.0);
    last_sent_velocity_.fill(0.0);
    if (disarm) {
      armed_ = false;
      stop_control_api();
      publish_armed_state();
    }
    RCLCPP_WARN(get_logger(), "JACO STOP/HOLD: %s", reason.c_str());
  }

  void handle_output_failure(const std::string & context)
  {
    fault_ = true;
    const std::string reason = context + ": " + last_error_;
    stop_motion(reason, true, "FAILED");
  }

  bool message_to_joint_vector(
    const thesis_interfaces::msg::JointCommand & message,
    JointVector & target,
    std::string & error) const
  {
    if (message.joint_names.size() != kJointCount ||
      message.positions.size() != kJointCount)
    {
      error = "expected six joints in canonical order";
      return false;
    }
    for (std::size_t index = 0; index < kJointCount; ++index) {
      if (message.joint_names[index] != kJointNames[index]) {
        error = "joint names are not in canonical order";
        return false;
      }
      target[index] = message.positions[index];
    }
    if (!thesis_hardware_bridge::inside_operational_limits(target)) {
      error = "target is non-finite or outside operational limits";
      return false;
    }
    return true;
  }

  bool state_is_fresh(const SteadyTime & now) const
  {
    if (!state_valid_) {
      return false;
    }
    return std::chrono::duration<double>(now - last_state_at_).count() <=
           state_timeout_sec_;
  }

  void command_callback(
    const thesis_interfaces::msg::JointCommand::SharedPtr message)
  {
    const std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    publish_timing(
      message->command_id, message->stamp, "ADAPTER_RECEIVE");
    const double duration = duration_seconds(message->duration);
    JointVector target{};
    std::string error;
    if (!message_to_joint_vector(*message, target, error) ||
      !std::isfinite(duration) || duration <= 0.0)
    {
      publish_execution(
        message->command_id, "REJECTED",
        error.empty() ? "invalid duration" : error,
        positions_, target, duration, nullptr);
      return;
    }
    if (!output_permitted_ || !armed_) {
      publish_execution(
        message->command_id, "DRY_RUN",
        "hardware output is disabled or not armed",
        positions_, target, duration, nullptr);
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (!state_is_fresh(now)) {
      publish_execution(
        message->command_id, "FAILED",
        "joint state is unavailable or stale",
        positions_, target, duration, nullptr);
      return;
    }
    if (active_goal_ || jog_active_) {
      publish_execution(
        message->command_id, "REJECTED",
        "another physical command is active",
        positions_, target, duration, nullptr);
      return;
    }

    try {
      ActiveGoal goal;
      goal.command_id = message->command_id;
      goal.intent_stamp = message->stamp;
      goal.start_stamp = get_clock()->now();
      goal.start = positions_;
      goal.target = target;
      goal.nominal_velocity =
        thesis_hardware_bridge::nominal_velocity(
        positions_, target, duration);
      goal.duration_sec = duration;
      // A newly accepted point remains at zero velocity until the runtime
      // supervisor explicitly publishes its first execution-control sample.
      goal.speed_scale = 0.0;
      goal.control_received = false;
      goal.started_at = now;
      goal.last_control_at = now;
      active_goal_ = goal;
      point_timing_pending_ = true;
    } catch (const std::exception & exception) {
      publish_execution(
        message->command_id, "REJECTED", exception.what(),
        positions_, target, duration, nullptr);
      return;
    }

    publish_execution(
      active_goal_->command_id, "ACCEPTED",
      "physical reference accepted by the USB adapter",
      active_goal_->start, active_goal_->target,
      active_goal_->duration_sec, &active_goal_->start_stamp);
  }

  void jog_callback(
    const thesis_interfaces::msg::JointCommand::SharedPtr message)
  {
    const std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    publish_timing(
      message->command_id, message->stamp, "ADAPTER_RECEIVE");
    if (!output_permitted_ || !armed_ || active_goal_) {
      return;
    }
    const double horizon = duration_seconds(message->duration);
    JointVector target{};
    std::string error;
    if (!message_to_joint_vector(*message, target, error) ||
      !std::isfinite(horizon) || horizon <= 0.0)
    {
      stop_motion("invalid supervised jog command", false, "CANCELED");
      return;
    }
    try {
      jog_velocity_ = thesis_hardware_bridge::nominal_velocity(
        positions_, target, horizon);
    } catch (const std::exception & exception) {
      stop_motion(
        std::string("invalid supervised jog command: ") + exception.what(),
        false, "CANCELED");
      return;
    }
    jog_active_ = true;
    jog_command_id_ = message->command_id;
    jog_intent_stamp_ = message->stamp;
    last_jog_at_ = std::chrono::steady_clock::now();
    jog_timing_pending_ = true;
  }

  void execution_control_callback(
    const thesis_interfaces::msg::ExecutionControl::SharedPtr message)
  {
    const std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    double scale = 0.0;
    try {
      scale = thesis_hardware_bridge::effective_speed_scale(
        message->state, message->speed_scale);
    } catch (const std::exception & exception) {
      stop_motion(
        std::string("invalid execution control: ") + exception.what(),
        true, "FAILED");
      fault_ = true;
      return;
    }

    if (message->state == "STOP" || scale <= 0.0) {
      publish_timing(
        message->command_id,
        active_goal_ ? active_goal_->intent_stamp : jog_intent_stamp_,
        "STOP_DETECTED", message->reason_code);
      stop_motion(
        "supervisor STOP: " + message->reason, false, "CANCELED");
      return;
    }
    if (active_goal_ &&
      message->command_id == active_goal_->command_id)
    {
      const auto now = std::chrono::steady_clock::now();
      active_goal_->control_received = true;
      if (std::abs(scale - active_goal_->speed_scale) > 1.0e-6) {
        replan_active_goal(scale, now);
      } else {
        active_goal_->last_control_at = now;
      }
    }
  }

  void replan_active_goal(double speed_scale, const SteadyTime & now)
  {
    if (!active_goal_) {
      return;
    }
    double remaining_nominal = 0.0;
    for (std::size_t index = 0; index < kJointCount; ++index) {
      const double error = std::abs(
        thesis_hardware_bridge::shortest_delta(
          index, active_goal_->target[index], positions_[index]));
      const double nominal_speed =
        std::abs(active_goal_->nominal_velocity[index]);
      if (nominal_speed > 1.0e-9) {
        remaining_nominal = std::max(
          remaining_nominal, error / nominal_speed);
      }
    }
    remaining_nominal = std::max(remaining_nominal, 0.15);
    const double replanned_duration = std::min(
      30.0, remaining_nominal / std::max(speed_scale, 0.05));
    try {
      active_goal_->nominal_velocity =
        thesis_hardware_bridge::nominal_velocity(
        positions_, active_goal_->target, remaining_nominal);
    } catch (const std::exception & exception) {
      fault_ = true;
      stop_motion(
        std::string("physical replan failed: ") + exception.what(),
        true, "FAILED");
      return;
    }
    active_goal_->start = positions_;
    active_goal_->start_stamp = get_clock()->now();
    active_goal_->duration_sec = replanned_duration;
    active_goal_->speed_scale = speed_scale;
    active_goal_->control_received = true;
    active_goal_->started_at = now;
    active_goal_->last_control_at = now;
    publish_execution(
      active_goal_->command_id, "ACCEPTED",
      "physical trajectory replanned at speed scale " +
      std::to_string(speed_scale),
      active_goal_->start, active_goal_->target,
      active_goal_->duration_sec, &active_goal_->start_stamp);
  }

  void arm_service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    const std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    if (!request->data) {
      stop_motion("runtime disarm requested", true, "CANCELED");
      response->success = true;
      response->message = "JACO output disarmed";
      return;
    }
    if (!output_permitted_) {
      response->success = false;
      response->message =
        "hardware_output_enabled is false; restart with explicit permission";
      return;
    }
    if (!connected_) {
      response->success = false;
      response->message = "JACO backend is not connected";
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (!state_is_fresh(now)) {
      response->success = false;
      response->message = "joint state is unavailable or stale";
      return;
    }
    if (armed_) {
      response->success = true;
      response->message = "JACO output is already armed";
      return;
    }
    consecutive_read_failures_ = 0;
    fault_ = false;
    last_error_.clear();
    if (!start_control_api()) {
      fault_ = true;
      stop_control_api();
      response->success = false;
      response->message = last_error_;
      publish_diagnostics();
      return;
    }
    armed_ = true;
    publish_armed_state();
    response->success = true;
    response->message =
      "JACO output armed; only supervised commands are accepted";
  }

  void feedback_tick()
  {
    const std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    const auto now = std::chrono::steady_clock::now();
    ++feedback_ticks_since_diagnostics_;
    const bool read_ok = read_hardware_state(now);
    if (read_ok) {
      consecutive_read_failures_ = 0;
      state_valid_ = true;
      last_state_at_ = now;
      if (!fault_) {
        last_error_.clear();
      }
    } else {
      ++consecutive_read_failures_;
      if (consecutive_read_failures_ >= maximum_read_failures_) {
        state_valid_ = false;
        fault_ = true;
        if (armed_) {
          stop_motion(
            "consecutive JACO state read failures", true, "FAILED");
        }
      }
    }
    if (read_ok && state_valid_) {
      publish_joint_state();
    }
  }

  void control_loop()
  {
    const auto period =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / control_rate_hz_));
    auto next_deadline = std::chrono::steady_clock::now();
    while (!control_stop_requested_.load() && rclcpp::ok()) {
      next_deadline += period;
      control_tick();
      const auto now = std::chrono::steady_clock::now();
      if (now > next_deadline + period * 5) {
        next_deadline = now;
        ++control_deadline_resets_;
      }
      std::this_thread::sleep_until(next_deadline);
    }
  }

  void control_tick()
  {
    const std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    const auto now = std::chrono::steady_clock::now();
    ++control_ticks_since_diagnostics_;
    update_mock_state(now);
    if (!armed_) {
      return;
    }
    if (!state_is_fresh(now)) {
      fault_ = true;
      stop_motion("JACO joint state watchdog expired", true, "FAILED");
      return;
    }

    JointVector command{};
    if (jog_active_) {
      const double age =
        std::chrono::duration<double>(now - last_jog_at_).count();
      if (age > jog_timeout_sec_) {
        publish_timing(
          jog_command_id_, jog_intent_stamp_,
          "HOLD_PUBLISH", "JOG_WATCHDOG_EXPIRED");
        stop_motion("JOG watchdog expired", false, "CANCELED");
        return;
      }
      command = jog_velocity_;
    } else if (active_goal_) {
      const double control_age = std::chrono::duration<double>(
        now - active_goal_->last_control_at).count();
      if (control_age > execution_control_timeout_sec_) {
        stop_motion(
          "execution-control watchdog expired", false, "CANCELED");
        return;
      }
      if (!active_goal_->control_received) {
        command.fill(0.0);
      } else if (thesis_hardware_bridge::target_reached(
          positions_, active_goal_->target, goal_tolerance_rad_))
      {
        const ActiveGoal completed = *active_goal_;
        send_zero_burst();
        active_goal_.reset();
        publish_execution(
          completed.command_id, "SUCCEEDED",
          "physical target reached within tolerance",
          completed.start, completed.target, completed.duration_sec,
          &completed.start_stamp);
        return;
      } else {
        const double elapsed = std::chrono::duration<double>(
          now - active_goal_->started_at).count();
        const double timeout = std::max(
          active_goal_->duration_sec * goal_timeout_factor_,
          active_goal_->duration_sec + goal_timeout_margin_sec_);
        if (elapsed > timeout) {
          stop_motion("physical goal timeout", false, "FAILED");
          return;
        }
        command = thesis_hardware_bridge::track_target_velocity(
          positions_,
          active_goal_->target,
          active_goal_->nominal_velocity,
          active_goal_->speed_scale,
          1.0 / control_rate_hz_,
          goal_tolerance_rad_);
      }
    }

    if (!send_velocity(command)) {
      handle_output_failure("physical command transmission failed");
      return;
    }
    ++output_commands_since_diagnostics_;
    if (active_goal_ && active_goal_->control_received &&
      point_timing_pending_)
    {
      publish_timing(
        active_goal_->command_id, active_goal_->intent_stamp,
        "CONTROLLER_PUBLISH");
      point_timing_pending_ = false;
    } else if (jog_active_ && jog_timing_pending_) {
      publish_timing(
        jog_command_id_, jog_intent_stamp_,
        "CONTROLLER_PUBLISH");
      jog_timing_pending_ = false;
    }
  }

  void publish_joint_state()
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "j2n6s300_link_base";
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
      "j2n6s300_joint_finger_tip_3",
    };
    message.position.resize(kPublishedJointCount, 0.0);
    message.velocity.resize(kPublishedJointCount, 0.0);
    for (std::size_t index = 0; index < kJointCount; ++index) {
      message.position[index] = positions_[index];
      message.velocity[index] = velocities_[index];
    }
    message.position[6] = fingers_[0];
    message.position[8] = fingers_[1];
    message.position[10] = fingers_[2];
    joint_state_publisher_->publish(message);
  }

  void publish_execution(
    const std::string & command_id,
    const std::string & status,
    const std::string & detail,
    const JointVector & start,
    const JointVector & target,
    double duration,
    const builtin_interfaces::msg::Time * start_stamp)
  {
    thesis_interfaces::msg::ExecutionTrajectory message;
    message.stamp = get_clock()->now();
    if (start_stamp != nullptr) {
      message.start_time = *start_stamp;
    }
    message.command_id = command_id;
    message.status = status;
    message.detail = detail;
    message.joint_names.assign(kJointNames.begin(), kJointNames.end());
    message.start_positions.assign(start.begin(), start.end());
    message.target_positions.assign(target.begin(), target.end());
    message.duration_sec = duration;
    execution_publisher_->publish(message);
  }

  void publish_timing(
    const std::string & command_id,
    const builtin_interfaces::msg::Time & intent_stamp,
    const std::string & stage,
    const std::string & detail = "")
  {
    thesis_interfaces::msg::PipelineTiming message;
    message.stamp = get_clock()->now();
    message.intent_stamp = intent_stamp;
    message.command_id = command_id;
    message.source = "hardware_adapter";
    message.stage = stage;
    message.sequence = ++timing_sequence_;
    message.monotonic_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    message.internal_duration_sec = -1.0;
    message.detail = detail;
    timing_publisher_->publish(message);
  }

  void publish_armed_state()
  {
    std_msgs::msg::Bool message;
    message.data = armed_;
    armed_publisher_->publish(message);
  }

  void publish_connection_state()
  {
    std_msgs::msg::Bool message;
    message.data = connected_;
    connected_publisher_->publish(message);
  }

  void publish_diagnostics()
  {
    const std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    const auto rate_now = std::chrono::steady_clock::now();
    const double rate_interval = std::chrono::duration<double>(
      rate_now - last_rate_sample_at_).count();
    if (rate_interval > 0.0) {
      measured_control_rate_hz_ =
        static_cast<double>(control_ticks_since_diagnostics_) /
        rate_interval;
      measured_feedback_rate_hz_ =
        static_cast<double>(feedback_ticks_since_diagnostics_) /
        rate_interval;
      measured_output_command_rate_hz_ =
        static_cast<double>(output_commands_since_diagnostics_) /
        rate_interval;
      control_ticks_since_diagnostics_ = 0;
      feedback_ticks_since_diagnostics_ = 0;
      output_commands_since_diagnostics_ = 0;
      last_rate_sample_at_ = rate_now;
    }
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = get_clock()->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "thesis_hardware_bridge/JACO2";
    status.hardware_id = serial_number_.empty() ? "UNAVAILABLE" : serial_number_;
    if (fault_ || !connected_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = last_error_.empty() ?
        "hardware backend unavailable" : last_error_;
    } else if (!output_permitted_ || !armed_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "read-only or runtime output disarmed";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "connected, state fresh and output armed";
    }
    status.values = {
      key_value("backend", mock_hardware_ ? "mock" : "kinova_usb"),
      key_value("usb_lock_held", bool_text(usb_lock_held_)),
      key_value("serial_reported", bool_text(serial_reported_)),
      key_value(
        "expected_serial_configured",
        bool_text(expected_serial_configured_)),
      key_value("device_count", std::to_string(device_count_)),
      key_value(
        "selected_device_index", std::to_string(selected_device_index_)),
      key_value("connected", bool_text(connected_)),
      key_value("output_permitted", bool_text(output_permitted_)),
      key_value("armed", bool_text(armed_)),
      key_value("state_valid", bool_text(state_valid_)),
      key_value("fault", bool_text(fault_)),
      key_value(
        "read_failures", std::to_string(consecutive_read_failures_)),
      key_value(
        "control_rate_hz_measured",
        std::to_string(measured_control_rate_hz_)),
      key_value(
        "feedback_rate_hz_measured",
        std::to_string(measured_feedback_rate_hz_)),
      key_value(
        "output_command_rate_hz_measured",
        std::to_string(measured_output_command_rate_hz_)),
      key_value(
        "control_deadline_resets",
        std::to_string(control_deadline_resets_.load())),
      key_value(
        "mode", active_goal_ ? "trajectory" :
        (jog_active_ ? "jog" : "hold")),
    };
    array.status.push_back(status);
    diagnostic_publisher_->publish(array);
    publish_connection_state();
    publish_armed_state();
  }

  bool mock_hardware_{false};
  bool output_permitted_{false};
  bool connected_{false};
  bool armed_{false};
  bool fault_{false};
  bool api_open_{false};
  bool control_api_started_{false};
  bool state_valid_{false};
  bool shutdown_complete_{false};
  bool jog_active_{false};
  bool point_timing_pending_{false};
  bool jog_timing_pending_{false};
  double state_publish_rate_hz_{20.0};
  double control_rate_hz_{100.0};
  double state_timeout_sec_{0.20};
  double jog_timeout_sec_{0.18};
  double execution_control_timeout_sec_{0.35};
  double goal_tolerance_rad_{0.005};
  double goal_timeout_factor_{4.0};
  double goal_timeout_margin_sec_{2.0};
  int maximum_read_failures_{3};
  int initialization_read_attempts_{10};
  int initialization_retry_delay_ms_{200};
  int zero_command_count_{10};
  int usb_lock_fd_{-1};
  int device_count_{0};
  int selected_device_index_{-1};
  std::size_t control_ticks_since_diagnostics_{0};
  std::size_t feedback_ticks_since_diagnostics_{0};
  std::size_t output_commands_since_diagnostics_{0};
  int consecutive_read_failures_{0};
  uint32_t timing_sequence_{0};
  double measured_control_rate_hz_{0.0};
  double measured_feedback_rate_hz_{0.0};
  double measured_output_command_rate_hz_{0.0};
  bool serial_reported_{false};
  bool expected_serial_configured_{false};
  bool usb_lock_held_{false};
  std::string serial_number_;
  std::string usb_lock_file_;
  std::string last_error_;
  std::string jog_command_id_;
  builtin_interfaces::msg::Time jog_intent_stamp_;
  JointVector positions_{};
  JointVector velocities_{};
  JointVector jog_velocity_{};
  JointVector last_sent_velocity_{};
  std::array<double, 3> fingers_{};
  SteadyTime last_state_at_{};
  SteadyTime last_jog_at_{};
  SteadyTime last_mock_update_{};
  SteadyTime last_rate_sample_at_{std::chrono::steady_clock::now()};
  std::optional<ActiveGoal> active_goal_;
  std::recursive_mutex state_mutex_;
  std::recursive_mutex api_mutex_;
  std::atomic<bool> control_stop_requested_{false};
  std::atomic<std::size_t> control_deadline_resets_{0};
  std::thread control_thread_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_publisher_;
  rclcpp::Publisher<thesis_interfaces::msg::ExecutionTrajectory>::SharedPtr
    execution_publisher_;
  rclcpp::Publisher<thesis_interfaces::msg::PipelineTiming>::SharedPtr
    timing_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostic_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_publisher_;
  rclcpp::Subscription<thesis_interfaces::msg::JointCommand>::SharedPtr
    command_subscription_;
  rclcpp::Subscription<thesis_interfaces::msg::JointCommand>::SharedPtr
    jog_subscription_;
  rclcpp::Subscription<thesis_interfaces::msg::ExecutionControl>::SharedPtr
    control_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<JacoHardwareNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    std::cerr << "JACO hardware adapter stopped: "
              << exception.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
