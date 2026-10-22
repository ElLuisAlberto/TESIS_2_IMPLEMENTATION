#include "thesis_jaco_perception/observation_probe_logic.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace thesis_jaco_perception
{
namespace
{
std::string json_escape(const std::string & value)
{
  std::string result;
  for (const char character : value) {
    if (character == '"' || character == '\\') {
      result.push_back('\\');
    }
    result.push_back(character);
  }
  return result;
}

template<typename Container>
void json_numbers(std::ostream & output, const Container & values)
{
  output << '[';
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0U) {
      output << ',';
    }
    output << values[index];
  }
  output << ']';
}

double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}
}  // namespace

void validate_probe_parameters(double observation_window_sec, double max_range_m)
{
  if (!std::isfinite(observation_window_sec) || observation_window_sec <= 0.0) {
    throw std::invalid_argument("observation_window_sec must be finite and positive");
  }
  if (!std::isfinite(max_range_m) || max_range_m <= kMinimumProbeRangeM) {
    throw std::invalid_argument("max_range_m must be finite and greater than 0.10");
  }
}

void observe_pointcloud(
  const sensor_msgs::msg::PointCloud2 & cloud, double received_ros_sec,
  double max_range_m, ObservationMetrics & metrics)
{
  validate_probe_parameters(1.0, max_range_m);
  if (!std::isfinite(received_ros_sec)) {
    throw std::invalid_argument("received ROS time must be finite");
  }
  metrics.pointcloud_frame_id = cloud.header.frame_id;
  metrics.width = cloud.width;
  metrics.height = cloud.height;
  metrics.point_step = cloud.point_step;
  metrics.max_range_m = max_range_m;
  metrics.fields.clear();
  for (const auto & field : cloud.fields) {
    metrics.fields.push_back(field.name);
  }
  ++metrics.clouds_received;
  const double stamp = stamp_seconds(cloud.header.stamp);
  metrics.last_stamp_age_sec = received_ros_sec - stamp;
  if (stamp <= 0.0) {
    ++metrics.null_stamps;
  } else if (metrics.last_stamp_age_sec < 0.0) {
    ++metrics.future_stamps;
  } else if (metrics.last_stamp_age_sec > kStaleStampAgeSec) {
    ++metrics.stale_stamps;
  }

  sensor_msgs::PointCloud2ConstIterator<float> x_iterator(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_iterator(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_iterator(cloud, "z");
  const std::size_t count = static_cast<std::size_t>(cloud.width) * cloud.height;
  for (std::size_t index = 0; index < count; ++index, ++x_iterator, ++y_iterator, ++z_iterator) {
    const double x = *x_iterator;
    const double y = *y_iterator;
    const double z = *z_iterator;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    ++metrics.finite_points;
    const double range = std::sqrt(x * x + y * y + z * z);
    if (range < kMinimumProbeRangeM) {
      ++metrics.below_min_range_points;
    } else if (range <= max_range_m) {
      ++metrics.in_range_points;
    } else {
      ++metrics.above_max_range_points;
    }
  }
}

void observe_camera_info(
  const sensor_msgs::msg::CameraInfo & camera_info, ObservationMetrics & metrics)
{
  metrics.camera_info.received = true;
  metrics.camera_info.frame_id = camera_info.header.frame_id;
  metrics.camera_info.width = camera_info.width;
  metrics.camera_info.height = camera_info.height;
  metrics.camera_info.k = camera_info.k;
  metrics.camera_info.p = camera_info.p;
  metrics.camera_info.d = camera_info.d;
  metrics.camera_info.distortion_model = camera_info.distortion_model;
}

double point_proportion(std::size_t count, std::size_t finite_points)
{
  return finite_points == 0U ? 0.0 : static_cast<double>(count) / finite_points;
}

bool write_json_report_if_requested(
  const std::string & report_path, const ObservationMetrics & metrics)
{
  if (report_path.empty()) {
    return false;
  }
  std::ofstream output(report_path, std::ios::out | std::ios::trunc);
  if (!output.is_open()) {
    throw std::runtime_error("cannot open requested report_path");
  }
  output << std::setprecision(17) << "{\n";
  output << "  \"pointcloud\": {\"frame_id\": \"" <<
    json_escape(metrics.pointcloud_frame_id) << "\", \"width\": " << metrics.width <<
    ", \"height\": " << metrics.height << ", \"point_step\": " << metrics.point_step <<
    ", \"fields\": [";
  for (std::size_t index = 0; index < metrics.fields.size(); ++index) {
    if (index != 0U) {
      output << ',';
    }
    output << "\"" << json_escape(metrics.fields[index]) << "\"";
  }
  output << "]},\n";
  output << "  \"clouds_received\": " << metrics.clouds_received << ",\n";
  output << "  \"observed_frequency_hz\": " << metrics.observed_frequency_hz << ",\n";
  output << "  \"last_stamp_age_sec\": " << metrics.last_stamp_age_sec << ",\n";
  output << "  \"stamp_quality\": {\"null\": " << metrics.null_stamps <<
    ", \"future\": " << metrics.future_stamps << ", \"stale\": " <<
    metrics.stale_stamps << "},\n";
  output << "  \"ranges\": {\"finite\": " << metrics.finite_points <<
    ", \"below_0_10_m\": " << metrics.below_min_range_points <<
    ", \"within_min_to_max_m\": " << metrics.in_range_points <<
    ", \"above_max_m\": " << metrics.above_max_range_points <<
    ", \"max_range_m\": " << metrics.max_range_m << "},\n";
  output << "  \"camera_info\": {\"received\": " <<
    (metrics.camera_info.received ? "true" : "false");
  if (metrics.camera_info.received) {
    output << ", \"frame_id\": \"" << json_escape(metrics.camera_info.frame_id) << "\"";
    output << ", \"width\": " << metrics.camera_info.width;
    output << ", \"height\": " << metrics.camera_info.height;
    output << ", \"K\": "; json_numbers(output, metrics.camera_info.k);
    output << ", \"P\": "; json_numbers(output, metrics.camera_info.p);
    output << ", \"D\": "; json_numbers(output, metrics.camera_info.d);
    output << ", \"distortion_model\": \"" <<
      json_escape(metrics.camera_info.distortion_model) << "\"";
  }
  output << "}\n}\n";
  if (!output.good()) {
    throw std::runtime_error("failed while writing requested report_path");
  }
  return true;
}

}  // namespace thesis_jaco_perception
