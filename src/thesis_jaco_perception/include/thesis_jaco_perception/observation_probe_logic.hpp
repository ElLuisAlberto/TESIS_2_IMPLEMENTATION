#ifndef THESIS_JACO_PERCEPTION__OBSERVATION_PROBE_LOGIC_HPP_
#define THESIS_JACO_PERCEPTION__OBSERVATION_PROBE_LOGIC_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace thesis_jaco_perception
{

constexpr double kMinimumProbeRangeM = 0.10;
constexpr double kStaleStampAgeSec = 0.50;

struct CameraInfoObservation
{
  bool received{false};
  std::string frame_id;
  std::uint32_t width{0U};
  std::uint32_t height{0U};
  std::array<double, 9> k{};
  std::array<double, 12> p{};
  std::vector<double> d;
  std::string distortion_model;
};

struct ObservationMetrics
{
  std::string pointcloud_frame_id;
  std::uint32_t width{0U};
  std::uint32_t height{0U};
  std::vector<std::string> fields;
  std::uint32_t point_step{0U};
  double max_range_m{0.0};
  std::size_t clouds_received{0U};
  double observed_frequency_hz{0.0};
  double last_stamp_age_sec{0.0};
  std::size_t null_stamps{0U};
  std::size_t future_stamps{0U};
  std::size_t stale_stamps{0U};
  std::size_t finite_points{0U};
  std::size_t below_min_range_points{0U};
  std::size_t in_range_points{0U};
  std::size_t above_max_range_points{0U};
  CameraInfoObservation camera_info;
};

void validate_probe_parameters(double observation_window_sec, double max_range_m);
void observe_pointcloud(
  const sensor_msgs::msg::PointCloud2 & cloud, double received_ros_sec,
  double max_range_m, ObservationMetrics & metrics);
void observe_camera_info(
  const sensor_msgs::msg::CameraInfo & camera_info, ObservationMetrics & metrics);
double point_proportion(std::size_t count, std::size_t finite_points);
bool write_json_report_if_requested(
  const std::string & report_path, const ObservationMetrics & metrics);

}  // namespace thesis_jaco_perception

#endif  // THESIS_JACO_PERCEPTION__OBSERVATION_PROBE_LOGIC_HPP_
