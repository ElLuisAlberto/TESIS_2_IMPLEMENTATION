#include "thesis_jaco_perception/self_filter_geometry.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

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
constexpr char kCameraOpticalFrame[] = "d435i_depth_optical_frame";
constexpr std::size_t kCapsuleCount = 6U;
constexpr std::size_t kClassCount = 3U;
constexpr std::size_t kMaximumSamples = 3U;
constexpr double kTimeoutSeconds = 5.0;
constexpr double kTfTimeoutSeconds = 0.05;
constexpr double kEpsilonTotalM = 0.07;
constexpr double kCoverageMarginM = 0.003;

const std::array<const char *, kCapsuleCount> kStartFrames = {
  "j2n6s300_link_base", "j2n6s300_link_1", "j2n6s300_link_2",
  "j2n6s300_link_3", "j2n6s300_link_4", "j2n6s300_link_5"};
const std::array<const char *, kCapsuleCount> kEndFrames = {
  "j2n6s300_link_1", "j2n6s300_link_2", "j2n6s300_link_3",
  "j2n6s300_link_4", "j2n6s300_link_5", "j2n6s300_end_effector"};
const std::array<double, kCapsuleCount> kRadiiM = {0.105, 0.105, 0.085, 0.075, 0.085, 0.090};
const std::array<const char *, kCapsuleCount> kCapsuleNames = {
  "base_to_shoulder", "shoulder_to_upper_arm", "upper_arm_to_forearm",
  "forearm_to_wrist_1", "wrist_1_to_wrist_2", "wrist_2_to_tool"};

enum class ProbeResult {kPending, kSuccess, kFrameError, kNoCloud, kTfError};

struct DeltaStatistics
{
  std::size_t count{};
  double minimum{std::numeric_limits<double>::quiet_NaN()};
  double median{std::numeric_limits<double>::quiet_NaN()};
  double p90{std::numeric_limits<double>::quiet_NaN()};
  double maximum{std::numeric_limits<double>::quiet_NaN()};
};

struct RayAccumulator
{
  std::vector<double> deltas;
  std::array<std::size_t, kClassCount> bands{};
};

struct CoverageAccumulator
{
  std::vector<double> axis_distances;
  std::vector<double> clearances;
  std::vector<double> required_radii;
};

struct CoverageStatistics
{
  std::size_t count{};
  double p50{std::numeric_limits<double>::quiet_NaN()};
  double p90{std::numeric_limits<double>::quiet_NaN()};
  double p95{std::numeric_limits<double>::quiet_NaN()};
  double p99{std::numeric_limits<double>::quiet_NaN()};
  double maximum{std::numeric_limits<double>::quiet_NaN()};
};

enum class CoverageLabel {kNoResidual, kLocalCoverageCandidate, kExcessiveOrAmbiguousExpansion};

struct ParsedCommandLine
{
  std::array<double, kCapsuleCount> radii{kRadiiM};
  bool custom_radii{};
  std::vector<char *> ros_arguments;
};

Point3 translation_of(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & value = transform.transform.translation;
  return {value.x, value.y, value.z};
}

double distance(const Point3 & first, const Point3 & second)
{
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  const double dz = first.z - second.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::size_t class_index(const PointClassification classification)
{
  switch (classification) {
    case PointClassification::EXTERNAL_CONFIRMED: return 0U;
    case PointClassification::SELF_CONFIRMED: return 1U;
    case PointClassification::OCCLUDED_UNKNOWN: return 2U;
  }
  return 0U;
}

const char * class_name(const std::size_t index)
{
  constexpr std::array<const char *, kClassCount> names = {
    "EXTERNAL_CONFIRMED", "SELF_CONFIRMED", "OCCLUDED_UNKNOWN"};
  return names.at(index);
}

double percentile(const std::vector<double> & sorted, const double fraction)
{
  const double position = fraction * static_cast<double>(sorted.size() - 1U);
  const std::size_t lower = static_cast<std::size_t>(std::floor(position));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
  return sorted[lower] + (position - static_cast<double>(lower)) *
    (sorted[upper] - sorted[lower]);
}

CoverageStatistics coverage_statistics(std::vector<double> values)
{
  if (values.empty()) {
    return {};
  }
  std::sort(values.begin(), values.end());
  return {values.size(), percentile(values, 0.5), percentile(values, 0.9),
    percentile(values, 0.95), percentile(values, 0.99), values.back()};
}

const char * coverage_label_name(const CoverageLabel label)
{
  switch (label) {
    case CoverageLabel::kNoResidual: return "NO_RESIDUAL";
    case CoverageLabel::kLocalCoverageCandidate: return "LOCAL_COVERAGE_CANDIDATE";
    case CoverageLabel::kExcessiveOrAmbiguousExpansion:
      return "EXCESSIVE_OR_AMBIGUOUS_EXPANSION";
  }
  return "EXCESSIVE_OR_AMBIGUOUS_EXPANSION";
}

CoverageLabel coverage_label(const CoverageStatistics & statistics, const double radius)
{
  if (statistics.count == 0U) {
    return CoverageLabel::kNoResidual;
  }
  const double p95_increment = statistics.p95 - radius;
  const double p99_spread = statistics.p99 - statistics.p95;
  if (statistics.count >= 20U && p95_increment > 0.0 && p95_increment <= 0.025 &&
    p99_spread <= 0.020)
  {
    return CoverageLabel::kLocalCoverageCandidate;
  }
  return CoverageLabel::kExcessiveOrAmbiguousExpansion;
}

bool parse_capsule_radii_csv(
  const std::string_view input, std::array<double, kCapsuleCount> & radii,
  std::string & error)
{
  std::array<double, kCapsuleCount> parsed{};
  std::size_t index = 0U;
  std::size_t begin = 0U;
  while (begin <= input.size()) {
    const std::size_t end = input.find(',', begin);
    const std::string token(input.substr(begin, end == std::string_view::npos ?
      std::string_view::npos : end - begin));
    if (token.empty() || index >= kCapsuleCount) {
      error = "--capsule-radii-m must contain exactly six finite positive comma-separated values";
      return false;
    }
    char * parsed_end = nullptr;
    const double value = std::strtod(token.c_str(), &parsed_end);
    if (parsed_end == token.c_str() || *parsed_end != '\0' || !std::isfinite(value) || value <= 0.0) {
      error = "--capsule-radii-m values must be finite and positive";
      return false;
    }
    parsed[index++] = value;
    if (end == std::string_view::npos) {
      break;
    }
    begin = end + 1U;
  }
  if (index != kCapsuleCount) {
    error = "--capsule-radii-m must contain exactly six finite positive comma-separated values";
    return false;
  }
  radii = parsed;
  return true;
}

bool parse_command_line(int argc, char ** argv, ParsedCommandLine & parsed, std::string & error)
{
  parsed.ros_arguments.clear();
  parsed.ros_arguments.push_back(argv[0]);
  for (int index = 1; index < argc; ++index) {
    if (std::string_view(argv[index]) != "--capsule-radii-m") {
      parsed.ros_arguments.push_back(argv[index]);
      continue;
    }
    if (parsed.custom_radii || index + 1 >= argc ||
      !parse_capsule_radii_csv(argv[index + 1], parsed.radii, error))
    {
      if (error.empty()) {
        error = "--capsule-radii-m may be specified once with six values";
      }
      return false;
    }
    parsed.custom_radii = true;
    ++index;
  }
  return true;
}

std::string radii_text(const std::array<double, kCapsuleCount> & radii)
{
  std::ostringstream output;
  output << std::setprecision(17);
  for (std::size_t index = 0; index < radii.size(); ++index) {
    if (index != 0U) {
      output << ',';
    }
    output << radii[index];
  }
  return output.str();
}

DeltaStatistics delta_statistics(std::vector<double> values)
{
  if (values.empty()) {
    return {};
  }
  std::sort(values.begin(), values.end());
  return {values.size(), values.front(), percentile(values, 0.5), percentile(values, 0.9), values.back()};
}

class JacoSelfResidualProbe : public rclcpp::Node
{
public:
  explicit JacoSelfResidualProbe(
    const rclcpp::NodeOptions & options, const std::array<double, kCapsuleCount> & radii,
    const bool custom_radii)
  : Node("jaco_self_residual_probe", options), tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_, this), radii_(radii), custom_radii_(custom_radii)
  {
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/thesis/jaco_perception/points_external_candidate", qos,
      std::bind(&JacoSelfResidualProbe::cloud_callback, this, std::placeholders::_1));
    timeout_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(kTimeoutSeconds)),
      std::bind(&JacoSelfResidualProbe::timeout_callback, this));
    RCLCPP_INFO(get_logger(),
      "READ_ONLY_RAY_RESIDUAL_PROBE input=/thesis/jaco_perception/points_external_candidate "
      "samples_max=%zu timeout_s=%.1f frame=world epsilon_m=%.6f radii_source=%s radii_m=%s",
      kMaximumSamples, kTimeoutSeconds, kEpsilonTotalM,
      custom_radii_ ? "custom_csv" : "historical_default", radii_text(radii_).c_str());
  }

  ProbeResult result() const {return result_;}

private:
  bool geometry_at_stamp(
    const builtin_interfaces::msg::Time & stamp, Point3 & camera_origin,
    std::vector<Capsule3> & capsules, std::string & error)
  {
    try {
      const rclcpp::Time exact_time(stamp);
      const auto timeout = rclcpp::Duration::from_seconds(kTfTimeoutSeconds);
      camera_origin = translation_of(tf_buffer_.lookupTransform(
        kWorldFrame, kCameraOpticalFrame, exact_time, timeout));
      capsules.clear();
      capsules.reserve(kCapsuleCount);
      for (std::size_t index = 0; index < kCapsuleCount; ++index) {
        const auto start = tf_buffer_.lookupTransform(
          kWorldFrame, kStartFrames[index], exact_time, timeout);
        const auto end = tf_buffer_.lookupTransform(
          kWorldFrame, kEndFrames[index], exact_time, timeout);
        capsules.push_back({translation_of(start), translation_of(end), radii_[index]});
      }
    } catch (const tf2::TransformException & exception) {
      error = exception.what();
      return false;
    }
    return true;
  }

  void process_cloud(
    const sensor_msgs::msg::PointCloud2 & cloud, const Point3 & camera_origin,
    const std::vector<Capsule3> & capsules)
  {
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
    const std::size_t points = static_cast<std::size_t>(cloud.width) * cloud.height;
    for (std::size_t index = 0; index < points; ++index, ++x, ++y, ++z) {
      const Point3 point{*x, *y, *z};
      if (!is_valid_point(point)) {
        ++invalid_points_;
        continue;
      }
      const PointClassification classification = classify_observed_point(
        camera_origin, point, capsules, kEpsilonTotalM);
      ++class_counts_.at(class_index(classification));
      if (classification == PointClassification::EXTERNAL_CONFIRMED) {
        std::size_t nearest_capsule = 0U;
        double nearest_axis_distance = std::numeric_limits<double>::infinity();
        for (std::size_t capsule = 0; capsule < capsules.size(); ++capsule) {
          const double axis_distance = point_segment_distance(
            point, capsules[capsule].start, capsules[capsule].end);
          const double surface_distance = axis_distance - capsules[capsule].radius;
          const double nearest_surface_distance = nearest_axis_distance -
            capsules[nearest_capsule].radius;
          if (surface_distance < nearest_surface_distance) {
            nearest_capsule = capsule;
            nearest_axis_distance = axis_distance;
          }
        }
        auto & coverage = coverage_.at(nearest_capsule);
        coverage.axis_distances.push_back(nearest_axis_distance);
        coverage.clearances.push_back(nearest_axis_distance - capsules[nearest_capsule].radius);
        coverage.required_radii.push_back(nearest_axis_distance + kCoverageMarginM);
      }
      const double r = distance(point, camera_origin);
      const auto intersection = first_ray_capsules_intersection(
        {camera_origin, {point.x - camera_origin.x, point.y - camera_origin.y,
          point.z - camera_origin.z}}, capsules);
      if (!intersection.has_value()) {
        ++no_intersection_points_;
        continue;
      }
      const double delta = r - intersection->distance;
      auto & accumulator = accumulators_.at(class_index(classification)).at(
        intersection->capsule_index);
      accumulator.deltas.push_back(delta);
      ++accumulator.bands.at(class_index(classification));
    }
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    if (result_ != ProbeResult::kPending) {
      return;
    }
    if (cloud->header.frame_id != kWorldFrame) {
      RCLCPP_ERROR(get_logger(), "PROBE_FRAME_ERROR expected=world received=%s",
        cloud->header.frame_id.c_str());
      finish(ProbeResult::kFrameError);
      return;
    }
    Point3 camera_origin;
    std::vector<Capsule3> capsules;
    std::string tf_error;
    if (!geometry_at_stamp(cloud->header.stamp, camera_origin, capsules, tf_error)) {
      ++discarded_tf_;
      last_tf_error_ = tf_error;
      return;
    }
    try {
      process_cloud(*cloud, camera_origin, capsules);
    } catch (const std::exception & exception) {
      ++discarded_invalid_cloud_;
      last_tf_error_ = exception.what();
      return;
    }
    ++samples_used_;
    last_stamp_ = cloud->header.stamp;
    RCLCPP_INFO(get_logger(), "TF_USED frame=world stamp=%d.%09u sample=%zu",
      last_stamp_.sec, last_stamp_.nanosec, samples_used_);
    if (samples_used_ >= kMaximumSamples) {
      finish(ProbeResult::kSuccess);
    }
  }

  void timeout_callback()
  {
    finish(samples_used_ > 0U ? ProbeResult::kSuccess :
      (discarded_tf_ > 0U ? ProbeResult::kTfError : ProbeResult::kNoCloud));
  }

  void report()
  {
    RCLCPP_INFO(get_logger(),
      "RAY_PROBE_SUMMARY samples_used=%zu points_valid=%zu external=%zu self=%zu occluded=%zu "
      "no_intersection=%zu invalid=%zu discarded_tf=%zu discarded_invalid_cloud=%zu "
      "epsilon_m=%.6f last_tf_frame=world last_stamp=%d.%09u",
      samples_used_, class_counts_[0] + class_counts_[1] + class_counts_[2], class_counts_[0],
      class_counts_[1], class_counts_[2], no_intersection_points_, invalid_points_, discarded_tf_,
      discarded_invalid_cloud_, kEpsilonTotalM, last_stamp_.sec, last_stamp_.nanosec);
    for (std::size_t classification = 0; classification < kClassCount; ++classification) {
      for (std::size_t capsule = 0; capsule < kCapsuleCount; ++capsule) {
        const auto statistics = delta_statistics(accumulators_[classification][capsule].deltas);
        const auto & bands = accumulators_[classification][capsule].bands;
        RCLCPP_INFO(get_logger(),
          "RAY_RESIDUAL class=%s capsule=%zu name=%s count=%zu min=%.6f median=%.6f "
          "p90=%.6f max=%.6f band_r_lt_s_minus_epsilon=%zu "
          "band_abs_delta_le_epsilon=%zu band_r_gt_s_plus_epsilon=%zu",
          class_name(classification), capsule, kCapsuleNames[capsule], statistics.count,
          statistics.minimum, statistics.median, statistics.p90, statistics.maximum,
          bands[0], bands[1], bands[2]);
      }
    }
    for (std::size_t capsule = 0; capsule < kCapsuleCount; ++capsule) {
      const auto required = coverage_statistics(coverage_[capsule].required_radii);
      const auto axis_distance = coverage_statistics(coverage_[capsule].axis_distances);
      const auto clearance = coverage_statistics(coverage_[capsule].clearances);
      const double increment_p95 = required.p95 - radii_[capsule];
      const double increment_p99 = required.p99 - radii_[capsule];
      RCLCPP_INFO(get_logger(),
        "RESIDUAL_COVERAGE capsule=%zu name=%s association=DIAGNOSTIC_ONLY "
        "count=%zu radius_v1_m=%.17g axis_distance_p50_m=%.17g clearance_p50_m=%.17g "
        "required_radius_p50_m=%.17g required_radius_p90_m=%.17g "
        "required_radius_p95_m=%.17g required_radius_p99_m=%.17g "
        "increment_p95_m=%.17g increment_p99_m=%.17g maximum_required_radius_m=%.17g "
        "label=%s",
        capsule, kCapsuleNames[capsule], required.count, radii_[capsule], axis_distance.p50,
        clearance.p50, required.p50, required.p90, required.p95, required.p99,
        increment_p95, increment_p99, required.maximum,
        coverage_label_name(coverage_label(required, radii_[capsule])));
    }
  }

  void finish(const ProbeResult result)
  {
    if (result_ != ProbeResult::kPending) {
      return;
    }
    result_ = result;
    report();
    cloud_subscription_.reset();
    timeout_timer_->cancel();
    rclcpp::shutdown();
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  std::array<std::array<RayAccumulator, kCapsuleCount>, kClassCount> accumulators_;
  std::array<CoverageAccumulator, kCapsuleCount> coverage_;
  std::array<double, kCapsuleCount> radii_;
  bool custom_radii_{};
  std::array<std::size_t, kClassCount> class_counts_{};
  builtin_interfaces::msg::Time last_stamp_;
  std::string last_tf_error_;
  std::size_t samples_used_{};
  std::size_t no_intersection_points_{};
  std::size_t invalid_points_{};
  std::size_t discarded_tf_{};
  std::size_t discarded_invalid_cloud_{};
  ProbeResult result_{ProbeResult::kPending};
};

int exit_code(const ProbeResult result)
{
  switch (result) {
    case ProbeResult::kSuccess: return 0;
    case ProbeResult::kFrameError: return 2;
    case ProbeResult::kNoCloud: return 3;
    case ProbeResult::kTfError: return 4;
    case ProbeResult::kPending: return 5;
  }
  return 5;
}

}  // namespace
}  // namespace thesis_jaco_perception

int main(int argc, char ** argv)
{
  thesis_jaco_perception::ParsedCommandLine command_line;
  std::string error;
  if (!thesis_jaco_perception::parse_command_line(argc, argv, command_line, error)) {
    std::fprintf(stderr, "PROBE_ARGUMENT_ERROR %s\n", error.c_str());
    return 6;
  }
  int ros_argc = static_cast<int>(command_line.ros_arguments.size());
  rclcpp::init(ros_argc, command_line.ros_arguments.data());
  rclcpp::NodeOptions options;
  options.enable_rosout(false).start_parameter_services(false).start_parameter_event_publisher(false);
  auto probe = std::make_shared<thesis_jaco_perception::JacoSelfResidualProbe>(
    options, command_line.radii, command_line.custom_radii);
  rclcpp::spin(probe);
  return thesis_jaco_perception::exit_code(probe->result());
}
