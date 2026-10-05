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

#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <thesis_interfaces/msg/obstacle.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace
{
double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1.0e-9;
}

std::string number(double value, int precision = 3)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

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
}  // namespace

class ObstacleExtractor : public rclcpp::Node
{
public:
  ObstacleExtractor()
  : Node("obstacle_extractor")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/thesis/perception/points_filtered");
    obstacle_topic_ = declare_parameter<std::string>(
      "obstacle_topic", "/thesis/perception/obstacle_candidate");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/thesis/perception/extraction_diagnostics");
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/thesis/perception/obstacle_marker");
    cluster_tolerance_m_ = declare_parameter<double>(
      "cluster_tolerance_m", 0.06);
    min_cluster_points_ = declare_parameter<int>("min_cluster_points", 30);
    max_cluster_points_ = declare_parameter<int>(
      "max_cluster_points", 200000);
    min_obstacle_radius_m_ = declare_parameter<double>(
      "min_obstacle_radius_m", 0.02);
    max_obstacle_radius_m_ = declare_parameter<double>(
      "max_obstacle_radius_m", 0.75);
    max_obstacle_distance_m_ = declare_parameter<double>(
      "max_obstacle_distance_m", 2.5);
    base_uncertainty_m_ = declare_parameter<double>(
      "base_uncertainty_m", 0.03);
    tracking_timeout_s_ = declare_parameter<double>(
      "tracking_timeout_s", 0.5);
    velocity_smoothing_ = declare_parameter<double>(
      "velocity_smoothing", 0.35);
    max_velocity_mps_ = declare_parameter<double>(
      "max_velocity_mps", 2.0);

    validate_parameters();

    // Obstacle decisions are time-sensitive: discard queued old clouds.
    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, sensor_qos,
      std::bind(&ObstacleExtractor::cloud_callback, this, std::placeholders::_1));
    obstacle_publisher_ = create_publisher<thesis_interfaces::msg::Obstacle>(
      obstacle_topic_, rclcpp::QoS(10).reliable());
    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10).reliable());
    marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      marker_topic_, rclcpp::QoS(10).reliable());

    RCLCPP_INFO(
      get_logger(),
      "Extractor RGB-D listo: %s -> %s (salida candidata, no supervisada)",
      input_topic_.c_str(), obstacle_topic_.c_str());
  }

private:
  struct Candidate
  {
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
    float radius{0.0F};
    double score{std::numeric_limits<double>::infinity()};
    std::size_t points{0U};
  };

  void validate_parameters() const
  {
    if (cluster_tolerance_m_ <= 0.0 || min_cluster_points_ <= 0 ||
      max_cluster_points_ < min_cluster_points_ ||
      min_obstacle_radius_m_ <= 0.0 ||
      max_obstacle_radius_m_ < min_obstacle_radius_m_ ||
      max_obstacle_distance_m_ <= 0.0 || base_uncertainty_m_ < 0.0 ||
      tracking_timeout_s_ <= 0.0 || max_velocity_mps_ <= 0.0 ||
      velocity_smoothing_ < 0.0 || velocity_smoothing_ > 1.0)
    {
      throw std::runtime_error("parametros invalidos del extractor de obstaculos");
    }
  }

  Candidate candidate_for_cluster(
    const pcl::PointCloud<pcl::PointXYZRGB> & cloud,
    const pcl::PointIndices & indices) const
  {
    Candidate candidate;
    candidate.points = indices.indices.size();
    float min_x = std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    float min_z = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    float max_z = -std::numeric_limits<float>::infinity();

    for (const int index : indices.indices) {
      const auto & point = cloud.points.at(static_cast<std::size_t>(index));
      min_x = std::min(min_x, point.x);
      min_y = std::min(min_y, point.y);
      min_z = std::min(min_z, point.z);
      max_x = std::max(max_x, point.x);
      max_y = std::max(max_y, point.y);
      max_z = std::max(max_z, point.z);
    }

    candidate.x = 0.5F * (min_x + max_x);
    candidate.y = 0.5F * (min_y + max_y);
    candidate.z = 0.5F * (min_z + max_z);
    float radius = 0.0F;
    for (const int index : indices.indices) {
      const auto & point = cloud.points.at(static_cast<std::size_t>(index));
      const float dx = point.x - candidate.x;
      const float dy = point.y - candidate.y;
      const float dz = point.z - candidate.z;
      radius = std::max(radius, std::sqrt(dx * dx + dy * dy + dz * dz));
    }
    candidate.radius = std::max(
      radius, static_cast<float>(min_obstacle_radius_m_));
    const double center_distance = std::sqrt(
      static_cast<double>(candidate.x * candidate.x) +
      static_cast<double>(candidate.y * candidate.y) +
      static_cast<double>(candidate.z * candidate.z));
    candidate.score = center_distance - static_cast<double>(candidate.radius);
    return candidate;
  }

  bool select_candidate(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
    Candidate & selected,
    std::size_t & cluster_count) const
  {
    cluster_count = 0U;
    if (cloud->size() < static_cast<std::size_t>(min_cluster_points_)) {
      return false;
    }

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZRGB>());
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> extraction;
    extraction.setClusterTolerance(cluster_tolerance_m_);
    extraction.setMinClusterSize(min_cluster_points_);
    extraction.setMaxClusterSize(max_cluster_points_);
    extraction.setSearchMethod(tree);
    extraction.setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusters;
    extraction.extract(clusters);
    cluster_count = clusters.size();

    bool found = false;
    for (const auto & cluster : clusters) {
      const Candidate candidate = candidate_for_cluster(*cloud, cluster);
      const double center_distance = candidate.score + candidate.radius;
      if (candidate.radius > max_obstacle_radius_m_ ||
        center_distance > max_obstacle_distance_m_)
      {
        continue;
      }
      if (!found || candidate.score < selected.score) {
        selected = candidate;
        found = true;
      }
    }
    return found;
  }

  void estimate_velocity(
    const Candidate & candidate,
    double stamp,
    double & velocity_x,
    double & velocity_y,
    double & velocity_z)
  {
    velocity_x = 0.0;
    velocity_y = 0.0;
    velocity_z = 0.0;
    if (previous_valid_) {
      const double dt = stamp - previous_stamp_;
      if (dt > 1.0e-4 && dt <= tracking_timeout_s_) {
        double raw_x = (candidate.x - previous_x_) / dt;
        double raw_y = (candidate.y - previous_y_) / dt;
        double raw_z = (candidate.z - previous_z_) / dt;
        const double speed = std::sqrt(
          raw_x * raw_x + raw_y * raw_y + raw_z * raw_z);
        if (speed > max_velocity_mps_) {
          const double scale = max_velocity_mps_ / speed;
          raw_x *= scale;
          raw_y *= scale;
          raw_z *= scale;
        }
        velocity_x = velocity_smoothing_ * raw_x +
          (1.0 - velocity_smoothing_) * previous_velocity_x_;
        velocity_y = velocity_smoothing_ * raw_y +
          (1.0 - velocity_smoothing_) * previous_velocity_y_;
        velocity_z = velocity_smoothing_ * raw_z +
          (1.0 - velocity_smoothing_) * previous_velocity_z_;
      }
    }

    previous_valid_ = true;
    previous_stamp_ = stamp;
    previous_x_ = candidate.x;
    previous_y_ = candidate.y;
    previous_z_ = candidate.z;
    previous_velocity_x_ = velocity_x;
    previous_velocity_y_ = velocity_y;
    previous_velocity_z_ = velocity_z;
  }

  void publish_marker(
    const std_msgs::msg::Header & header,
    const Candidate * candidate)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "rgbd_obstacle_candidate";
    marker.id = 0;
    if (candidate == nullptr) {
      marker.action = visualization_msgs::msg::Marker::DELETE;
      marker_publisher_->publish(marker);
      return;
    }
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = candidate->x;
    marker.pose.position.y = candidate->y;
    marker.pose.position.z = candidate->z;
    marker.pose.orientation.w = 1.0;
    const double diameter = 2.0 *
      (static_cast<double>(candidate->radius) + base_uncertainty_m_);
    marker.scale.x = diameter;
    marker.scale.y = diameter;
    marker.scale.z = diameter;
    marker.color.r = 0.10F;
    marker.color.g = 0.78F;
    marker.color.b = 0.95F;
    marker.color.a = 0.75F;
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 350000000;
    marker_publisher_->publish(marker);
  }

  void publish_diagnostics(
    const std_msgs::msg::Header & header,
    bool has_obstacle,
    std::size_t input_points,
    std::size_t cluster_count,
    std::size_t selected_points,
    double processing_ms,
    double radius)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "RGB-D obstacle extraction";
    status.hardware_id = "perception_adapter";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = has_obstacle ?
      "READY: obstacle candidate available" :
      "READY: no obstacle candidate";
    add_value(status, "source_alive", "true");
    add_value(status, "has_obstacle", has_obstacle ? "true" : "false");
    add_value(status, "frame_id", header.frame_id);
    add_value(status, "input_points", std::to_string(input_points));
    add_value(status, "cluster_count", std::to_string(cluster_count));
    add_value(status, "selected_points", std::to_string(selected_points));
    add_value(status, "processing_ms", number(processing_ms));
    add_value(status, "radius_m", number(radius));
    add_value(status, "output_topic", obstacle_topic_);

    diagnostic_msgs::msg::DiagnosticArray report;
    report.header = header;
    report.status.push_back(status);
    diagnostics_publisher_->publish(report);
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    const auto start = std::chrono::steady_clock::now();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*msg, *cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr finite_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
    std::vector<int> finite_indices;
    pcl::removeNaNFromPointCloud(*cloud, *finite_cloud, finite_indices);

    Candidate selected;
    std::size_t cluster_count = 0U;
    const bool found = select_candidate(finite_cloud, selected, cluster_count);
    const auto finish = std::chrono::steady_clock::now();
    const double processing_ms = 1000.0 *
      std::chrono::duration<double>(finish - start).count();

    if (!found) {
      previous_valid_ = false;
      previous_velocity_x_ = 0.0;
      previous_velocity_y_ = 0.0;
      previous_velocity_z_ = 0.0;
      publish_marker(msg->header, nullptr);
      publish_diagnostics(
        msg->header, false, finite_cloud->size(), cluster_count, 0U,
        processing_ms, 0.0);
      return;
    }

    double stamp = stamp_seconds(msg->header.stamp);
    if (stamp <= 0.0) {
      stamp = this->now().seconds();
    }
    double velocity_x = 0.0;
    double velocity_y = 0.0;
    double velocity_z = 0.0;
    estimate_velocity(
      selected, stamp, velocity_x, velocity_y, velocity_z);

    thesis_interfaces::msg::Obstacle obstacle;
    obstacle.header = msg->header;
    obstacle.center.x = selected.x;
    obstacle.center.y = selected.y;
    obstacle.center.z = selected.z;
    obstacle.velocity.x = velocity_x;
    obstacle.velocity.y = velocity_y;
    obstacle.velocity.z = velocity_z;
    obstacle.radius = selected.radius;
    obstacle.uncertainty = base_uncertainty_m_;
    obstacle_publisher_->publish(obstacle);
    publish_marker(msg->header, &selected);
    publish_diagnostics(
      msg->header, true, finite_cloud->size(), cluster_count,
      selected.points, processing_ms, selected.radius);
  }

  std::string input_topic_;
  std::string obstacle_topic_;
  std::string diagnostics_topic_;
  std::string marker_topic_;
  double cluster_tolerance_m_;
  int min_cluster_points_;
  int max_cluster_points_;
  double min_obstacle_radius_m_;
  double max_obstacle_radius_m_;
  double max_obstacle_distance_m_;
  double base_uncertainty_m_;
  double tracking_timeout_s_;
  double velocity_smoothing_;
  double max_velocity_mps_;

  bool previous_valid_{false};
  double previous_stamp_{0.0};
  double previous_x_{0.0};
  double previous_y_{0.0};
  double previous_z_{0.0};
  double previous_velocity_x_{0.0};
  double previous_velocity_y_{0.0};
  double previous_velocity_z_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<thesis_interfaces::msg::Obstacle>::SharedPtr obstacle_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleExtractor>());
  rclcpp::shutdown();
  return 0;
}
