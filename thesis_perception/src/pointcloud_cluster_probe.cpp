#include "thesis_perception/pointcloud_cluster_metrics.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace
{

enum class ProbeResult
{
  kPending,
  kSuccess,
  kFrameError,
  kNoCloud,
  kProcessingError,
};

class PointCloudClusterProbe : public rclcpp::Node
{
public:
  explicit PointCloudClusterProbe(const rclcpp::NodeOptions & options)
  : Node("pointcloud_cluster_probe", options)
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/thesis/jaco_perception/points_external_candidate");
    cluster_tolerance_m_ = declare_parameter<double>("cluster_tolerance_m", 0.04);
    min_cluster_points_ = declare_parameter<int>("min_cluster_points", 20);
    max_cluster_points_ = declare_parameter<int>("max_cluster_points", 200000);
    timeout_s_ = declare_parameter<double>("timeout_s", 5.0);
    if (input_topic_.empty() || cluster_tolerance_m_ <= 0.0 || min_cluster_points_ <= 0 ||
      max_cluster_points_ < min_cluster_points_ || timeout_s_ <= 0.0 || timeout_s_ > 5.0)
    {
      throw std::invalid_argument("invalid read-only cluster probe parameters");
    }

    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&PointCloudClusterProbe::cloud_callback, this, std::placeholders::_1));
    timeout_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timeout_s_)),
      std::bind(&PointCloudClusterProbe::timeout_callback, this));
    RCLCPP_INFO(
      get_logger(),
      "READ_ONLY_CLUSTER_PROBE input=%s tolerance_m=%.3f min_points=%d max_points=%d timeout_s=%.1f",
      input_topic_.c_str(), cluster_tolerance_m_, min_cluster_points_, max_cluster_points_, timeout_s_);
  }

  ProbeResult result() const
  {
    return result_;
  }

private:
  void finish(const ProbeResult result)
  {
    if (result_ != ProbeResult::kPending) {
      return;
    }
    result_ = result;
    subscription_.reset();
    timeout_timer_->cancel();
    rclcpp::shutdown();
  }

  void timeout_callback()
  {
    RCLCPP_ERROR(get_logger(), "PROBE_NO_CLOUD timeout_s=%.1f", timeout_s_);
    finish(ProbeResult::kNoCloud);
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    if (result_ != ProbeResult::kPending) {
      return;
    }
    if (message->header.frame_id != "world") {
      RCLCPP_ERROR(
        get_logger(), "PROBE_FRAME_ERROR expected=world received=%s",
        message->header.frame_id.c_str());
      finish(ProbeResult::kFrameError);
      return;
    }

    try {
      auto cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
        new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::fromROSMsg(*message, *cloud);
      const auto metrics = thesis_perception::extract_cluster_metrics(
        cloud, cluster_tolerance_m_, min_cluster_points_, max_cluster_points_);
      RCLCPP_INFO(
        get_logger(), "PROBE_FRAME_OK frame=world cloud_points=%zu clusters=%zu",
        cloud->points.size(), metrics.size());
      for (std::size_t index = 0; index < metrics.size(); ++index) {
        const auto & cluster = metrics[index];
        RCLCPP_INFO(
          get_logger(),
          "CLUSTER rank=%zu points=%zu centroid=(%.6f,%.6f,%.6f) "
          "bounds_x=(%.6f,%.6f) bounds_y=(%.6f,%.6f) bounds_z=(%.6f,%.6f) "
          "distance_world_origin=%.6f",
          index + 1U, cluster.points, cluster.centroid_x, cluster.centroid_y, cluster.centroid_z,
          cluster.min_x, cluster.max_x, cluster.min_y, cluster.max_y,
          cluster.min_z, cluster.max_z, cluster.distance_to_world_origin);
      }
      finish(ProbeResult::kSuccess);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "PROBE_PROCESSING_ERROR %s", error.what());
      finish(ProbeResult::kProcessingError);
    }
  }

  std::string input_topic_;
  double cluster_tolerance_m_{};
  int min_cluster_points_{};
  int max_cluster_points_{};
  double timeout_s_{};
  ProbeResult result_{ProbeResult::kPending};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

int exit_code(const ProbeResult result)
{
  switch (result) {
    case ProbeResult::kSuccess:
      return 0;
    case ProbeResult::kFrameError:
      return 2;
    case ProbeResult::kNoCloud:
      return 3;
    case ProbeResult::kProcessingError:
      return 4;
    case ProbeResult::kPending:
      return 5;
  }
  return 5;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options
    .enable_rosout(false)
    .start_parameter_services(false)
    .start_parameter_event_publisher(false);
  auto probe = std::make_shared<PointCloudClusterProbe>(options);
  rclcpp::spin(probe);
  return exit_code(probe->result());
}
