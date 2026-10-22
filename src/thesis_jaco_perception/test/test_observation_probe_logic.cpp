#include "thesis_jaco_perception/observation_probe_logic.hpp"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace probe = thesis_jaco_perception;

namespace
{
sensor_msgs::msg::PointCloud2 synthetic_cloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "observed_camera_frame";
  cloud.header.stamp.sec = 10;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(3U);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  *x = 0.05F; *y = 0.0F; *z = 0.0F; ++x; ++y; ++z;
  *x = 0.50F; *y = 0.0F; *z = 0.0F; ++x; ++y; ++z;
  *x = 2.00F; *y = 0.0F; *z = 0.0F;
  return cloud;
}

TEST(ObservationProbeLogic, CountsSyntheticRangeBucketsAndMetadata)
{
  probe::ObservationMetrics metrics;
  probe::observe_pointcloud(synthetic_cloud(), 10.1, 1.5, metrics);
  EXPECT_EQ(metrics.pointcloud_frame_id, "observed_camera_frame");
  EXPECT_EQ(metrics.clouds_received, 1U);
  EXPECT_EQ(metrics.finite_points, 3U);
  EXPECT_EQ(metrics.below_min_range_points, 1U);
  EXPECT_EQ(metrics.in_range_points, 1U);
  EXPECT_EQ(metrics.above_max_range_points, 1U);
  EXPECT_NEAR(metrics.last_stamp_age_sec, 0.1, 1.0e-12);
  EXPECT_NEAR(probe::point_proportion(metrics.in_range_points, metrics.finite_points),
    1.0 / 3.0, 1.0e-12);
  EXPECT_EQ(metrics.width, 3U);
  EXPECT_EQ(metrics.height, 1U);
  EXPECT_FALSE(metrics.fields.empty());
}

TEST(ObservationProbeLogic, CountsNullFutureAndStaleStamps)
{
  probe::ObservationMetrics metrics;
  auto cloud = synthetic_cloud();
  cloud.header.stamp.sec = 0;
  probe::observe_pointcloud(cloud, 10.0, 1.5, metrics);
  cloud.header.stamp.sec = 11;
  probe::observe_pointcloud(cloud, 10.0, 1.5, metrics);
  cloud.header.stamp.sec = 1;
  probe::observe_pointcloud(cloud, 10.0, 1.5, metrics);
  EXPECT_EQ(metrics.null_stamps, 1U);
  EXPECT_EQ(metrics.future_stamps, 1U);
  EXPECT_EQ(metrics.stale_stamps, 1U);
}

TEST(ObservationProbeLogic, RejectsInvalidParameters)
{
  EXPECT_THROW(probe::validate_probe_parameters(0.0, 1.5), std::invalid_argument);
  EXPECT_THROW(probe::validate_probe_parameters(30.0, 0.10), std::invalid_argument);
  EXPECT_THROW(probe::validate_probe_parameters(
    std::numeric_limits<double>::infinity(), 1.5), std::invalid_argument);
}

TEST(ObservationProbeLogic, EmptyReportPathDoesNotWriteAFile)
{
  probe::ObservationMetrics metrics;
  EXPECT_FALSE(probe::write_json_report_if_requested("", metrics));
}

TEST(ObservationProbeLogic, WritesJsonOnlyToRequestedTemporaryPath)
{
  probe::ObservationMetrics metrics;
  probe::observe_pointcloud(synthetic_cloud(), 10.0, 1.5, metrics);
  const auto path = std::filesystem::temp_directory_path() /
    "thesis_jaco_perception_observation_probe_test.json";
  std::filesystem::remove(path);
  ASSERT_TRUE(probe::write_json_report_if_requested(path.string(), metrics));
  ASSERT_TRUE(std::filesystem::exists(path));
  std::ifstream report(path);
  const std::string content((std::istreambuf_iterator<char>(report)),
    std::istreambuf_iterator<char>());
  EXPECT_NE(content.find("\"pointcloud\""), std::string::npos);
  EXPECT_NE(content.find("observed_camera_frame"), std::string::npos);
  EXPECT_NE(content.find("\"ranges\""), std::string::npos);
  std::filesystem::remove(path);
}

TEST(ObservationProbeLogic, StoresOptionalCameraInfoWithoutAssumingIt)
{
  probe::ObservationMetrics metrics;
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.header.frame_id = "camera_info_frame";
  camera_info.width = 640U;
  camera_info.height = 480U;
  camera_info.distortion_model = "observed_model";
  probe::observe_camera_info(camera_info, metrics);
  EXPECT_TRUE(metrics.camera_info.received);
  EXPECT_EQ(metrics.camera_info.frame_id, "camera_info_frame");
  EXPECT_EQ(metrics.camera_info.width, 640U);
  EXPECT_EQ(metrics.camera_info.height, 480U);
}
}  // namespace
