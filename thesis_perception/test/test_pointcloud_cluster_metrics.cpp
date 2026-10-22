#include "thesis_perception/pointcloud_cluster_metrics.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

#include <gtest/gtest.h>

namespace
{

TEST(PointCloudClusterMetrics, SeparatesClustersAndReportsMetrics)
{
  auto cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  cloud->push_back({0.00F, 0.00F, 0.00F});
  cloud->push_back({0.02F, 0.00F, 0.00F});
  cloud->push_back({0.00F, 0.02F, 0.00F});
  cloud->push_back({1.00F, 1.00F, 1.00F});
  cloud->push_back({1.02F, 1.00F, 1.00F});

  const auto clusters = thesis_perception::extract_cluster_metrics(cloud, 0.05, 2, 10);

  ASSERT_EQ(clusters.size(), 2U);
  EXPECT_EQ(clusters[0].points, 3U);
  EXPECT_NEAR(clusters[0].centroid_x, 0.006666667, 1e-6);
  EXPECT_NEAR(clusters[0].centroid_y, 0.006666667, 1e-6);
  EXPECT_NEAR(clusters[0].centroid_z, 0.0, 1e-6);
  EXPECT_NEAR(clusters[0].min_x, 0.0, 1e-6);
  EXPECT_NEAR(clusters[0].max_x, 0.02, 1e-6);
  EXPECT_NEAR(clusters[0].min_y, 0.0, 1e-6);
  EXPECT_NEAR(clusters[0].max_y, 0.02, 1e-6);
  EXPECT_NEAR(clusters[0].min_z, 0.0, 1e-6);
  EXPECT_NEAR(clusters[0].max_z, 0.0, 1e-6);
  EXPECT_NEAR(clusters[0].distance_to_world_origin, 0.00942809, 1e-6);

  EXPECT_EQ(clusters[1].points, 2U);
  EXPECT_NEAR(clusters[1].centroid_x, 1.01, 1e-6);
  EXPECT_NEAR(clusters[1].centroid_y, 1.0, 1e-6);
  EXPECT_NEAR(clusters[1].centroid_z, 1.0, 1e-6);
  EXPECT_NEAR(clusters[1].min_x, 1.0, 1e-6);
  EXPECT_NEAR(clusters[1].max_x, 1.02, 1e-6);
}

}  // namespace
