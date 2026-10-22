#ifndef THESIS_PERCEPTION__POINTCLOUD_CLUSTER_METRICS_HPP_
#define THESIS_PERCEPTION__POINTCLOUD_CLUSTER_METRICS_HPP_

#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace thesis_perception
{

struct ClusterMetrics
{
  std::size_t points{};
  double centroid_x{};
  double centroid_y{};
  double centroid_z{};
  double min_x{};
  double max_x{};
  double min_y{};
  double max_y{};
  double min_z{};
  double max_z{};
  double distance_to_world_origin{};
};

inline ClusterMetrics cluster_metrics(
  const pcl::PointCloud<pcl::PointXYZRGB> & cloud,
  const pcl::PointIndices & indices)
{
  if (indices.indices.empty()) {
    throw std::invalid_argument("cluster indices must not be empty");
  }

  ClusterMetrics result;
  result.points = indices.indices.size();
  result.min_x = result.min_y = result.min_z = std::numeric_limits<double>::infinity();
  result.max_x = result.max_y = result.max_z = -std::numeric_limits<double>::infinity();

  for (const int index : indices.indices) {
    const auto & point = cloud.points.at(static_cast<std::size_t>(index));
    const double x = point.x;
    const double y = point.y;
    const double z = point.z;
    result.centroid_x += x;
    result.centroid_y += y;
    result.centroid_z += z;
    result.min_x = std::min(result.min_x, x);
    result.max_x = std::max(result.max_x, x);
    result.min_y = std::min(result.min_y, y);
    result.max_y = std::max(result.max_y, y);
    result.min_z = std::min(result.min_z, z);
    result.max_z = std::max(result.max_z, z);
  }

  const double count = static_cast<double>(result.points);
  result.centroid_x /= count;
  result.centroid_y /= count;
  result.centroid_z /= count;
  result.distance_to_world_origin = std::sqrt(
    result.centroid_x * result.centroid_x + result.centroid_y * result.centroid_y +
    result.centroid_z * result.centroid_z);
  return result;
}

inline std::vector<ClusterMetrics> extract_cluster_metrics(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & input,
  const double tolerance_m,
  const int min_points,
  const int max_points)
{
  if (!input || tolerance_m <= 0.0 || min_points <= 0 || max_points < min_points) {
    throw std::invalid_argument("invalid cluster extraction configuration");
  }

  auto finite = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
    new pcl::PointCloud<pcl::PointXYZRGB>());
  std::vector<int> finite_indices;
  pcl::removeNaNFromPointCloud(*input, *finite, finite_indices);
  if (finite->empty()) {
    return {};
  }

  auto tree = pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(
    new pcl::search::KdTree<pcl::PointXYZRGB>());
  tree->setInputCloud(finite);
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> extraction;
  extraction.setClusterTolerance(static_cast<float>(tolerance_m));
  extraction.setMinClusterSize(min_points);
  extraction.setMaxClusterSize(max_points);
  extraction.setSearchMethod(tree);
  extraction.setInputCloud(finite);

  std::vector<pcl::PointIndices> clusters;
  extraction.extract(clusters);
  std::vector<ClusterMetrics> result;
  result.reserve(clusters.size());
  for (const auto & cluster : clusters) {
    result.push_back(cluster_metrics(*finite, cluster));
  }
  std::sort(result.begin(), result.end(), [](const auto & first, const auto & second) {
    return first.points > second.points;
  });
  return result;
}

}  // namespace thesis_perception

#endif  // THESIS_PERCEPTION__POINTCLOUD_CLUSTER_METRICS_HPP_
