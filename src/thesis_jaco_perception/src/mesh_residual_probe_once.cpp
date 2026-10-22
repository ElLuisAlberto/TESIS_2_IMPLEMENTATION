#include "thesis_jaco_perception/self_mesh_geometry.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using thesis_jaco_perception::MeshLinkTransform;
using thesis_jaco_perception::MeshPoint3;

static std::array<double, 16> matrix(const geometry_msgs::msg::TransformStamped & t)
{
  const auto & q = t.transform.rotation;
  const double x = q.x, y = q.y, z = q.z, w = q.w;
  return {1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w, t.transform.translation.x,
    2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w, t.transform.translation.y,
    2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y, t.transform.translation.z, 0, 0, 0, 1};
}

static double percentile(std::vector<double> values, double fraction)
{
  std::sort(values.begin(), values.end());
  const double pos = fraction * (values.size() - 1);
  const auto lo = static_cast<std::size_t>(std::floor(pos));
  const auto hi = static_cast<std::size_t>(std::ceil(pos));
  return values[lo] + (pos - lo) * (values[hi] - values[lo]);
}

class Probe : public rclcpp::Node
{
public:
  explicit Probe(const std::string & xml)
  : Node("mesh_residual_probe_once", rclcpp::NodeOptions().enable_rosout(false)
      .start_parameter_services(false).start_parameter_event_publisher(false)),
    tf_(get_clock()), listener_(tf_), meshes_(thesis_jaco_perception::load_collision_meshes_from_urdf_xml(xml)),
    deadline_(std::chrono::steady_clock::now() + std::chrono::seconds(5))
  {
    std::cout << "MESHES " << meshes_.size() << std::endl;
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/thesis/perception/points_filtered", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {receive(*cloud);});
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
      if (std::chrono::steady_clock::now() >= deadline_) finish("TIMEOUT");
    });
  }

private:
  void receive(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    if (done_) return;
    if (cloud.header.frame_id != "world") {finish("FRAME_NOT_WORLD"); return;}
    try {
      const rclcpp::Time stamp(cloud.header.stamp);
      const auto timeout = rclcpp::Duration::from_seconds(0.05);
      auto camera = tf_.lookupTransform("world", "d435i_depth_optical_frame", stamp, timeout);
      std::vector<MeshLinkTransform> transforms;
      for (const auto & mesh : meshes_) {
        transforms.push_back({mesh.link_frame, matrix(tf_.lookupTransform(
          "world", mesh.link_frame, stamp, timeout))});
      }
      auto world = thesis_jaco_perception::update_world_mesh_instances(meshes_, transforms);
      const MeshPoint3 origin{camera.transform.translation.x, camera.transform.translation.y,
        camera.transform.translation.z};
      const std::size_t total = static_cast<std::size_t>(cloud.width) * cloud.height;
      const std::size_t stride = std::max<std::size_t>(1, (total + 4999) / 5000);
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
      std::size_t sampled = 0, hits = 0, no_hit = 0;
      for (std::size_t i = 0; i < total; ++i, ++x, ++y, ++z) {
        if (i % stride || std::chrono::steady_clock::now() >= deadline_) continue;
        MeshPoint3 point{*x, *y, *z};
        if (!thesis_jaco_perception::is_finite(point)) continue;
        ++sampled;
        auto hit = thesis_jaco_perception::first_world_mesh_intersection(
          meshes_, transforms, origin, point);
        if (!hit) {++no_hit; continue;}
        ++hits;
        residuals_[{hit->link_frame, hit->uri}].push_back(hit->delta_ray);
      }
      std::cout << "CLOUD stamp=" << cloud.header.stamp.sec << '.' << cloud.header.stamp.nanosec
        << " frame=" << cloud.header.frame_id << " tf_stamp=" << cloud.header.stamp.sec << '.'
        << cloud.header.stamp.nanosec << " total=" << total << " stride=" << stride
        << " sampled=" << sampled << " hits=" << hits << " no_hit=" << no_hit << std::endl;
      ++clouds_;
      if (clouds_ >= 3 || std::chrono::steady_clock::now() >= deadline_) finish("COMPLETE");
    } catch (const std::exception & e) {
      std::cout << "DISCARDED stamp=" << cloud.header.stamp.sec << '.'
        << cloud.header.stamp.nanosec << " reason=" << e.what() << std::endl;
    }
  }

  void finish(const char * status)
  {
    if (done_) return;
    done_ = true;
    std::cout << "RESULT " << status << " clouds=" << clouds_ << std::endl;
    for (const auto & entry : residuals_) {
      const auto & v = entry.second;
      const auto neg = std::count_if(v.begin(), v.end(), [](double d) {return d < 0;});
      const auto pos = std::count_if(v.begin(), v.end(), [](double d) {return d > 0;});
      std::cout << "LINK " << entry.first.first << " uri=" << entry.first.second
        << " count=" << v.size() << " negative=" << neg << " positive=" << pos
        << " p50=" << percentile(v, .50) << " p90=" << percentile(v, .90)
        << " p95=" << percentile(v, .95) << " p99=" << percentile(v, .99) << std::endl;
    }
    rclcpp::shutdown();
  }

  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  std::vector<thesis_jaco_perception::MeshInstance> meshes_;
  std::chrono::steady_clock::time_point deadline_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<std::pair<std::string, std::string>, std::vector<double>> residuals_;
  std::size_t clouds_{};
  bool done_{};
};

int main(int argc, char ** argv)
{
  if (argc != 2) return 2;
  std::ifstream input(argv[1]);
  if (!input) return 3;
  std::ostringstream xml;
  xml << input.rdbuf();
  rclcpp::init(argc, argv);
  auto probe = std::make_shared<Probe>(xml.str());
  rclcpp::spin(probe);
}
