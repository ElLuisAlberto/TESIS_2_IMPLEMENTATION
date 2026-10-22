#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace {
using Image = sensor_msgs::msg::Image;
using Info = sensor_msgs::msg::CameraInfo;
using Cloud = sensor_msgs::msg::PointCloud2;
using Transform = geometry_msgs::msg::TransformStamped;
constexpr std::int64_t kMaxDeltaNs = 33000000;

template<class T> std::int64_t stamp_ns(const T & message)
{
  return static_cast<std::int64_t>(message.header.stamp.sec) * 1000000000LL +
    message.header.stamp.nanosec;
}

template<class T> typename T::ConstSharedPtr nearest(
  const Cloud & cloud, const std::deque<typename T::ConstSharedPtr> & messages)
{
  return *std::min_element(messages.begin(), messages.end(), [&](const auto & a, const auto & b) {
    return std::llabs(stamp_ns(*a) - stamp_ns(cloud)) <
      std::llabs(stamp_ns(*b) - stamp_ns(cloud));
  });
}

template<class T> void write_message(const T & message)
{
  rclcpp::Serialization<T> serializer;
  rclcpp::SerializedMessage serial;
  serializer.serialize_message(&message, &serial);
  const auto & data = serial.get_rcl_serialized_message();
  const auto length = static_cast<std::uint32_t>(data.buffer_length);
  std::cout.write(reinterpret_cast<const char *>(&length), sizeof(length));
  std::cout.write(reinterpret_cast<const char *>(data.buffer), length);
}

struct Capture {
  Image::ConstSharedPtr rgb;
  Info::ConstSharedPtr info;
  Cloud::ConstSharedPtr cloud;
  Transform depth_tf;
  Transform color_tf;
  Transform world_color_tf;
  Transform world_camera_link_tf;
  Transform camera_link_color_tf;
  Transform color_camera_link_tf;
  Transform camera_link_world_tf;
  std::vector<Transform> link_tfs;
};

class Snapshot : public rclcpp::Node {
public:
  explicit Snapshot(std::vector<std::string> collision_links, bool tf_complete)
  : Node("mesh_arm_snapshot_once", rclcpp::NodeOptions().enable_rosout(false)
      .start_parameter_services(false).start_parameter_event_publisher(false)),
    tf_(get_clock()), listener_(tf_, this, false), collision_links_(std::move(collision_links)),
    tf_complete_(tf_complete)
  {
    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    rgb_sub_ = create_subscription<Image>("/camera/d435i/color/image_raw", qos,
      [this](Image::ConstSharedPtr value) {push<Image>(rgb_, value);});
    info_sub_ = create_subscription<Info>("/camera/d435i/color/camera_info", qos,
      [this](Info::ConstSharedPtr value) {push<Info>(info_, value);});
    cloud_sub_ = create_subscription<Cloud>("/thesis/perception/points_filtered", qos,
      [this](Cloud::ConstSharedPtr value) {push<Cloud>(cloud_, value);});
  }

  std::optional<Capture> try_match()
  {
    if (rgb_.empty() || info_.empty() || cloud_.empty()) return {};
    for (auto it = cloud_.rbegin(); it != cloud_.rend(); ++it) {
      auto rgb = nearest<Image>(**it, rgb_);
      auto info = nearest<Info>(**it, info_);
      auto rgb_delta = std::llabs(stamp_ns(*rgb) - stamp_ns(**it));
      auto info_delta = std::llabs(stamp_ns(*info) - stamp_ns(**it));
      best_delta_ns_ = std::min<std::int64_t>(best_delta_ns_, std::max(rgb_delta, info_delta));
      if (rgb_delta > kMaxDeltaNs || info_delta > kMaxDeltaNs ||
          (*it)->header.frame_id != "world" ||
          rgb->header.frame_id != info->header.frame_id) continue;
      try {
        auto stamp = rclcpp::Time((*it)->header.stamp);
        auto timeout = rclcpp::Duration::from_seconds(0.05);
        auto depth = tf_.lookupTransform("world", "d435i_depth_optical_frame", stamp, timeout);
        auto color = tf_.lookupTransform(rgb->header.frame_id, "world", stamp, timeout);
        Transform world_color;
        Transform world_camera_link;
        Transform camera_link_color;
        Transform color_camera_link;
        Transform camera_link_world;
        std::vector<Transform> link_tfs;
        if (!collision_links_.empty()) {
          world_color = tf_.lookupTransform("world", rgb->header.frame_id, stamp, timeout);
          if (tf_complete_) {
            world_camera_link = tf_.lookupTransform("world", "d435i_link", stamp, timeout);
            camera_link_color = tf_.lookupTransform("d435i_link", rgb->header.frame_id,
              stamp, timeout);
            color_camera_link = tf_.lookupTransform(rgb->header.frame_id, "d435i_link",
              stamp, timeout);
            camera_link_world = tf_.lookupTransform("d435i_link", "world", stamp, timeout);
          }
          for (const auto & link : collision_links_) {
            link_tfs.push_back(tf_.lookupTransform("world", link, stamp, timeout));
          }
        }
        return Capture{rgb, info, *it, depth, color, world_color, world_camera_link,
          camera_link_color, color_camera_link, camera_link_world, link_tfs};
      } catch (const std::exception &) {
        continue;
      }
    }
    return {};
  }

  std::int64_t best_delta_ns() const {return best_delta_ns_;}

private:
  template<class T> static void push(
    std::deque<typename T::ConstSharedPtr> & messages, typename T::ConstSharedPtr value)
  {
    messages.push_back(value);
    if (messages.size() > 12) messages.pop_front();
  }

  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  std::vector<std::string> collision_links_;
  bool tf_complete_;
  std::deque<Image::ConstSharedPtr> rgb_;
  std::deque<Info::ConstSharedPtr> info_;
  std::deque<Cloud::ConstSharedPtr> cloud_;
  std::int64_t best_delta_ns_{INT64_MAX};
  rclcpp::Subscription<Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<Info>::SharedPtr info_sub_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
};
}

int main(int argc, char ** argv)
{
  std::vector<std::string> collision_links;
  bool tf_complete = argc > 1 && std::string(argv[1]) == "--tf-complete";
  for (int i = tf_complete ? 2 : 1; i < argc; ++i) collision_links.emplace_back(argv[i]);
  if (tf_complete && collision_links.empty()) {
    std::cerr << "TF-complete capture requires collision links\n";
    return 2;
  }
  int ros_argc = 1;
  rclcpp::init(ros_argc, argv);
  auto node = std::make_shared<Snapshot>(collision_links, tf_complete);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(20));
    auto capture = node->try_match();
    if (!capture) continue;
    std::cout.write(tf_complete ? "MESH8T1" :
      (collision_links.empty() ? "MESH8P1" : "MESH8R1"), 7);
    write_message(*capture->rgb);
    write_message(*capture->info);
    write_message(*capture->cloud);
    write_message(capture->depth_tf);
    write_message(capture->color_tf);
    if (!collision_links.empty()) {
      write_message(capture->world_color_tf);
      if (tf_complete) {
        write_message(capture->world_camera_link_tf);
        write_message(capture->camera_link_color_tf);
        write_message(capture->color_camera_link_tf);
        write_message(capture->camera_link_world_tf);
      }
      for (const auto & tf : capture->link_tfs) write_message(tf);
    }
    std::cout.flush();
    executor.remove_node(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
  }
  std::cerr << "BLOCKED_SYNC_UNAVAILABLE best_max_delta_ns=" << node->best_delta_ns() << '\n';
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 3;
}
