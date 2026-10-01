#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class PointCloudPreprocessor : public rclcpp::Node
{
public:
  PointCloudPreprocessor()
  : Node("pointcloud_preprocessor"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/camera/d435i/depth/color/points");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/thesis/perception/points_filtered");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    use_latest_transform_ = declare_parameter<bool>("use_latest_transform", true);
    restamp_output_ = declare_parameter<bool>("restamp_output", false);
    enable_crop_ = declare_parameter<bool>("enable_crop", true);
    enable_voxel_ = declare_parameter<bool>("enable_voxel", true);
    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.02);
    min_x_ = declare_parameter<double>("min_x", -1.5);
    max_x_ = declare_parameter<double>("max_x", 1.5);
    min_y_ = declare_parameter<double>("min_y", -1.5);
    max_y_ = declare_parameter<double>("max_y", 1.5);
    min_z_ = declare_parameter<double>("min_z", 0.0);
    max_z_ = declare_parameter<double>("max_z", 2.5);

    if (voxel_leaf_size_ <= 0.0) {
      throw std::runtime_error("voxel_leaf_size debe ser mayor que cero");
    }

    auto qos = rclcpp::SensorDataQoS().keep_last(5);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&PointCloudPreprocessor::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Entrada: %s | Salida: %s | Frame objetivo: %s",
      input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str());
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 transformed;

    try {
      if (!target_frame_.empty() && msg->header.frame_id != target_frame_) {
        const rclcpp::Time lookup_time = use_latest_transform_ ?
          rclcpp::Time(0, 0, RCL_ROS_TIME) : rclcpp::Time(msg->header.stamp);
        const auto transform = tf_buffer_.lookupTransform(
          target_frame_, msg->header.frame_id, lookup_time,
          rclcpp::Duration::from_seconds(0.10));
        tf2::doTransform(*msg, transformed, transform);
      } else {
        transformed = *msg;
      }
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No se puede transformar %s -> %s: %s",
        msg->header.frame_id.c_str(), target_frame_.c_str(), error.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(transformed, *cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped = cloud;
    if (enable_crop_) {
      cropped.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::CropBox<pcl::PointXYZRGB> crop;
      crop.setInputCloud(cloud);
      crop.setMin(Eigen::Vector4f(min_x_, min_y_, min_z_, 1.0F));
      crop.setMax(Eigen::Vector4f(max_x_, max_y_, max_z_, 1.0F));
      crop.filter(*cropped);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered = cropped;
    if (enable_voxel_) {
      filtered.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
      voxel.setInputCloud(cropped);
      const auto leaf = static_cast<float>(voxel_leaf_size_);
      voxel.setLeafSize(leaf, leaf, leaf);
      voxel.filter(*filtered);
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*filtered, output);
    output.header.frame_id = target_frame_.empty() ? transformed.header.frame_id : target_frame_;
    output.header.stamp = restamp_output_ ? this->now().to_msg() : transformed.header.stamp;
    publisher_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  bool use_latest_transform_;
  bool restamp_output_;
  bool enable_crop_;
  bool enable_voxel_;
  double voxel_leaf_size_;
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
  double min_z_;
  double max_z_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudPreprocessor>());
  rclcpp::shutdown();
  return 0;
}
