#pragma once                       // Only include once per compile
#ifndef POINTCLOUD_CONCATENATE_HPP // Conditional compiling
#define POINTCLOUD_CONCATENATE_HPP

// Includes
#include <array>
#include <mutex>

#include <rclcpp/rclcpp.hpp> // ROS header

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

// Macro to warn about unset parameters
#define PARAM_WARN(param_name, default_val)                                 \
  std::cout << "\033[33m"                                                   \
            << "[WARN] Param is not set: " << param_name                    \
            << ". Setting to default value: " << default_val << "\033[0m\n" \
            << std::endl

// Define class
class PointcloudConcatenate : public rclcpp::Node
{
public:
  // Constructor and destructor
  PointcloudConcatenate();
  ~PointcloudConcatenate();

  // Public functions
  void handleParams();
  void update();
  double getHz();

  // Public variables and objects

private:
  // Parameters
  std::string param_frame_target_;
  int param_clouds_;
  double param_hz_;
  std::string cloud_in1_topic_;
  std::string cloud_in2_topic_;
  std::string cloud_in3_topic_;
  std::string cloud_in4_topic_;
  std::string cloud_out_topic_;
  double transform_timeout_sec_;
  bool fallback_to_latest_transform_;
  // A source whose newest cloud arrived longer ago than this (wall seconds) is
  // treated as stale and dropped from the merge, so a dead sensor falls out
  // while normal inter-sensor jitter does not. <= 0 disables the check.
  double max_cloud_age_;

  // Publisher and subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in1_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in2_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in3_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in4_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_out_;

  // Timer and callback groups: the subscriptions run in a reentrant group and
  // the (potentially TF-blocking) update timer in its own group, so under a
  // MultiThreadedExecutor a slow update() cannot starve incoming clouds.
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

  // Private functions
  void subCallbackCloudIn1(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void subCallbackCloudIn2(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void subCallbackCloudIn3(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void subCallbackCloudIn4(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publishPointcloud(sensor_msgs::msg::PointCloud2 &cloud);
  // Rewrites an arbitrary input cloud into the canonical XYZI layout
  // (point_step = 16) so clouds from different drivers (Ouster's wide
  // ouster_ros::Point vs sick_scan_xd's XYZI) become byte-compatible and can
  // actually be concatenated. Missing intensity is filled with 0.
  void normalizeToXYZI(const sensor_msgs::msg::PointCloud2 &cloud_in, sensor_msgs::msg::PointCloud2 &cloud_out);
  void concatenateFlexible(const sensor_msgs::msg::PointCloud2 &cloud_a, const sensor_msgs::msg::PointCloud2 &cloud_b, sensor_msgs::msg::PointCloud2 &cloud_out);
  bool transformCloudToTarget(
    const sensor_msgs::msg::PointCloud2 & cloud_in,
    sensor_msgs::msg::PointCloud2 & cloud_out,
    const char * cloud_name);

  // Other

  // Latest-sample buffers: the callbacks swap in the newest cloud per source
  // (cheap shared-ptr assignment) and update() snapshots them under the mutex.
  // We keep the *latest* cloud from every source — never "consume once" — so
  // every published frame contains all enabled sensors regardless of which one
  // happened to arrive most recently.
  std::mutex cloud_mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_in1_ptr_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_in2_ptr_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_in3_ptr_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_in4_ptr_;
  rclcpp::Time cloud_in1_arrival_;
  rclcpp::Time cloud_in2_arrival_;
  rclcpp::Time cloud_in3_arrival_;
  rclcpp::Time cloud_in4_arrival_;

  sensor_msgs::msg::PointCloud2 cloud_out_;

  // Initialization tf2 listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif // POINTCLOUD_CONCATENATE_HPP
