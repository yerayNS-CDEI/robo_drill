// Copyright 2026 robo_drill

#include "robo_drill/back_out_from_obstacle.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "pluginlib/class_list_macros.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#pragma GCC diagnostic pop

namespace robo_drill
{

BackOutFromObstacle::BackOutFromObstacle()
: TimedBehavior<BackOutFromObstacleAction>(),
  feedback_(std::make_shared<BackOutFromObstacleAction::Feedback>()),
  max_distance_(0.5),
  command_speed_(0.1),
  costmap_frame_("odom"),
  search_radius_(2.0),
  obstacle_cost_min_(128),
  clear_cost_threshold_(50.0),
  forward_check_dist_(0.15),
  planner_backend_("moveit"),
  cloud_topic_("/combined_cloud_filtered"),
  cloud_min_z_(0.1),
  cloud_max_z_(2.0)
{
}

void BackOutFromObstacle::onConfigure()
{
  auto node = this->node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // The local costmap is rolling in its own frame (e.g. odom). The collision
  // checker scores poses in that frame, so robot poses must be looked up there
  // rather than in the behavior server's global_frame (map).
  nav2_util::declare_parameter_if_not_declared(
    node, this->behavior_name_ + ".costmap_frame", rclcpp::ParameterValue("odom"));
  node->get_parameter(this->behavior_name_ + ".costmap_frame", costmap_frame_);

  nav2_util::declare_parameter_if_not_declared(
    node, this->behavior_name_ + ".search_radius", rclcpp::ParameterValue(2.0));
  node->get_parameter(this->behavior_name_ + ".search_radius", search_radius_);

  nav2_util::declare_parameter_if_not_declared(
    node, this->behavior_name_ + ".forward_check_dist", rclcpp::ParameterValue(0.15));
  node->get_parameter(this->behavior_name_ + ".forward_check_dist", forward_check_dist_);

  nav2_util::declare_parameter_if_not_declared(
    node, this->behavior_name_ + ".obstacle_cost_min", rclcpp::ParameterValue(128));
  node->get_parameter(this->behavior_name_ + ".obstacle_cost_min", obstacle_cost_min_);

  nav2_util::declare_parameter_if_not_declared(
    node, this->behavior_name_ + ".clear_cost_threshold", rclcpp::ParameterValue(50.0));
  node->get_parameter(this->behavior_name_ + ".clear_cost_threshold", clear_cost_threshold_);

  nav2_util::declare_parameter_if_not_declared(
    node, this->behavior_name_ + ".planner_backend", rclcpp::ParameterValue("moveit"));
  node->get_parameter(this->behavior_name_ + ".planner_backend", planner_backend_);
  nav2_util::declare_parameter_if_not_declared(
    node, this->behavior_name_ + ".cloud_topic", rclcpp::ParameterValue(cloud_topic_));
  node->get_parameter(this->behavior_name_ + ".cloud_topic", cloud_topic_);
  nav2_util::declare_parameter_if_not_declared(
    node, this->behavior_name_ + ".cloud_min_z", rclcpp::ParameterValue(0.1));
  node->get_parameter(this->behavior_name_ + ".cloud_min_z", cloud_min_z_);
  nav2_util::declare_parameter_if_not_declared(
    node, this->behavior_name_ + ".cloud_max_z", rclcpp::ParameterValue(2.0));
  node->get_parameter(this->behavior_name_ + ".cloud_max_z", cloud_max_z_);

  // Subscribe to the raw local costmap (RELIABLE + TRANSIENT_LOCAL to match the
  // costmap publisher). Used for the repulsive escape-direction computation.
  std::string costmap_topic = "local_costmap/costmap_raw";
  node->get_parameter("costmap_topic", costmap_topic);
  rclcpp::QoS costmap_qos(rclcpp::KeepLast(1));
  costmap_qos.reliable().transient_local();
  costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
    costmap_topic, costmap_qos,
    std::bind(&BackOutFromObstacle::onCostmap, this, std::placeholders::_1));

  // Filtered pointcloud (best-effort sensor stream). Preferred escape-direction
  // source when the MoveIt backend is in use.
  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&BackOutFromObstacle::onCloud, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->logger_,
    "BackOutFromObstacle configured (costmap_topic=%s, cloud_topic=%s, backend=%s, frame=%s).",
    costmap_topic.c_str(), cloud_topic_.c_str(), planner_backend_.c_str(), costmap_frame_.c_str());
}

void BackOutFromObstacle::onCostmap(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  costmap_msg_ = msg;
}

void BackOutFromObstacle::onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  cloud_msg_ = msg;
}

bool BackOutFromObstacle::computeEscapeHeadingFromCloud(
  double rx, double ry, double & out_heading)
{
  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud = cloud_msg_;
  }
  if (!cloud || (cloud->width * cloud->height) == 0) {
    return false;
  }

  // Bring the cloud into the costmap frame (where the robot pose lives).
  tf2::Transform cloud_to_map;
  try {
    const auto tf_msg = this->tf_->lookupTransform(
      costmap_frame_, cloud->header.frame_id, tf2::TimePointZero);
    tf2::fromMsg(tf_msg.transform, cloud_to_map);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->logger_, "BackOutFromObstacle: TF %s->%s failed (%s); using costmap direction.",
      costmap_frame_.c_str(), cloud->header.frame_id.c_str(), ex.what());
    return false;
  }

  // Cost-weighted repulsive vector: unit vectors from each in-band obstacle
  // point toward the robot. The resultant points away from the densest side,
  // i.e. toward the clear side to strafe to.
  double sum_x = 0.0;
  double sum_y = 0.0;
  size_t n = 0;
  sensor_msgs::PointCloud2ConstIterator<float> it_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(*cloud, "z");
  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    if (!std::isfinite(*it_x) || !std::isfinite(*it_y) || !std::isfinite(*it_z)) {continue;}
    const tf2::Vector3 p = cloud_to_map * tf2::Vector3(*it_x, *it_y, *it_z);
    if (p.z() < cloud_min_z_ || p.z() > cloud_max_z_) {continue;}
    const double vx = rx - p.x();
    const double vy = ry - p.y();
    const double d = std::hypot(vx, vy);
    if (d < 1e-3 || d > search_radius_) {continue;}
    sum_x += vx / d;
    sum_y += vy / d;
    ++n;
  }

  if (n == 0 || std::hypot(sum_x, sum_y) < 1e-6) {
    return false;
  }
  out_heading = std::atan2(sum_y, sum_x);
  return true;
}

bool BackOutFromObstacle::computeEscapeHeading(double rx, double ry, double & out_heading)
{
  nav2_msgs::msg::Costmap::SharedPtr cm;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    cm = costmap_msg_;
  }
  if (!cm) {
    return false;
  }

  const double res = cm->metadata.resolution;
  const int size_x = static_cast<int>(cm->metadata.size_x);
  const int size_y = static_cast<int>(cm->metadata.size_y);
  const double origin_x = cm->metadata.origin.position.x;
  const double origin_y = cm->metadata.origin.position.y;
  if (res <= 0.0 || size_x == 0 || size_y == 0) {
    return false;
  }

  const int mrx = static_cast<int>((rx - origin_x) / res);
  const int mry = static_cast<int>((ry - origin_y) / res);
  const int radius_cells = static_cast<int>(search_radius_ / res);

  // Cost-weighted repulsive vector: sum of unit vectors pointing from each
  // obstacle cell toward the robot. The resultant points away from the obstacle
  // mass, even when the footprint's max cost is saturated at 253/254.
  double sum_x = 0.0;
  double sum_y = 0.0;
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    const int my = mry + dy;
    if (my < 0 || my >= size_y) {continue;}
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const int mx = mrx + dx;
      if (mx < 0 || mx >= size_x) {continue;}
      if (dx == 0 && dy == 0) {continue;}

      const uint8_t cost = cm->data[my * size_x + mx];
      // 255 (NO_INFORMATION) is unknown, not an obstacle.
      if (cost < obstacle_cost_min_ || cost == 255) {continue;}

      const double cx = origin_x + (mx + 0.5) * res;
      const double cy = origin_y + (my + 0.5) * res;
      const double vx = rx - cx;
      const double vy = ry - cy;
      const double d = std::hypot(vx, vy);
      if (d < 1e-6) {continue;}

      const double w = static_cast<double>(cost);
      sum_x += w * vx / d;
      sum_y += w * vy / d;
    }
  }

  if (std::hypot(sum_x, sum_y) < 1e-6) {
    return false;
  }
  out_heading = std::atan2(sum_y, sum_x);
  return true;
}

nav2_behaviors::Status BackOutFromObstacle::onRun(
  const std::shared_ptr<const BackOutFromObstacleAction::Goal> command)
{
  max_distance_ = command->max_distance > 0.0 ? command->max_distance : max_distance_;
  command_speed_ = command->speed > 0.0 ? command->speed : command_speed_;
  command_time_allowance_ = command->time_allowance;
  end_time_ = this->clock_->now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
      initial_pose_, *this->tf_, costmap_frame_, this->robot_base_frame_,
      this->transform_tolerance_))
  {
    RCLCPP_ERROR(
      this->logger_, "BackOutFromObstacle: initial pose not available in frame '%s'.",
      costmap_frame_.c_str());
    return nav2_behaviors::Status::FAILED;
  }

  RCLCPP_WARN(
    this->logger_,
    "BackOutFromObstacle: escaping (max_distance=%.2f m, speed=%.2f m/s).",
    max_distance_, command_speed_);
  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status BackOutFromObstacle::onCycleUpdate()
{
  rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    this->stopRobot();
    RCLCPP_WARN(this->logger_, "BackOutFromObstacle: exceeded time allowance.");
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *this->tf_, costmap_frame_, this->robot_base_frame_,
      this->transform_tolerance_))
  {
    RCLCPP_ERROR(this->logger_, "BackOutFromObstacle: current pose not available.");
    this->stopRobot();
    return nav2_behaviors::Status::FAILED;
  }
  const double rx = current_pose.pose.position.x;
  const double ry = current_pose.pose.position.y;
  const double theta = tf2::getYaw(current_pose.pose.orientation);

  const double distance = std::hypot(
    initial_pose_.pose.position.x - rx, initial_pose_.pose.position.y - ry);
  feedback_->distance_traveled = distance;
  this->action_server_->publish_feedback(feedback_);

  // Completion metric: max footprint cost (arm-aware) at the current pose.
  geometry_msgs::msg::Pose2D current2d;
  current2d.x = rx;
  current2d.y = ry;
  current2d.theta = theta;
  const double footprint_cost = this->collision_checker_->scorePose(current2d, true);

  // Escape the inflation gradient that freezes the controller, not just strict
  // collision: stop once the footprint cost is low enough for it to move again.
  if (footprint_cost < clear_cost_threshold_) {
    this->stopRobot();
    RCLCPP_WARN(
      this->logger_, "BackOutFromObstacle: clear (cost %.0f) after %.2f m.",
      footprint_cost, distance);
    return nav2_behaviors::Status::SUCCEEDED;
  }

  if (distance >= max_distance_) {
    this->stopRobot();
    RCLCPP_WARN(
      this->logger_,
      "BackOutFromObstacle: reached max distance %.2f m (cost still %.0f).",
      max_distance_, footprint_cost);
    return nav2_behaviors::Status::SUCCEEDED;
  }

  // Direction: with the MoveIt backend, choose the escape side from the filtered
  // pointcloud (sees what the arm is near in 3D); otherwise, or if no cloud is
  // available, fall back to the cost-weighted repulsive vector over the raw
  // costmap (robust even at saturated cost where the footprint max has no gradient).
  double heading_world;
  bool have_dir = false;
  if (planner_backend_ == "moveit") {
    have_dir = computeEscapeHeadingFromCloud(rx, ry, heading_world);
  }
  if (!have_dir) {
    have_dir = computeEscapeHeading(rx, ry, heading_world);
  }
  if (!have_dir) {
    // No obstacle points/cells found (or no data yet): nothing to push away from.
    this->stopRobot();
    RCLCPP_WARN(
      this->logger_,
      "BackOutFromObstacle: no obstacle direction found (cost %.0f) after %.2f m.",
      footprint_cost, distance);
    return nav2_behaviors::Status::SUCCEEDED;
  }

  // Veto: don't strafe if projecting the footprint forward along the escape
  // heading would land in a *worse* pose (e.g. toward another obstacle). This
  // keeps the backout from driving the (arm-aware) footprint into a new hazard.
  geometry_msgs::msg::Pose2D probe;
  probe.x = rx + forward_check_dist_ * std::cos(heading_world);
  probe.y = ry + forward_check_dist_ * std::sin(heading_world);
  probe.theta = theta;
  const double projected_cost = this->collision_checker_->scorePose(probe, false);
  if (projected_cost > footprint_cost) {
    this->stopRobot();
    RCLCPP_WARN(
      this->logger_,
      "BackOutFromObstacle: escape blocked by another obstacle "
      "(cost %.0f -> %.0f) after %.2f m.", footprint_cost, projected_cost, distance);
    return nav2_behaviors::Status::SUCCEEDED;
  }

  // World-frame escape heading -> body-frame strafe (turret stays fixed in map,
  // no rotation; the base realizes the lateral motion via the SimController).
  const double body_angle = heading_world - theta;
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = command_speed_ * std::cos(body_angle);
  cmd_vel->linear.y = command_speed_ * std::sin(body_angle);
  cmd_vel->angular.z = 0.0;
  this->vel_pub_->publish(std::move(cmd_vel));

  return nav2_behaviors::Status::RUNNING;
}

}  // namespace robo_drill

PLUGINLIB_EXPORT_CLASS(robo_drill::BackOutFromObstacle, nav2_core::Behavior)
