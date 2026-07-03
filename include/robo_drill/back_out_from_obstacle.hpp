// Copyright 2026 robo_drill
//
// Direction-aware recovery behavior. Unlike the stock BackUp/DriveOnHeading
// behaviors (which only move along the robot's +/-x axis), this behavior
// strafes the holonomic base directly away from whatever the (dynamic,
// arm-aware) footprint is wedged against. It performs gradient descent on the
// footprint collision score so it escapes sideways / 45-degree wedges, then
// yields control back to Nav2 for replanning once the footprint is clear.

#ifndef ROBO_DRILL__BACK_OUT_FROM_OBSTACLE_HPP_
#define ROBO_DRILL__BACK_OUT_FROM_OBSTACLE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav2_behaviors/timed_behavior.hpp"
#include "robo_drill/action/back_out_from_obstacle.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace robo_drill
{

using BackOutFromObstacleAction = robo_drill::action::BackOutFromObstacle;

class BackOutFromObstacle : public nav2_behaviors::TimedBehavior<BackOutFromObstacleAction>
{
public:
  BackOutFromObstacle();
  ~BackOutFromObstacle() override = default;

  nav2_behaviors::Status onRun(
    const std::shared_ptr<const BackOutFromObstacleAction::Goal> command) override;

  nav2_behaviors::Status onCycleUpdate() override;

  void onConfigure() override;

protected:
  // Compute the world-frame heading (rad) that points away from the surrounding
  // obstacle cells: a cost-weighted repulsive vector summed over costmap cells
  // within search_radius_ of the robot. Returns false if no obstacle cells were
  // found (nothing to escape) or no costmap has arrived yet.
  bool computeEscapeHeading(double rx, double ry, double & out_heading);

  // Same repulsive-vector idea, but summed over the robot-body-filtered
  // pointcloud (cloud_topic_) instead of the 2D costmap: it sees obstacles the
  // arm is near in 3D and decides which side to strafe toward. Returns false if
  // no cloud/transform is available or no points fall in the search band.
  bool computeEscapeHeadingFromCloud(double rx, double ry, double & out_heading);

  void onCostmap(const nav2_msgs::msg::Costmap::SharedPtr msg);
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  BackOutFromObstacleAction::Feedback::SharedPtr feedback_;

  geometry_msgs::msg::PoseStamped initial_pose_;
  double max_distance_;
  double command_speed_;
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;

  // Raw local costmap (for the repulsive escape-direction computation).
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  nav2_msgs::msg::Costmap::SharedPtr costmap_msg_;
  std::mutex costmap_mutex_;

  // Robot-body-filtered pointcloud (preferred escape-direction source on the
  // MoveIt backend).
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_;
  std::mutex cloud_mutex_;

  // Frame the local costmap is published in (matches local_costmap.global_frame).
  std::string costmap_frame_;
  // Radius (m) around the robot to sum obstacle cells over. Should cover the
  // footprint extent (incl. arm reach) so far cells the arm overlaps are seen.
  double search_radius_;
  // Only cells at/above this cost contribute to the repulsive vector.
  int obstacle_cost_min_;
  // Stop once the footprint's max cost drops below this (out of the steep part
  // of the inflation gradient that freezes the controller).
  double clear_cost_threshold_;
  // How far ahead (m) to project the footprint to veto strafing into a *worse*
  // pose (e.g. toward another obstacle) while escaping.
  double forward_check_dist_;

  // Arm planner backend (from the launch). "moveit" -> use the filtered cloud to
  // choose the escape side; anything else -> costmap-only.
  std::string planner_backend_;
  // Filtered pointcloud topic and the height band (costmap_frame z) of points
  // that count as obstacles.
  std::string cloud_topic_;
  double cloud_min_z_;
  double cloud_max_z_;
};

}  // namespace robo_drill

#endif  // ROBO_DRILL__BACK_OUT_FROM_OBSTACLE_HPP_
