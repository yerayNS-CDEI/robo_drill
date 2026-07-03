/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef NAV_EXPLORE_H_
#define NAV_EXPLORE_H_

#include <robo_drill/costmap_client.h>
#include <robo_drill/frontier_search.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::placeholders;

#define ACTION_NAME "navigate_to_pose"  // ROS 2 Humble

namespace explore
{
/**
 * @class Explore
 * @brief A class adhering to the robot_actions::Action interface that moves the
 * robot base to explore its environment.
 */
class Explore : public rclcpp::Node
{
public:
  Explore();
  ~Explore();

  void start();
  void stop(bool finished_exploring = false);
  void resume();

  using NavigationGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

private:
  /**
   * @brief  Make a global plan
   */
  void makePlan();

  // /**
  //  * @brief  Publish a frontiers as markers
  //  */
  void visualizeFrontiers(
      const std::vector<frontier_exploration::Frontier>& frontiers);

    /**
   * @brief  Publish Nav2 goal as a marker for visualization
   * @param goal The navigation goal pose
   * @param is_exploration_goal True for red exploration goal, false for green return-to-home goal
   */
  void publishGoalMarker(const geometry_msgs::msg::PoseStamped& goal_pose, 
                         bool is_exploration_goal = true);
                             
  bool goalOnBlacklist(const geometry_msgs::msg::Point& goal);

  /**
   * @brief Add a goal to the (time-expiring) blacklist.
   */
  void addToBlacklist(const geometry_msgs::msg::Point& goal);

  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  // void
  // goal_response_callback(std::shared_future<NavigationGoalHandle::SharedPtr>
  // future);
  void reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                   const geometry_msgs::msg::Point& frontier_goal);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_array_publisher_;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    goal_marker_publisher_;
  rclcpp::Logger logger_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  Costmap2DClient costmap_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      move_base_client_;
  frontier_exploration::FrontierSearch search_;
  rclcpp::TimerBase::SharedPtr exploring_timer_;
  // rclcpp::TimerBase::SharedPtr oneshot_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_subscription_;
  void resumeCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // Blacklisted goals with the time they were added, so a goal that failed once
  // (e.g. a transient obstacle) becomes eligible again after blacklist_expiry_.
  struct BlacklistedGoal {
    geometry_msgs::msg::Point point;
    rclcpp::Time stamp;
  };
  std::vector<BlacklistedGoal> frontier_blacklist_;
  geometry_msgs::msg::Point prev_goal_;
  double prev_distance_;
  rclcpp::Time last_progress_;
  size_t last_markers_count_;

  // Commit-to-goal state: while a goal is in flight we keep pursuing it (no
  // preempting Nav2 on every map update) until it is reached, aborted, becomes
  // invalid, or a dramatically closer frontier appears.
  bool goal_active_ = false;
  double current_goal_distance_ = 0.0;

  // Startup grace: the costmap is mostly unknown right after launch, so a
  // transient "no frontiers" result must NOT permanently end exploration. We
  // only treat an empty result as "done" once the robot has actually started
  // exploring (sent at least one goal), or after startup_timeout_ elapses.
  bool ever_sent_goal_ = false;
  rclcpp::Time start_time_;
  double startup_timeout_;

  geometry_msgs::msg::Pose initial_pose_;
  void returnToInitialPose(void);

  // parameters
  double planner_frequency_;
  double potential_scale_, orientation_scale_, gain_scale_;
  double progress_timeout_;
  bool visualize_;
  bool return_to_init_;
  std::string robot_base_frame_;
  bool resuming_ = false;
  // robustness / behavior tuning
  double min_obstacle_clearance_;
  double clearance_search_radius_;
  double frontier_size_cap_;
  double blacklist_radius_;
  double blacklist_expiry_;
  double switch_ratio_;
  // When false (default) the robot commits to an active goal until Nav2 reports it
  // terminal (reached/aborted) or it stalls; it is NOT abandoned just because its
  // backing frontier was explored away/blacklisted mid-transit. Set true to restore
  // the old "replan as soon as the active goal is invalidated" behavior.
  bool replan_on_invalid_goal_;
  bool goal_orientation_to_frontier_;
  // Minimum distance (m) from the robot to a frontier's projected goal for that
  // frontier to be selectable. Goals inside the (slightly inflated) footprint are
  // no-ops Nav2 reports "reached" without moving, which stalls exploration at
  // startup. Treated as the inflated footprint radius.
  double min_goal_distance_;
};
}  // namespace explore

#endif