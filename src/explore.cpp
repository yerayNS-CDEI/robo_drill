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

#include <robo_drill/explore.h>

#include <algorithm>
#include <thread>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);
  // robustness / behavior tuning
  this->declare_parameter<float>("min_obstacle_clearance", 0.45);
  this->declare_parameter<float>("clearance_search_radius", 0.8);
  this->declare_parameter<float>("frontier_size_cap", 30.0);
  this->declare_parameter<float>("blacklist_radius", 0.75);
  this->declare_parameter<float>("blacklist_expiry", 30.0);
  this->declare_parameter<float>("switch_ratio", 0.5);
  this->declare_parameter<bool>("replan_on_invalid_goal", false);
  this->declare_parameter<bool>("goal_orientation_to_frontier", true);
  this->declare_parameter<float>("min_goal_distance", 0.7);
  this->declare_parameter<float>("footprint_clearing_radius", 0.6);
  this->declare_parameter<float>("startup_timeout", 30.0);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("min_obstacle_clearance", min_obstacle_clearance_);
  this->get_parameter("clearance_search_radius", clearance_search_radius_);
  this->get_parameter("frontier_size_cap", frontier_size_cap_);
  this->get_parameter("blacklist_radius", blacklist_radius_);
  this->get_parameter("blacklist_expiry", blacklist_expiry_);
  this->get_parameter("switch_ratio", switch_ratio_);
  this->get_parameter("replan_on_invalid_goal", replan_on_invalid_goal_);
  this->get_parameter("goal_orientation_to_frontier",
                      goal_orientation_to_frontier_);
  this->get_parameter("min_goal_distance", min_goal_distance_);
  double footprint_clearing_radius;
  this->get_parameter("footprint_clearing_radius", footprint_clearing_radius);
  this->get_parameter("startup_timeout", startup_timeout_);
  start_time_ = this->now();

  progress_timeout_ = timeout;
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  search_ = frontier_exploration::FrontierSearch(
      costmap_client_.getCostmap(), potential_scale_, gain_scale_,
      min_frontier_size, min_obstacle_clearance_, clearance_search_radius_,
      frontier_size_cap_, footprint_clearing_radius, logger_);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // Publisher for Nav2 goal marker
goal_marker_publisher_ =
    this->create_publisher<visualization_msgs::msg::Marker>("explore/goal_marker", 10);

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  // Start exploration right away
  makePlan();
}

Explore::~Explore()
{
  stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::msg::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::msg::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::msg::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = rclcpp::Duration::from_seconds(0);  // Humble onwards
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    // m.pose.position = {}; // compile warning
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.goal)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.initial;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::publishGoalMarker(const geometry_msgs::msg::PoseStamped& goal_pose,
                                bool is_exploration_goal)
{
  visualization_msgs::msg::Marker goal_marker;
  goal_marker.header = goal_pose.header;
  goal_marker.ns = "nav2_goal";
  goal_marker.id = 0;
  goal_marker.type = visualization_msgs::msg::Marker::ARROW;
  goal_marker.action = visualization_msgs::msg::Marker::ADD;
  goal_marker.pose = goal_pose.pose;
  goal_marker.scale.x = 0.5;  // Arrow length
  goal_marker.scale.y = 0.1;  // Arrow width
  goal_marker.scale.z = 0.1;  // Arrow height
  
  if (is_exploration_goal) {
    // Red arrow for exploration goals
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 0.0;
  } else {
    // Green arrow for return-to-home goal
    goal_marker.color.r = 0.0;
    goal_marker.color.g = 1.0;
    goal_marker.color.b = 0.0;
  }
  goal_marker.color.a = 1.0;
  goal_marker.lifetime = rclcpp::Duration::from_seconds(0);  // Lives forever
  
  goal_marker_publisher_->publish(goal_marker);
}

void Explore::makePlan()
{
  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  // get frontiers sorted according to cost (nearest-first, see frontierCost)
  auto frontiers = search_.searchFrom(pose.position);
  RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f dist: %f size: %u", i,
                 frontiers[i].cost, frontiers[i].min_distance,
                 frontiers[i].size);
  }

  if (frontiers.empty()) {
    // Don't permanently quit on a transient empty result while the map is still
    // filling in around the robot at startup; wait for the next cycle instead.
    if (!ever_sent_goal_ &&
        (this->now() - start_time_) < tf2::durationFromSec(startup_timeout_)) {
      RCLCPP_INFO_THROTTLE(logger_, *this->get_clock(), 2000,
                           "No frontiers yet - waiting for the map to grow.");
      return;
    }
    RCLCPP_WARN(logger_, "No frontiers found, stopping.");
    stop(true);
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // Best candidate: lowest-cost frontier that has a clearance-safe goal point
  // (frontier_search already dropped frontiers with no safe approach point), is
  // not currently blacklisted, AND whose goal is far enough from the robot to
  // actually require motion. A goal inside the (slightly inflated) footprint is a
  // no-op Nav2 reports "reached" without moving, so the map never grows and
  // exploration stalls in place - skip those.
  auto reachable = [this, &pose](const frontier_exploration::Frontier& f) {
    if (!f.goal_valid || goalOnBlacklist(f.goal)) {
      return false;
    }
    const double d = std::hypot(f.goal.x - pose.position.x,
                                f.goal.y - pose.position.y);
    return d >= min_goal_distance_;
  };
  auto frontier = std::find_if(frontiers.begin(), frontiers.end(), reachable);

  geometry_msgs::msg::Point candidate;
  double candidate_distance;

  if (frontier != frontiers.end()) {
    // Normal case: target the clearance-safe projected point, NOT the raw
    // centroid, so the goal never lands on/next to a wall.
    candidate = frontier->goal;
    candidate_distance = frontier->min_distance;
  } else {
    // Nothing far enough to drive to. Either exploration is genuinely complete,
    // or (typically at startup) the only frontiers sit inside the footprint.
    // Distinguish the two by looking for ANY valid, non-blacklisted frontier.
    auto nearest =
        std::find_if(frontiers.begin(), frontiers.end(),
                     [this](const frontier_exploration::Frontier& f) {
                       return f.goal_valid && !goalOnBlacklist(f.goal);
                     });
    if (nearest == frontiers.end()) {
      // Same startup grace: all current frontiers are merely blacklisted/invalid
      // transients; wait rather than declaring exploration finished in place.
      if (!ever_sent_goal_ &&
          (this->now() - start_time_) < tf2::durationFromSec(startup_timeout_)) {
        RCLCPP_INFO_THROTTLE(logger_, *this->get_clock(), 2000,
                             "No reachable frontier yet - waiting for the map "
                             "to grow.");
        return;
      }
      RCLCPP_WARN(logger_, "All frontiers traversed/tried out, stopping.");
      stop(true);
      return;
    }
    // Within-footprint frontier(s) only: synthesize a goal just beyond the
    // footprint heading toward the nearest frontier, so the robot starts moving
    // and reveals new map instead of declaring itself finished in place.
    frontier = nearest;
    double dx = nearest->centroid.x - pose.position.x;
    double dy = nearest->centroid.y - pose.position.y;
    double norm = std::hypot(dx, dy);
    if (norm < 1e-3) {
      dx = 1.0;
      dy = 0.0;
      norm = 1.0;
    }
    const double reach = min_goal_distance_ + 0.1;
    candidate.x = pose.position.x + (dx / norm) * reach;
    candidate.y = pose.position.y + (dy / norm) * reach;
    candidate.z = 0.0;
    candidate_distance = reach;
    RCLCPP_INFO(logger_,
                "Only within-footprint frontiers; nudging forward to "
                "(%.2f, %.2f) to start moving.",
                candidate.x, candidate.y);
  }

  // --- Commit-to-goal hysteresis ---
  // While a goal is in flight, keep pursuing it rather than preempting Nav2 on
  // every map update (that caused the robot to swing). Only re-decide if the
  // active goal became invalid, we stalled past the progress timeout, or a
  // dramatically closer frontier appeared.
  if (goal_active_) {
    double robot_to_goal = std::hypot(prev_goal_.x - pose.position.x,
                                      prev_goal_.y - pose.position.y);
    // progress watchdog: reset the stall timer whenever we get measurably closer
    if (robot_to_goal + 0.05 < prev_distance_) {
      last_progress_ = this->now();
      prev_distance_ = robot_to_goal;
    }

    bool stalled = (this->now() - last_progress_ >
                    tf2::durationFromSec(progress_timeout_)) &&
                   !resuming_;

    // active goal no longer backed by a frontier (area explored) or blacklisted.
    // Only evaluated when replan_on_invalid_goal_ is set; by default we commit to
    // the active goal and let Nav2 drive it to a terminal result (or stall) instead.
    bool active_invalid = false;
    if (replan_on_invalid_goal_) {
      bool frontier_still_there = false;
      for (const auto& f : frontiers) {
        if (f.goal_valid &&
            std::hypot(f.goal.x - prev_goal_.x, f.goal.y - prev_goal_.y) <
                blacklist_radius_) {
          frontier_still_there = true;
          break;
        }
      }
      active_invalid = goalOnBlacklist(prev_goal_) || !frontier_still_there;
    }

    bool much_closer =
        switch_ratio_ > 0.0 &&
        candidate_distance < switch_ratio_ * current_goal_distance_ &&
        !same_point(candidate, prev_goal_);

    if (stalled) {
      RCLCPP_WARN(logger_,
                  "No progress toward goal for %.1fs - blacklisting and "
                  "replanning.",
                  progress_timeout_);
      addToBlacklist(prev_goal_);
      goal_active_ = false;
      // No explicit cancel: the goal we send next preempts this one. An explicit
      // cancel here would halt follow_path and then the new goal would halt it
      // again - the double halt is what produces "Failed to get result for
      // follow_path in node halt!" and stalls the robot.
      // Re-enter so the candidate is re-selected against the now-updated
      // blacklist (otherwise we would re-send the goal we just blacklisted).
      makePlan();
      return;
    } else if (active_invalid) {
      RCLCPP_INFO(logger_,
                  "Active goal explored away or blacklisted - replanning.");
      goal_active_ = false;  // new goal below preempts the active one
    } else if (much_closer) {
      RCLCPP_INFO(logger_,
                  "Much closer frontier appeared (%.2fm vs %.2fm) - switching.",
                  candidate_distance, current_goal_distance_);
      goal_active_ = false;  // new goal below preempts the active one
    } else {
      // keep pursuing the committed goal; do not disturb Nav2
      return;
    }
  }

  // ensure only first call of makePlan was set resuming to true
  if (resuming_) {
    resuming_ = false;
  }

  // --- Commit to and send the new goal ---
  prev_goal_ = candidate;
  current_goal_distance_ = candidate_distance;
  prev_distance_ = candidate_distance;
  last_progress_ = this->now();
  goal_active_ = true;
  ever_sent_goal_ = true;

  RCLCPP_INFO(logger_, "New GOAL: (%.2f, %.2f) dist=%.2fm size=%u",
              candidate.x, candidate.y, candidate_distance, frontier->size);

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = candidate;

  // Orient the robot to look into the unexplored frontier (better sensor
  // coverage and higher planner success) instead of a fixed heading.
  if (goal_orientation_to_frontier_) {
    double dx = frontier->centroid.x - candidate.x;
    double dy = frontier->centroid.y - candidate.y;
    if (std::hypot(dx, dy) < 1e-3) {
      // degenerate: face the direction of travel
      dx = candidate.x - pose.position.x;
      dy = candidate.y - pose.position.y;
    }
    tf2::Quaternion q;
    q.setRPY(0, 0, std::atan2(dy, dx));
    goal.pose.pose.orientation = tf2::toMsg(q);
  } else {
    goal.pose.pose.orientation.w = 1.0;
  }
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  // Publish goal marker
  publishGoalMarker(goal.pose, true);
  rclcpp::shutdown();
  send_goal_options.result_callback =
      [this, candidate](const NavigationGoalHandle::WrappedResult& result) {
        reachedGoal(result, candidate);
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  // Publish goal marker for visualization
  publishGoalMarker(goal.pose, false);
  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  
  // Add result callback to track when robot reaches initial pose
  send_goal_options.result_callback =
      [this](const NavigationGoalHandle::WrappedResult& result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(logger_, "Successfully returned to initial pose.");
            rclcpp::shutdown();  // Shutdown the node after reaching initial pose
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(logger_, "Failed to return to initial pose - goal aborted.");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(logger_, "Return to initial pose was canceled.");
            break;
          default:
            RCLCPP_ERROR(logger_, "Unknown result code from return to initial pose.");
            break;
        }
      };
  
  move_base_client_->async_send_goal(goal, send_goal_options);
}

void Explore::addToBlacklist(const geometry_msgs::msg::Point& goal)
{
  frontier_blacklist_.push_back({goal, this->now()});
  RCLCPP_INFO(logger_, "Blacklisted goal (%.2f, %.2f) for %.0fs", goal.x, goal.y,
              blacklist_expiry_);
}

bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  // Drop entries older than blacklist_expiry_ so a goal that failed once (e.g. a
  // transient obstacle) can be retried later instead of being permanently dead.
  const rclcpp::Time now = this->now();
  frontier_blacklist_.erase(
      std::remove_if(frontier_blacklist_.begin(), frontier_blacklist_.end(),
                     [&](const BlacklistedGoal& b) {
                       return (now - b.stamp) >
                              tf2::durationFromSec(blacklist_expiry_);
                     }),
      frontier_blacklist_.end());

  for (const auto& b : frontier_blacklist_) {
    if (std::hypot(goal.x - b.point.x, goal.y - b.point.y) < blacklist_radius_) {
      return true;
    }
  }
  return false;
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal)
{
  // Only the goal we are currently committed to drives planning. A goal that was
  // preempted/superseded (e.g. when we switched to a closer frontier) reports
  // ABORTED or CANCELED later - ignore it BEFORE any blacklisting, otherwise we
  // would blacklist a perfectly good goal we simply abandoned, or fire a
  // duplicate replan.
  if (!same_point(frontier_goal, prev_goal_)) {
    RCLCPP_DEBUG(logger_, "Ignoring result for superseded goal (%.2f, %.2f).",
                 frontier_goal.x, frontier_goal.y);
    return;
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger_, "Goal reached (%.2f, %.2f).", frontier_goal.x,
                  frontier_goal.y);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      // Nav2 could not reach this goal (unreachable / planner failure). Make it
      // visible and blacklist so we do not keep retrying the same dead goal.
      RCLCPP_WARN(logger_,
                  "Goal aborted by Nav2 (%.2f, %.2f) - unreachable, "
                  "blacklisting.",
                  frontier_goal.x, frontier_goal.y);
      addToBlacklist(frontier_goal);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      // Cancellation is driven by our own stop logic; do not replan here.
      RCLCPP_DEBUG(logger_, "Goal was canceled");
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      break;
  }

  // Drive the next plan immediately. Single-threaded executor serializes this
  // callback with makePlan, so a direct call is safe (no timer needed).
  goal_active_ = false;
  makePlan();
}

void Explore::start()
{
  RCLCPP_INFO(logger_, "Exploration started.");
}

void Explore::stop(bool finished_exploring)
{
  RCLCPP_INFO(logger_, "Exploration stopped.");

  goal_active_ = false;

  // Cancel exploration timer
  exploring_timer_->cancel();
  
  // If returning to initial pose, don't cancel goals yet - let current goal finish
  // then send return-to-init goal. Otherwise cancel all goals.
  if (return_to_init_ && finished_exploring) {
    // Cancel goals after a brief delay to ensure they're properly aborted
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    move_base_client_->async_cancel_all_goals();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    returnToInitialPose();
  } else {
    move_base_client_->async_cancel_all_goals();
  }
}

void Explore::resume()
{
  resuming_ = true;
  RCLCPP_INFO(logger_, "Exploration resuming.");
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
      std::make_shared<explore::Explore>()); 
  rclcpp::shutdown();
  return 0;
}