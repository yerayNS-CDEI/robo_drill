// Copyright 2026 robo_drill

#include "robo_drill/retract_arm_from_obstacle.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "pluginlib/class_list_macros.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace robo_drill
{

namespace
{
double normalizeAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}
}  // namespace

RetractArmFromObstacle::RetractArmFromObstacle()
: TimedBehavior<RetractArmFromObstacleAction>(),
  feedback_(std::make_shared<RetractArmFromObstacleAction::Feedback>()),
  phase_(Phase::PanSpin),
  prefer_pan_spin_(true),
  fold_fallback_(true),
  max_pan_change_(1.571),
  prev_cost_(255.0),
  costmap_frame_("odom"),
  search_radius_(2.0),
  obstacle_cost_min_(128),
  clear_cost_threshold_(50.0),
  planner_backend_("moveit"),
  joint_goal_topic_("/arm/joint_goal"),
  cancel_service_("/emergency_stop"),
  execution_status_topic_("/execution_status"),
  goal_failed_topic_("/planner/goal_failed"),
  joint_states_topic_("/joint_states"),
  arm_mount_yaw_(-2.3562),
  home_lift_(-1.2),
  home_elbow_(-2.3),
  pan_spin_lift_vertical_(-1.5708),
  pan_spin_lift_step_(0.7854),
  max_lift_raise_(0.8),
  elbow_lift_coupling_(1.0),
  lift_baseline_(0.0),
  lift_baseline_set_(false),
  tip_frame_("arm_tool0"),
  pan_joint_sign_(1.0),
  pan_sign_(1.0),
  pan_flipped_(false)
{
  arm_joints_ = {
    "arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_joint",
    "arm_wrist_1_joint", "arm_wrist_2_joint", "arm_wrist_3_joint"};
}

void RetractArmFromObstacle::onConfigure()
{
  auto node = this->node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  const std::string n = this->behavior_name_;

  nav2_util::declare_parameter_if_not_declared(
    node, n + ".costmap_frame", rclcpp::ParameterValue("odom"));
  node->get_parameter(n + ".costmap_frame", costmap_frame_);
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".search_radius", rclcpp::ParameterValue(2.0));
  node->get_parameter(n + ".search_radius", search_radius_);
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".obstacle_cost_min", rclcpp::ParameterValue(128));
  node->get_parameter(n + ".obstacle_cost_min", obstacle_cost_min_);
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".clear_cost_threshold", rclcpp::ParameterValue(50.0));
  node->get_parameter(n + ".clear_cost_threshold", clear_cost_threshold_);
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".planner_backend", rclcpp::ParameterValue("moveit"));
  node->get_parameter(n + ".planner_backend", planner_backend_);
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".prefer_pan_spin", rclcpp::ParameterValue(true));
  node->get_parameter(n + ".prefer_pan_spin", prefer_pan_spin_);
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".fold_fallback", rclcpp::ParameterValue(true));
  node->get_parameter(n + ".fold_fallback", fold_fallback_);
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".joint_goal_topic", rclcpp::ParameterValue(joint_goal_topic_));
  node->get_parameter(n + ".joint_goal_topic", joint_goal_topic_);
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".cancel_service", rclcpp::ParameterValue(cancel_service_));
  node->get_parameter(n + ".cancel_service", cancel_service_);
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".execution_status_topic", rclcpp::ParameterValue(execution_status_topic_));
  node->get_parameter(n + ".execution_status_topic", execution_status_topic_);
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".goal_failed_topic", rclcpp::ParameterValue(goal_failed_topic_));
  node->get_parameter(n + ".goal_failed_topic", goal_failed_topic_);
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".joint_states_topic", rclcpp::ParameterValue(joint_states_topic_));
  node->get_parameter(n + ".joint_states_topic", joint_states_topic_);
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".arm_joints", rclcpp::ParameterValue(arm_joints_));
  node->get_parameter(n + ".arm_joints", arm_joints_);
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".arm_mount_yaw", rclcpp::ParameterValue(-2.3562));
  node->get_parameter(n + ".arm_mount_yaw", arm_mount_yaw_);
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".home_lift", rclcpp::ParameterValue(-1.2));
  node->get_parameter(n + ".home_lift", home_lift_);
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".home_elbow", rclcpp::ParameterValue(-2.3));
  node->get_parameter(n + ".home_elbow", home_elbow_);
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".tip_frame", rclcpp::ParameterValue(tip_frame_));
  node->get_parameter(n + ".tip_frame", tip_frame_);
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".pan_joint_sign", rclcpp::ParameterValue(1.0));
  node->get_parameter(n + ".pan_joint_sign", pan_joint_sign_);
  pan_sign_ = pan_joint_sign_ >= 0.0 ? 1.0 : -1.0;
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".pan_spin_lift_vertical", rclcpp::ParameterValue(-1.5708));
  node->get_parameter(n + ".pan_spin_lift_vertical", pan_spin_lift_vertical_);
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".pan_spin_lift_step", rclcpp::ParameterValue(0.7854));
  node->get_parameter(n + ".pan_spin_lift_step", pan_spin_lift_step_);
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".max_lift_raise", rclcpp::ParameterValue(0.8));
  node->get_parameter(n + ".max_lift_raise", max_lift_raise_);
  nav2_util::declare_parameter_if_not_declared(
    node, n + ".elbow_lift_coupling", rclcpp::ParameterValue(1.0));
  node->get_parameter(n + ".elbow_lift_coupling", elbow_lift_coupling_);

  if (arm_joints_.size() != 6) {
    RCLCPP_ERROR(
      this->logger_, "RetractArmFromObstacle: arm_joints must list 6 joints (got %zu).",
      arm_joints_.size());
  }

  joint_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_, rclcpp::SensorDataQoS(),
    std::bind(&RetractArmFromObstacle::onJointState, this, std::placeholders::_1));

  std::string costmap_topic = "local_costmap/costmap_raw";
  node->get_parameter("costmap_topic", costmap_topic);
  rclcpp::QoS costmap_qos(rclcpp::KeepLast(1));
  costmap_qos.reliable().transient_local();
  costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
    costmap_topic, costmap_qos,
    std::bind(&RetractArmFromObstacle::onCostmap, this, std::placeholders::_1));

  exec_status_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    execution_status_topic_, 10,
    [this](const std_msgs::msg::Bool::SharedPtr m) {if (m->data) {execution_succeeded_ = true;}});
  goal_failed_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    goal_failed_topic_, 10,
    [this](const std_msgs::msg::Bool::SharedPtr m) {if (m->data) {goal_failed_ = true;}});

  joint_goal_pub_ = node->create_publisher<sensor_msgs::msg::JointState>(joint_goal_topic_, 1);
  cancel_client_ = node->create_client<std_srvs::srv::Trigger>(cancel_service_);

  RCLCPP_INFO(
    this->logger_, "RetractArmFromObstacle configured (planner goal=%s, cancel=%s).",
    joint_goal_topic_.c_str(), cancel_service_.c_str());
}

void RetractArmFromObstacle::onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joint_mutex_);
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    joint_pos_[msg->name[i]] = msg->position[i];
  }
}

void RetractArmFromObstacle::onCostmap(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  costmap_msg_ = msg;
}

double RetractArmFromObstacle::jointPos(const std::string & name, double fallback)
{
  std::lock_guard<std::mutex> lock(joint_mutex_);
  auto it = joint_pos_.find(name);
  return it != joint_pos_.end() ? it->second : fallback;
}

bool RetractArmFromObstacle::computeEscapeHeading(double rx, double ry, double & out_heading)
{
  nav2_msgs::msg::Costmap::SharedPtr cm;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    cm = costmap_msg_;
  }
  if (!cm) {return false;}

  const double res = cm->metadata.resolution;
  const int size_x = static_cast<int>(cm->metadata.size_x);
  const int size_y = static_cast<int>(cm->metadata.size_y);
  const double origin_x = cm->metadata.origin.position.x;
  const double origin_y = cm->metadata.origin.position.y;
  if (res <= 0.0 || size_x == 0 || size_y == 0) {return false;}

  const int mrx = static_cast<int>((rx - origin_x) / res);
  const int mry = static_cast<int>((ry - origin_y) / res);
  const int radius_cells = static_cast<int>(search_radius_ / res);

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
  if (std::hypot(sum_x, sum_y) < 1e-6) {return false;}
  out_heading = std::atan2(sum_y, sum_x);
  return true;
}

void RetractArmFromObstacle::sendJointGoal(
  double pan, double lift, double elbow, double w1, double w2, double w3)
{
  sensor_msgs::msg::JointState goal;
  goal.header.stamp = this->clock_->now();
  goal.name = arm_joints_;
  goal.position = {pan, lift, elbow, w1, w2, w3};
  joint_goal_pub_->publish(goal);
}

void RetractArmFromObstacle::cancelArm()
{
  if (cancel_client_ && cancel_client_->service_is_ready()) {
    cancel_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  }
}

void RetractArmFromObstacle::onActionCompletion()
{
  cancelArm();
}

double RetractArmFromObstacle::liftRaiseTarget(double cur_lift)
{
  // Track the resting (least-raised) lift as the baseline so the total raise can
  // be bounded. "Raised" is toward vertical, which is the more-negative side, so
  // the baseline is the *largest* lift value we've seen.
  if (!lift_baseline_set_) {
    lift_baseline_ = cur_lift;
    lift_baseline_set_ = true;
  } else {
    lift_baseline_ = std::max(lift_baseline_, cur_lift);
  }

  // Most-raised lift allowed: no more than max_lift_raise_ above the baseline,
  // and never past vertical. (Both limits are on the more-negative side.)
  const double raise_floor = std::max(pan_spin_lift_vertical_, lift_baseline_ - max_lift_raise_);

  // Raise toward vertical, capped at pan_spin_lift_step_ per goal so the lift
  // creeps up rather than snapping, then clamp to the total-raise floor so it
  // doesn't keep climbing on every obstacle.
  const double delta = std::clamp(
    pan_spin_lift_vertical_ - cur_lift, -pan_spin_lift_step_, pan_spin_lift_step_);
  return std::max(cur_lift + delta, raise_floor);
}

double RetractArmFromObstacle::elbowRetractTarget(double cur_elbow, double lift_delta) const
{
  // Counter-rotate the forearm opposite to the lift raise so the arm folds
  // inward (retracts) instead of swinging its reach outward as the shoulder
  // lifts. lift_delta is negative when raising, so this rotates the elbow the
  // other way.
  return cur_elbow - elbow_lift_coupling_ * lift_delta;
}

bool RetractArmFromObstacle::computePanTarget(double cur_pan, double & out_pan)
{
  if (max_pan_change_ <= 0.0) {return false;}
  geometry_msgs::msg::PoseStamped pose;
  if (!nav2_util::getCurrentPose(
      pose, *this->tf_, costmap_frame_, this->robot_base_frame_, this->transform_tolerance_))
  {
    return false;
  }
  const double theta = tf2::getYaw(pose.pose.orientation);
  double heading_world;
  if (!computeEscapeHeading(pose.pose.position.x, pose.pose.position.y, heading_world)) {
    return false;
  }
  // Obstacle-free "away" direction expressed in the base frame.
  const double away_body = normalizeAngle(heading_world - theta);

  // Current arm-tip azimuth in the base frame: read from the real TF when
  // available, else fall back to the analytic mount-yaw + pan estimate.
  double tip_az;
  try {
    const auto tf = this->tf_->lookupTransform(
      this->robot_base_frame_, tip_frame_, tf2::TimePointZero);
    tip_az = std::atan2(tf.transform.translation.y, tf.transform.translation.x);
  } catch (const tf2::TransformException & ex) {
    tip_az = normalizeAngle(arm_mount_yaw_ + cur_pan);
    RCLCPP_WARN(
      this->logger_, "RetractArmFromObstacle: TF %s->%s failed (%s); using analytic azimuth.",
      this->robot_base_frame_.c_str(), tip_frame_.c_str(), ex.what());
  }

  // Rotate the tip toward the away direction, applying the working pan sign so
  // a self-correction (reversePanSpin) flips which way the shoulder pan turns,
  // capped so the sweep can't fling the arm around.
  const double delta = normalizeAngle(away_body - tip_az);
  out_pan = cur_pan + std::clamp(pan_sign_ * delta, -max_pan_change_, max_pan_change_);
  return true;
}

void RetractArmFromObstacle::startFold()
{
  const double cur_pan = jointPos(arm_joints_[0], 0.0);
  const double w1 = jointPos(arm_joints_[3], 0.0);
  const double w2 = jointPos(arm_joints_[4], 0.0);
  const double w3 = jointPos(arm_joints_[5], 0.0);
  // Fold reach toward the compact home shape, keeping the pan where the spin
  // left it. Cancel-on-clear still stops it partway.
  sendJointGoal(cur_pan, home_lift_, home_elbow_, w1, w2, w3);
  phase_ = Phase::Fold;
  execution_succeeded_ = false;
  goal_failed_ = false;
  prev_cost_ = 255.0;
  RCLCPP_WARN(this->logger_, "RetractArmFromObstacle: pan-spin did not clear, folding reach down.");
}

void RetractArmFromObstacle::reversePanSpin()
{
  pan_sign_ = -pan_sign_;
  pan_flipped_ = true;

  const double cur_pan = jointPos(arm_joints_[0], 0.0);
  const double cur_lift = jointPos(arm_joints_[1], home_lift_);
  const double cur_elbow = jointPos(arm_joints_[2], home_elbow_);
  const double w1 = jointPos(arm_joints_[3], 0.0);
  const double w2 = jointPos(arm_joints_[4], 0.0);
  const double w3 = jointPos(arm_joints_[5], 0.0);

  double pan_target;
  if (computePanTarget(cur_pan, pan_target)) {
    const double lift_target = liftRaiseTarget(cur_lift);
    const double elbow_target = elbowRetractTarget(cur_elbow, lift_target - cur_lift);
    sendJointGoal(pan_target, lift_target, elbow_target, w1, w2, w3);
    RCLCPP_WARN(
      this->logger_, "RetractArmFromObstacle: first spin worsened it; reversing pan direction "
      "(pan %.2f->%.2f, lift %.2f->%.2f, elbow %.2f->%.2f rad).",
      cur_pan, pan_target, cur_lift, lift_target, cur_elbow, elbow_target);
  } else {
    RCLCPP_WARN(this->logger_, "RetractArmFromObstacle: reverse spin has no direction; folding.");
  }
  execution_succeeded_ = false;
  goal_failed_ = false;
  prev_cost_ = 255.0;
}

nav2_behaviors::Status RetractArmFromObstacle::onRun(
  const std::shared_ptr<const RetractArmFromObstacleAction::Goal> command)
{
  max_pan_change_ = command->max_pan_change != 0.0 ?
    std::fabs(command->max_pan_change) : max_pan_change_;
  command_time_allowance_ = command->time_allowance;
  end_time_ = this->clock_->now() + command_time_allowance_;
  prev_cost_ = 255.0;
  execution_succeeded_ = false;
  goal_failed_ = false;
  pan_sign_ = pan_joint_sign_ >= 0.0 ? 1.0 : -1.0;
  pan_flipped_ = false;

  // The joint-space retract is only wired for the MoveIt backend. Under the
  // legacy (Cartesian) planner, hand off to the next recovery (base back-out).
  if (planner_backend_ != "moveit") {
    RCLCPP_WARN(
      this->logger_,
      "RetractArmFromObstacle: planner_backend='%s' (not moveit) - skipping arm retract, "
      "handing off to base recovery.", planner_backend_.c_str());
    return nav2_behaviors::Status::FAILED;
  }

  const double cur_pan = jointPos(arm_joints_[0], 1e9);
  if (cur_pan > 1e8) {
    RCLCPP_ERROR(this->logger_, "RetractArmFromObstacle: no /joint_states for the arm yet.");
    return nav2_behaviors::Status::FAILED;
  }
  const double cur_lift = jointPos(arm_joints_[1], home_lift_);
  const double cur_elbow = jointPos(arm_joints_[2], home_elbow_);
  const double w1 = jointPos(arm_joints_[3], 0.0);
  const double w2 = jointPos(arm_joints_[4], 0.0);
  const double w3 = jointPos(arm_joints_[5], 0.0);

  double pan_target;
  const bool have_dir = computePanTarget(cur_pan, pan_target);

  if (prefer_pan_spin_ && have_dir) {
    // Pan-spin: reorient the arm toward the clear side, raise the shoulder lift
    // toward vertical (bounded), and counter-rotate the forearm so the arm folds
    // inward as it lifts; the planner sweeps it slowly and we cancel the moment
    // the footprint clears.
    const double lift_target = liftRaiseTarget(cur_lift);
    const double elbow_target = elbowRetractTarget(cur_elbow, lift_target - cur_lift);
    sendJointGoal(pan_target, lift_target, elbow_target, w1, w2, w3);
    phase_ = Phase::PanSpin;
    RCLCPP_WARN(
      this->logger_, "RetractArmFromObstacle: retracting arm to clear footprint "
      "(pan %.2f->%.2f, lift %.2f->%.2f, elbow %.2f->%.2f rad).",
      cur_pan, pan_target, cur_lift, lift_target, cur_elbow, elbow_target);
  } else {
    // No clear direction (or pan-spin disabled): fold reach toward home.
    const double pan = have_dir ? pan_target : cur_pan;
    sendJointGoal(pan, home_lift_, home_elbow_, w1, w2, w3);
    phase_ = Phase::Fold;
    RCLCPP_WARN(
      this->logger_, "RetractArmFromObstacle: folding arm to clear footprint "
      "(pan %.2f -> %.2f rad).", cur_pan, pan);
  }
  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status RetractArmFromObstacle::onCycleUpdate()
{
  rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    cancelArm();
    RCLCPP_WARN(this->logger_, "RetractArmFromObstacle: exceeded time allowance.");
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped pose;
  if (!nav2_util::getCurrentPose(
      pose, *this->tf_, costmap_frame_, this->robot_base_frame_, this->transform_tolerance_))
  {
    cancelArm();
    RCLCPP_ERROR(this->logger_, "RetractArmFromObstacle: current pose not available.");
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = pose.pose.position.x;
  pose2d.y = pose.pose.position.y;
  pose2d.theta = tf2::getYaw(pose.pose.orientation);
  const double cost = this->collision_checker_->scorePose(pose2d, true);
  feedback_->footprint_cost = cost;
  this->action_server_->publish_feedback(feedback_);

  // Cleared: cancel the planner goal so the arm stops where it is (minimal).
  if (cost < clear_cost_threshold_) {
    cancelArm();
    RCLCPP_WARN(this->logger_, "RetractArmFromObstacle: footprint clear (cost %.0f).", cost);
    return nav2_behaviors::Status::SUCCEEDED;
  }

  // When the current attempt cannot clear it (planner failed, motion finished,
  // or it made things worse), self-correct the pan direction once, then fall
  // back from pan-spin to folding before giving up to the base recovery.
  const bool worse = cost > prev_cost_ + 1.0;
  if (goal_failed_ || execution_succeeded_ || worse) {
    cancelArm();
    if (phase_ == Phase::PanSpin) {
      // A wrong-direction spin shows up as a raised cost or a refused plan.
      // Reverse the pan once before giving up on spinning; a cleanly finished
      // motion that simply didn't reach far enough goes straight to folding.
      if ((worse || goal_failed_) && !pan_flipped_) {
        reversePanSpin();
        return nav2_behaviors::Status::RUNNING;
      }
      if (fold_fallback_) {
        startFold();
        return nav2_behaviors::Status::RUNNING;
      }
    }
    const char * why = worse ? "raised cost" : (goal_failed_ ? "planner failure" : "fully retracted");
    RCLCPP_WARN(
      this->logger_, "RetractArmFromObstacle: %s, still not clear (cost %.0f) - handing off.",
      why, cost);
    return nav2_behaviors::Status::SUCCEEDED;
  }
  prev_cost_ = cost;

  return nav2_behaviors::Status::RUNNING;
}

}  // namespace robo_drill

PLUGINLIB_EXPORT_CLASS(robo_drill::RetractArmFromObstacle, nav2_core::Behavior)
