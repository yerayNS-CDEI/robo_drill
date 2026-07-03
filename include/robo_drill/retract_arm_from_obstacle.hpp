// Copyright 2026 robo_drill
//
// Recovery behavior that frees the (arm-aware) footprint from an obstacle by
// retracting the UR10e arm instead of moving the base. It routes the motion
// through the arm planner (MoveIt) rather than commanding the controller
// directly: it publishes a folded "home-ward, biased away from the obstacle"
// joint goal to /arm/joint_goal, lets the planner execute it (collision-aware,
// at its own max_velocity_scaling), and cancels via the /emergency_stop service
// the instant the footprint clears -> minimal, planned retraction. Because the
// arm stays retracted, it also stops the chassis from sweeping a fully extended
// arm back into the obstacle while turning to follow the path.

#ifndef ROBO_DRILL__RETRACT_ARM_FROM_OBSTACLE_HPP_
#define ROBO_DRILL__RETRACT_ARM_FROM_OBSTACLE_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_behaviors/timed_behavior.hpp"
#include "robo_drill/action/retract_arm_from_obstacle.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace robo_drill
{

using RetractArmFromObstacleAction = robo_drill::action::RetractArmFromObstacle;

class RetractArmFromObstacle : public nav2_behaviors::TimedBehavior<RetractArmFromObstacleAction>
{
public:
  RetractArmFromObstacle();
  ~RetractArmFromObstacle() override = default;

  nav2_behaviors::Status onRun(
    const std::shared_ptr<const RetractArmFromObstacleAction::Goal> command) override;

  nav2_behaviors::Status onCycleUpdate() override;

  void onConfigure() override;
  void onActionCompletion() override;

protected:
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onCostmap(const nav2_msgs::msg::Costmap::SharedPtr msg);

  double jointPos(const std::string & name, double fallback);

  // World-frame heading pointing away from the surrounding obstacle mass.
  bool computeEscapeHeading(double rx, double ry, double & out_heading);

  // Publish a joint goal to the planner.
  void sendJointGoal(double pan, double lift, double elbow, double w1, double w2, double w3);
  // Cancel the active planner goal (clean, non-latching) so the arm stops here.
  void cancelArm();
  // Switch from pan-spin to the folding fallback (fold lift/elbow toward home).
  void startFold();
  // Shoulder-lift goal that raises the arm toward vertical (z-up) from its
  // current pose by up to pan_spin_lift_step_ per goal, but never more than
  // max_lift_raise_ above the arm's resting lift (tracked in lift_baseline_) nor
  // past pan_spin_lift_vertical_ - so it doesn't keep creeping up on every
  // obstacle. Updates lift_baseline_.
  double liftRaiseTarget(double cur_lift);
  // Forearm (elbow) goal that counter-rotates opposite to the lift raise by
  // elbow_lift_coupling_, so raising the shoulder folds the arm inward (retract)
  // instead of swinging the reach outward.
  double elbowRetractTarget(double cur_elbow, double lift_delta) const;
  // Reverse the working pan direction once and resend the pan-spin goal (the
  // self-correction when the first spin worsened the cost or the planner balked).
  void reversePanSpin();
  // Pan target (rad) that rotates the arm-tip azimuth toward the obstacle-free
  // side, capped by max_pan_change_. Returns false if no obstacle direction.
  bool computePanTarget(double cur_pan, double & out_pan);

  // Pan-spin reorients the arm (keeps reach) toward the clear side; if that
  // can't clear it, fall back to folding the reach down.
  enum class Phase { PanSpin, Fold };
  Phase phase_;
  bool prefer_pan_spin_;
  bool fold_fallback_;

  RetractArmFromObstacleAction::Feedback::SharedPtr feedback_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exec_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_failed_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_goal_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_client_;

  std::mutex joint_mutex_;
  std::map<std::string, double> joint_pos_;
  std::mutex costmap_mutex_;
  nav2_msgs::msg::Costmap::SharedPtr costmap_msg_;

  std::atomic<bool> execution_succeeded_{false};
  std::atomic<bool> goal_failed_{false};

  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
  double max_pan_change_;   // cap on pan bias for this run (rad)
  double prev_cost_;        // footprint cost last cycle (for the veto)

  // Frames / costmap repulsion (same model as BackOutFromObstacle).
  std::string costmap_frame_;
  double search_radius_;
  int obstacle_cost_min_;
  double clear_cost_threshold_;

  // Planner interface. Selected to match the arm launch's planner_backend:
  //  - "moveit": joint goal via /arm/joint_goal (joint-space fold; implemented).
  //  - "legacy": Cartesian planner_node (/arm/goal_pose) - not yet wired here.
  std::string planner_backend_;
  std::string joint_goal_topic_;
  std::string cancel_service_;
  std::string execution_status_topic_;
  std::string goal_failed_topic_;
  std::string joint_states_topic_;
  std::vector<std::string> arm_joints_;   // pan, lift, elbow, w1, w2, w3
  double arm_mount_yaw_;     // analytic fallback for the tip azimuth (rad)
  double home_lift_;
  double home_elbow_;
  // Pan-spin also raises the shoulder lift toward "vertical" (upper arm pointing
  // up, z-axis of map): pan_spin_lift_vertical_ is that target value and
  // pan_spin_lift_step_ caps how far we raise from the current pose per goal.
  // max_lift_raise_ caps the *total* raise above the resting lift so the arm
  // doesn't creep all the way up over successive obstacles; lift_baseline_ tracks
  // that resting (most-extended) lift. elbow_lift_coupling_ counter-rotates the
  // forearm as the lift raises so the arm retracts inward.
  double pan_spin_lift_vertical_;
  double pan_spin_lift_step_;
  double max_lift_raise_;
  double elbow_lift_coupling_;
  double lift_baseline_;
  bool lift_baseline_set_;

  // Pan-spin direction. The current tip azimuth is read from tip_frame_ via TF;
  // pan_joint_sign_ maps a +pan command to +/- world azimuth. pan_sign_ is the
  // working sign for the run (init from the param); on a worsening/failed spin
  // it is flipped once (pan_flipped_) before giving up to folding.
  std::string tip_frame_;
  double pan_joint_sign_;
  double pan_sign_;
  bool pan_flipped_;
};

}  // namespace robo_drill

#endif  // ROBO_DRILL__RETRACT_ARM_FROM_OBSTACLE_HPP_
