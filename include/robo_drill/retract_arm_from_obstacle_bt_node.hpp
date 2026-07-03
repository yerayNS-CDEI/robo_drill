// Copyright 2026 robo_drill
//
// Behavior-tree action node wrapping the robo_drill RetractArmFromObstacle action.

#ifndef ROBO_DRILL__RETRACT_ARM_FROM_OBSTACLE_BT_NODE_HPP_
#define ROBO_DRILL__RETRACT_ARM_FROM_OBSTACLE_BT_NODE_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "robo_drill/action/retract_arm_from_obstacle.hpp"

namespace robo_drill
{

class RetractArmFromObstacleAction
  : public nav2_behavior_tree::BtActionNode<robo_drill::action::RetractArmFromObstacle>
{
  using Action = robo_drill::action::RetractArmFromObstacle;

public:
  RetractArmFromObstacleAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("max_pan_change", 0.785, "Cap on pan bias (rad)"),
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for retracting")
      });
  }
};

}  // namespace robo_drill

#endif  // ROBO_DRILL__RETRACT_ARM_FROM_OBSTACLE_BT_NODE_HPP_
