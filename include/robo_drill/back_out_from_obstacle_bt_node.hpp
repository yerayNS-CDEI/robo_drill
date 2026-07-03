// Copyright 2026 robo_drill
//
// Behavior-tree action node wrapping the robo_drill BackOutFromObstacle action,
// so it can be invoked from a Nav2 recovery behavior tree.

#ifndef ROBO_DRILL__BACK_OUT_FROM_OBSTACLE_BT_NODE_HPP_
#define ROBO_DRILL__BACK_OUT_FROM_OBSTACLE_BT_NODE_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "robo_drill/action/back_out_from_obstacle.hpp"

namespace robo_drill
{

class BackOutFromObstacleAction
  : public nav2_behavior_tree::BtActionNode<robo_drill::action::BackOutFromObstacle>
{
  using Action = robo_drill::action::BackOutFromObstacle;

public:
  BackOutFromObstacleAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("max_distance", 0.5, "Max distance (m) to escape"),
        BT::InputPort<double>("speed", 0.1, "Escape strafe speed (m/s)"),
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for escaping")
      });
  }
};

}  // namespace robo_drill

#endif  // ROBO_DRILL__BACK_OUT_FROM_OBSTACLE_BT_NODE_HPP_
