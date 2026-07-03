// Copyright 2026 robo_drill

#include "robo_drill/retract_arm_from_obstacle_bt_node.hpp"

#include <memory>
#include <string>

namespace robo_drill
{

RetractArmFromObstacleAction::RetractArmFromObstacleAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<robo_drill::action::RetractArmFromObstacle>(
    xml_tag_name, action_name, conf)
{
}

void RetractArmFromObstacleAction::on_tick()
{
  double max_pan_change = 0.785;
  double time_allowance = 10.0;
  getInput("max_pan_change", max_pan_change);
  getInput("time_allowance", time_allowance);

  goal_.max_pan_change = static_cast<float>(max_pan_change);
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

}  // namespace robo_drill

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<robo_drill::RetractArmFromObstacleAction>(
        name, "retract_arm_from_obstacle", config);
    };

  factory.registerBuilder<robo_drill::RetractArmFromObstacleAction>(
    "RetractArmFromObstacle", builder);
}
