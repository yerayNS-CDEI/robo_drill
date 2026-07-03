// Copyright 2026 robo_drill

#include "robo_drill/back_out_from_obstacle_bt_node.hpp"

#include <memory>
#include <string>

namespace robo_drill
{

BackOutFromObstacleAction::BackOutFromObstacleAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<robo_drill::action::BackOutFromObstacle>(
    xml_tag_name, action_name, conf)
{
}

void BackOutFromObstacleAction::on_tick()
{
  double max_distance = 0.5;
  double speed = 0.1;
  double time_allowance = 10.0;
  getInput("max_distance", max_distance);
  getInput("speed", speed);
  getInput("time_allowance", time_allowance);

  goal_.max_distance = static_cast<float>(max_distance);
  goal_.speed = static_cast<float>(speed);
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

}  // namespace robo_drill

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<robo_drill::BackOutFromObstacleAction>(
        name, "back_out_from_obstacle", config);
    };

  factory.registerBuilder<robo_drill::BackOutFromObstacleAction>(
    "BackOutFromObstacle", builder);
}
