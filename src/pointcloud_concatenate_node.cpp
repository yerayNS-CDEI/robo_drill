#include "robo_drill/pointcloud_concatenate.hpp"

int main(int argc, char **argv)
{
  // Create node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointcloudConcatenate>();

  // The node creates its own update timer (in a separate callback group from
  // the subscriptions). A MultiThreadedExecutor lets the subscriptions keep
  // draining while update() is busy in TF/concatenation, so a slow merge can't
  // throttle the input rate.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
