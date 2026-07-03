#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from action_msgs.srv import CancelGoal
from unique_identifier_msgs.msg import UUID

class CancelGoalNode(Node):
    def __init__(self):
        super().__init__('cancel_goal_node')
        self.cancel_goal_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')

    def send_cancel_request(self, uuid=None):
        # Wait for the CancelGoal service to be available
        self.get_logger().info('Waiting for the cancel_goal service...')
        self.cancel_goal_client.wait_for_service()

        # Create a CancelGoal request
        cancel_request = CancelGoal.Request()
        if uuid is None:
            # Cancel all goals (UUID of all zeros)
            cancel_request.goal_info.goal_id = UUID(uuid=[0] * 16)
            cancel_request.goal_info.stamp.sec = 0
            cancel_request.goal_info.stamp.nanosec = 0
        else:
            # Cancel a specific goal by UUID
            cancel_request.goal_info.goal_id = UUID(uuid=uuid)

        # Send the request and wait for the response
        self.get_logger().info('Sending cancel request...')
        future = self.cancel_goal_client.call_async(cancel_request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        # Process the response
        if response and response.goals_canceling:
            self.get_logger().info(f'Successfully canceled {len(response.goals_canceling)} goal(s).')
        else:
            self.get_logger().warn('No active goals to cancel.')

def main(args=None):
    rclpy.init(args=args)
    node = CancelGoalNode()

    # Example: Cancel all goals
    node.send_cancel_request()

    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
