#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main() -> None:
    rclpy.init()

    navigator = BasicNavigator()
    navigator.waitUntilNav2Active(localizer='controller_server')

    # Define your list of goal poses
    goal_positions = [
        (1.0, 0.0, 0.0, 1.0),  # (x, y, z, w)
        (2.0, 0.0, 0.0, 1.0),
        (3.0, 0.0, 0.0, 1.0)
    ]

    for idx, (x, y, z, w) in enumerate(goal_positions):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = z
        goal_pose.pose.orientation.w = w

        print(f"Sending goal {idx + 1}: x={x}, y={y}")
        navigator.goToPose(goal_pose)

        i = 0
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                print(f"Goal {idx + 1} ETA: {eta:.0f} seconds")

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"Goal {idx + 1} succeeded!")
        elif result == TaskResult.CANCELED:
            print(f"Goal {idx + 1} was canceled!")
        elif result == TaskResult.FAILED:
            (error_code, error_msg) = navigator.getTaskError()
            print(f"Goal {idx + 1} failed! {error_code}: {error_msg}")
        else:
            print(f"Goal {idx + 1} has an unknown result.")

    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()
