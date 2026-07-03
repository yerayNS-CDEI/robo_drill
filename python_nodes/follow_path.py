#!/usr/bin/env python3

import math
import csv
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

"""
Navigation demo that follows a path read from a CSV file.
CSV format: x,y,yaw_deg

Run with:
    ros2 run your_package your_script_name --ros-args -p csv_path:=/absolute/path/to/path.csv
"""

def create_pose_stamped(x, y, yaw_deg, frame_id='map', stamp=None):
    pose = PoseStamped()
    # pose.header.frame_id = frame_id
    # pose.header.stamp = stamp

    pose.pose.position.x = x
    pose.pose.position.y = y

    yaw_rad = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

    return pose

def load_path_from_csv(file_path, frame_id, stamp):
    path_msg = Path()
    path_msg.header.frame_id = frame_id
    path_msg.header.stamp = stamp

    with open(file_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if row[0].lower() == 'x':
                continue  # skip header
            x, y, yaw_deg = map(float, row)
            pose = create_pose_stamped(x, y, yaw_deg, frame_id, stamp)
            path_msg.poses.append(pose)

    return path_msg

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.declare_parameter('csv_path', '')
        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value

        if not csv_path:
            self.get_logger().error("Missing required parameter: csv_path")
            rclpy.shutdown()
            return

        navigator = BasicNavigator()
        navigator.waitUntilNav2Active(localizer='controller_server')

        now = navigator.get_clock().now().to_msg()
        path_msg = load_path_from_csv(csv_path, frame_id='map', stamp=now)

        self.get_logger().info(f"Loaded {len(path_msg.poses)} poses from: {csv_path}")
        navigator.followPath(path_msg)

        i = 0
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info(
                    f"Remaining distance: {feedback.distance_to_goal:.2f} m | Speed: {feedback.speed:.2f} m/s"
                )

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded.")
        elif result == TaskResult.CANCELED:
            self.get_logger().warn("Goal was canceled.")
        elif result == TaskResult.FAILED:
            error_code, error_msg = navigator.getTaskError()
            self.get_logger().error(f"Goal failed: {error_code}: {error_msg}")
        else:
            self.get_logger().warn("Goal has an unknown status.")

        # navigator.lifecycleShutdown()
        # rclpy.shutdown()


def main():
    rclpy.init()
    node = PathFollower()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
