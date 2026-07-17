#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Helper to compute byte size for PointField datatypes
def _datatype_size(datatype: int) -> int:
    sizes = {
        PointField.INT8: 1,
        PointField.UINT8: 1,
        PointField.INT16: 2,
        PointField.UINT16: 2,
        PointField.INT32: 4,
        PointField.UINT32: 4,
        PointField.FLOAT32: 4,
        PointField.FLOAT64: 8,
    }
    return sizes.get(datatype, 0)

class RemoveIntensityNode(Node):
    def __init__(self):
        super().__init__('republish_lidar')

        # Parameters
        self.declare_parameter('input_topic', '/points')
        self.declare_parameter('output_topic', '/points_no_intensity')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # Subscriber and Publisher
        self.subscriber = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pointcloud_callback,
            qos_profile
        )

        # Use same QoS profile for publisher
        self.publisher = self.create_publisher(PointCloud2, output_topic, qos_profile)

        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing to: {output_topic}')

    def pointcloud_callback(self, msg: PointCloud2):
        # Collect fields and locate intensity
        fields = list(msg.fields)
        names = [f.name for f in fields]
        if 'intensity' not in names:
            self.publisher.publish(msg)
            return

        # Identify intensity field and its size
        intensity_field = next(f for f in fields if f.name == 'intensity')
        count = intensity_field.count if intensity_field.count > 0 else 1
        intensity_size = _datatype_size(intensity_field.datatype) * count
        intensity_offset = intensity_field.offset

        # Build new fields with adjusted offsets (remove intensity bytes)
        new_fields = []
        for f in fields:
            if f.name == 'intensity':
                continue
            new_offset = f.offset if f.offset < intensity_offset else f.offset - intensity_size
            new_fields.append(PointField(name=f.name, offset=new_offset, datatype=f.datatype, count=f.count))
        new_fields.sort(key=lambda f: f.offset)

        # Compute new point and row steps
        new_point_step = msg.point_step - intensity_size
        new_row_step = new_point_step * msg.width

        # Remove intensity bytes per point from raw data
        data_mv = memoryview(msg.data)
        new_data = bytearray(new_row_step * msg.height)
        write_idx = 0
        for r in range(msg.height):
            row_start = r * msg.row_step
            for c in range(msg.width):
                p_start = row_start + c * msg.point_step
                # head before intensity
                head = data_mv[p_start : p_start + intensity_offset]
                # tail after intensity
                tail = data_mv[p_start + intensity_offset + intensity_size : p_start + msg.point_step]
                new_data[write_idx : write_idx + len(head)] = head
                write_idx += len(head)
                new_data[write_idx : write_idx + len(tail)] = tail
                write_idx += len(tail)

        # Create new PointCloud2 message
        new_msg = PointCloud2()
        new_msg.header = msg.header
        new_msg.height = msg.height
        new_msg.width = msg.width
        new_msg.fields = new_fields
        new_msg.is_bigendian = msg.is_bigendian
        new_msg.point_step = new_point_step
        new_msg.row_step = new_row_step
        new_msg.data = bytes(new_data)
        new_msg.is_dense = msg.is_dense

        self.publisher.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RemoveIntensityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
