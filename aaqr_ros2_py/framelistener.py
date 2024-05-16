import math

from geometry_msgs.msg import Twist, PointStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class FrameListener(Node):

    def __init__(self):
        super().__init__('turtlebot4_tf2_frame_listener')

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
            'target_frame', 'odom').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

        # Create a publisher for the turtle's position with timestamp
        self.position_publisher = self.create_publisher(PointStamped, 'turtlebot4_position', 10)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = 'base_link'
        to_frame_rel = self.target_frame

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        # Extract the position from the transform
        position = t.transform.translation

        # Create a PointStamped message and fill it with the position data and timestamp
        position_msg = PointStamped()
        position_msg.header.stamp = t.header.stamp
        position_msg.header.frame_id = to_frame_rel
        position_msg.point.x = position.x
        position_msg.point.y = position.y
        position_msg.point.z = position.z

        # Publish the position
        self.position_publisher.publish(position_msg)


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
