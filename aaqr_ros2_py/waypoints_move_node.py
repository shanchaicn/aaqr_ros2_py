import os
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, Point

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

home_dir = os.path.expanduser('~')
aaqr_dir = os.path.join(home_dir, 'aaqr')
pose_dir = os.path.join(aaqr_dir, 'pose')

class FrameListener(Node):

    def __init__(self):
        super().__init__('waypoints_move_node')

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
            'target_frame', 'odom').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a publisher for the turtle's position with timestamp
        self.position_publisher = self.create_publisher(PointStamped, 'turtlebot4_checkpoints', 10)

        # Initialize TurtleBot4Navigator
        self.navigator = TurtleBot4Navigator()

        # Dock the robot before initializing pose
        if not self.navigator.getDockedStatus():
            self.navigator.info('Docking before initializing pose')
            self.navigator.dock()

        # Set initial pose
        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)

        # Wait for Nav2
        self.navigator.waitUntilNav2Active()

        # Load goal poses from YAML file
        self.map_name = self.declare_parameter('map_name', 'map').get_parameter_value().string_value
        
        self.filepath = os.path.join(pose_dir, f'{self.map_name}.yaml')
        self.goal_poses = self.load_waypoints_from_yaml(self.filepath)

        # Convert goal poses to PoseStamped messages
        self.poses = []
        for key, value in self.goal_poses.items():
            pose = PoseStamped()
            pose.header.frame_id = 'map'  # Change the frame_id as required
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            pose.pose.position = Point(x=value['position']['x'], y=value['position']['y'], z=value['position']['z'])
            pose.pose.orientation = Quaternion(w=value['orientation']['w'],
                                                x=value['orientation']['x'],
                                                y=value['orientation']['y'],
                                                z=value['orientation']['z'])
            self.poses.append(pose)

        # Undock and start following waypoints
        self.navigator.undock()
        self.follow_waypoints_and_record_position()

        # After navigating, dock the robot
        self.navigator.dock()

    def load_waypoints_from_yaml(self, file_path):
        with open(file_path, 'r') as yaml_file:
            waypoints = yaml.safe_load(yaml_file)
        return waypoints

    def follow_waypoints_and_record_position(self):
        for pose in self.poses:
            self.navigator.startToPose(pose)
            # Wait until the robot reaches the current waypoint
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=1.0)
            self.record_current_position()

    def record_current_position(self):
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


if __name__ == '__main__':
    main()
