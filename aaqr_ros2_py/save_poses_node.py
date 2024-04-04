from docutils import SettingsSpec
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml

import sys, select

import tty, termios
import os

home_dir = os.path.expanduser('~')

filepath = os.path.join(home_dir, 'current_pose.yaml')

class PoseSaver(Node):
    def __init__(self):

        super().__init__('pose_saver')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Press 's' to save the current pose to YAML file.")
        

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.get_logger().info(f"Received posex: {self.current_pose.position.x}")
        self.get_logger().info(f"Received posey: {self.current_pose.position.y}")


    def save_pose_to_yaml(self, file_path):
        pose_dict = {
            'position': {
                'x': self.current_pose.position.x,
                'y': self.current_pose.position.y,
                'z': self.current_pose.position.z
            },
            'orientation': {
                'x': self.current_pose.orientation.x,
                'y': self.current_pose.orientation.y,
                'z': self.current_pose.orientation.z,
                'w': self.current_pose.orientation.w
            }
        }
        with open(file_path, 'w') as yaml_file:
            yaml.dump(pose_dict, yaml_file)
            self.get_logger().info(f"Pose saved to {file_path}")
    
    def save_points(self, pose_id, pose_yaml, file_path):
        pose_dict = {
            'position': {
                'x': self.current_pose.position.x,
                'y': self.current_pose.position.y,
                'z': self.current_pose.position.z
            },
            'orientation': {
                'x': self.current_pose.orientation.x,
                'y': self.current_pose.orientation.y,
                'z': self.current_pose.orientation.z,
                'w': self.current_pose.orientation.w
            }
        }
        pose_yaml[pose_id] = pose_dict
        with open(file_path, 'w') as yaml_file:
            yaml.dump(pose_yaml, yaml_file)
            self.get_logger().info("Saved points to {}".format(filepath))

    def delete_last_pose(self, points_yaml, filepath):
        if points_yaml:
            last_pose_id = max(points_yaml.keys())
            del points_yaml[last_pose_id]
            with open(filepath, "w") as yamlfile:
                yaml.dump(points_yaml, yamlfile)
            self.get_logger().info("Deleted the last saved pose (ID: {})".format(last_pose_id))
        else:
            self.get_logger().info("No poses to delete.")

    def run(self):
        if os.path.exists(filepath):
            self.get_logger().info("The YAML file already exists. Quitting the node.")
            pass
        else:
            points_yaml = {}
            pose_id = 1
            try:
                while rclpy.ok():
                    key = input("Press 's' to save pose, 'd' to delete last pose, 'q' to quit: ")
                    if key == 's':
                        rclpy.spin_once(self)
                        self.save_points(pose_id,points_yaml,filepath)
                        print("==== save point {} ====".format(pose_id))
                        pose_id = pose_id + 1
                    elif key == 'd':
                        self.delete_last_pose(points_yaml, filepath)
                        print("=== delete last pose ===")
                        pose_id = pose_id - 1
                    elif key == 'q':
                        break
            except KeyboardInterrupt:
                pass
            finally:
                self.get_logger().info("Shutting down...")


def main():
    rclpy.init()
    pose_saver = PoseSaver()
    pose_saver.run()
    print('node end')
    rclpy.shutdown()
    print('rclpy end')


if __name__ == '__main__':
    main()
