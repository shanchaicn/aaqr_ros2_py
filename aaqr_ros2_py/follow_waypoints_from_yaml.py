#!/usr/bin/env python3

import rclpy
import yaml
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import os
from geometry_msgs.msg import PoseStamped, Quaternion, Point

home_dir = os.path.expanduser('~')
aaqr_dir = os.path.join(home_dir, 'aaqr')
pose_dir = os.path.join(aaqr_dir, 'pose')
os.makedirs(pose_dir, exist_ok=True)

filepath = os.path.join(pose_dir, 'map.yaml')

def load_waypoints_from_yaml(file_path):
    with open(file_path, 'r') as yaml_file:
        waypoints = yaml.safe_load(yaml_file)
    return waypoints


def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before initializing pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Load goal poses from YAML file

    goal_poses = load_waypoints_from_yaml(filepath)


    poses = []
    for key, value in goal_poses.items():
        pose = PoseStamped()
        pose.header.frame_id = 'map'  # Change the frame_id as required
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position = Point(x=value['position']['x'], y=value['position']['y'], z=value['position']['z'])
        pose.pose.orientation = Quaternion(w=value['orientation']['w'],
                                            x=value['orientation']['x'],
                                            y=value['orientation']['y'],
                                            z=value['orientation']['z'])
        poses.append(pose)

    # Undock
    navigator.undock()

    # Follow Waypoints
    navigator.startFollowWaypoints(poses)

    # Finished navigating, dock
    navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
