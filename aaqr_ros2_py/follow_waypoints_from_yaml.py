#!/usr/bin/env python3

import rclpy
import yaml
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


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
    waypoints_file = 'waypoints.yaml'  # Update with your YAML file path
    goal_poses = load_waypoints_from_yaml(waypoints_file)

    # Convert loaded coordinates to PoseStamped messages
    goal_pose_msgs = []
    for pose in goal_poses:
        position = pose['position']
        direction = pose['direction']
        goal_pose_msgs.append(navigator.getPoseStamped(position, direction))

    # Undock
    navigator.undock()

    # Follow Waypoints
    navigator.startFollowWaypoints(goal_pose_msgs)

    # Finished navigating, dock
    navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
