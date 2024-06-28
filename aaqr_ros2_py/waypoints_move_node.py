#!/usr/bin/env python3
import os
import yaml
import requests
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from threading import Thread, Lock
from time import sleep
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, Point
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped, PointStamped
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from datetime import datetime, timedelta

BATTERY_HIGH = 0.95
BATTERY_LOW = 0.2  # when the robot will go charge
BATTERY_CRITICAL = 0.1  # when the robot will shutdown

home_dir = os.path.expanduser('~')
aaqr_dir = os.path.join(home_dir, 'aaqr')
pose_dir = os.path.join(aaqr_dir, 'pose')

class BatteryAndPoseMonitor(Node):
    def __init__(self, lock):
        super().__init__('battery_monitor')
        self.lock = lock
        self.battery_state_subscriber = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_state_callback,
            10)
        self.position_subscriber = self.create_subscription(
            PointStamped,
            'turtlebot4_position',
            self.position_callback,
            10)
        self.battery_percent = None
        self.current_position = None

    def battery_state_callback(self, batt_msg: BatteryState):
        with self.lock:
            self.battery_percent = batt_msg.percentage

    def position_callback(self, position_msg: PointStamped):
        with self.lock:
            self.current_position = position_msg
        # print(f'{self.current_position.point.x},{self.current_position.point.y}')

    def thread_function(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin()

class WaypointsMove(Node):
    def __init__(self):
        super().__init__('wp_move')
        self.lock = Lock()
        self.battery_pose_monitor = BatteryAndPoseMonitor(self.lock)
        self.navigator = TurtleBot4Navigator()
        self.monitor_thread = Thread(target=self.battery_pose_monitor.thread_function, daemon=True)
        self.monitor_thread.start()
        

        # Initial parameters in node
        self.wp_name = self.declare_parameter('wp_name', 'wp').get_parameter_value().string_value
        # self.sleep_time = 5.0
        self.sleep_time = self.declare_parameter('sleep_time', 5.0).get_parameter_value().double_value
        self.filepath = os.path.join(pose_dir, f'{self.wp_name}.yaml')
        

        # Initial pose and undock
        if not self.navigator.getDockedStatus():
            self.navigator.info('Docking before initialising pose')
            self.navigator.dock()
        initial_pose = self.navigator.getPoseStamped([0.0,0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()
        # while True:
        #     with self.lock:
        #         battery_percent = self.battery_pose_monitor.battery_percent
        #         self.real_position = self.battery_pose_monitor.current_position.point
        #         self.real_timestamp = self.battery_pose_monitor.current_position.header.stamp
        #     self.get_logger().info(f'Recorded position: x = {self.real_position.x}, y = {self.real_position.y}, z = {self.real_position.z}, timestamp = {self.real_timestamp}')
        # load waypoints from yaml
        self.navigator.info('Loading waypoints')
        self.goal_poses = self.load_waypoints_from_yaml(self.filepath)

        # follow waypoints: stop several seconds and record the pose
        for index, pose in enumerate(self.goal_poses):
            self.navigator.info(f"Heading to the Point{index}")
            self.navigator.startToPose(pose)
            sleep(self.sleep_time) # 5sec
            print("====== UPDATE + =======")
            # update the pose amd batterystate
            with self.lock:
                battery_percent = self.battery_pose_monitor.battery_percent
                self.real_position = self.battery_pose_monitor.current_position.point
                self.real_timestamp = self.battery_pose_monitor.current_position.header.stamp
            print("====== UPDATE =======")
            self.record_current_position(index)
        # Go near the dock
        self.navigator.info('Docking for charge')
        self.navigator.startToPose(self.navigator.getPoseStamped([-0.3, 0.0],
                                TurtleBot4Directions.NORTH))
        self.navigator.dock()

        if not self.navigator.getDockedStatus():
            self.navigator.error('Robot failed to dock')
            
        # After navigating, dock the robot
        self.get_logger().info('Finished')
        
    def load_waypoints_from_yaml(self,file_path):
        with open(file_path, 'r') as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
        poses = []
        
        for key, value in yaml_data.items():
            pose = PoseStamped()
            pose.header.frame_id = 'map'  # Change the frame_id as required
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            pose.pose.position = Point(x=value['position']['x'], y=value['position']['y'], z=value['position']['z'])
            pose.pose.orientation = Quaternion(w=value['orientation']['w'],
                                                x=value['orientation']['x'],
                                                y=value['orientation']['y'],
                                                z=value['orientation']['z'])
            poses.append(pose)
        return poses
    def record_current_position(self,index):         
        # Convert the ROS2 timestamp to datetime        
        timestamp_sec = self.real_timestamp.sec         
        timestamp_nanosec = self.real_timestamp.nanosec         
        timestamp = datetime.utcfromtimestamp(timestamp_sec) + timedelta(microseconds=timestamp_nanosec / 1000)         
        formatted_timestamp = timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  
        # To match the format with millisecondsmap
        # Change this to save the sensor reading and stamped point into SQL
        try:
            response = requests.post(
                'http://192.168.8.101:5000/render-timestamp-data',
                data={
                    'timestamp' : formatted_timestamp,
                    'x' : self.real_position.x,
                    'y' : self.real_position.y,
                    'z' : self.real_position.z,
                }
            )
            if response.status_code == 200:
                self.get_logger().info(f'Successfully sent data to server: {response.text}')
            else:
                self.get_logger().info(f'Failed to send data to server: {response.status_code}')
        except Exception as e:
            self.get_logger().error(f'Exception occured while sending data to server: {str(e)}')
            
        with open(os.path.join(pose_dir, f'{self.wp_name}_data.yaml'), 'a') as file:             
            yaml.dump({
                index:{
                    'position': {                     
                        'x': self.real_position.x,                     
                        'y': self.real_position.y,                     
                        'z': self.real_position.z                 
                    },                 
                'timestamp': formatted_timestamp   
                }                 
          
                }, file)         
        self.get_logger().info(f'Recorded position: x = {self.real_position.x}, y = {self.real_position.y}, z = {self.real_position.z}, timestamp = {formatted_timestamp}')

def main(args=None):
    rclpy.init(args=args)
    try:
       wp_node = WaypointsMove()
    except KeyboardInterrupt:
        # Handle Ctrl+C (SIGINT) gracefully
        wp_node.get_logger().info('WaypointsMove Node interrupted and shutting down.')
    finally:
        # Cleanup and shutdown the node and ROS properly
        wp_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()