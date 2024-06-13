
import os
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, Point
import time
import rclpy
from rclpy.node import Node
import threading
from rclpy.executors import MultiThreadedExecutor
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from datetime import datetime, timedelta
import requests

home_dir = os.path.expanduser('~')
aaqr_dir = os.path.join(home_dir, 'aaqr')
pose_dir = os.path.join(aaqr_dir, 'pose')


class Waypointsmove(Node):

    def __init__(self):
        super().__init__('waypoints_move_node')

        self.position_sub = self.create_subscription(PointStamped, 'turtlebot4_position', self.listener_callback, 10)

        # Initialize TurtleBot4Navigator
        self.navigator = TurtleBot4Navigator()

        # Dock the robot before initializing pose
        # if not self.navigator.getDockedStatus():
        #     self.navigator.info('Docking before initializing pose')
        #     self.navigator.dock()

        # Set initial pose
        # if  self.navigator.getDockedStatus():
        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().debug("Set_the_initial_pose")
        # Wait for Nav2
        self.navigator.waitUntilNav2Active()

        # Load goal poses from YAML file
        self.wp_name = self.declare_parameter('wp_name', 'wp').get_parameter_value().string_value
        self.filepath = os.path.join(pose_dir, f'{self.wp_name}.yaml')
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

        # Start the subscription thread
        self.subscription_thread = threading.Thread(target=self.subscribe_thread)
        self.subscription_thread.start()

        # Undock and start following waypoints
        self.navigator.undock()
        self.follow_waypoints_and_record_position()
        # Go near the dock
        self.navigator.info('Docking for charge')
        self.navigator.startToPose(self.navigator.getPoseStamped([-0.3, 0.0],
                                TurtleBot4Directions.NORTH))
        self.navigator.dock()

        if not self.navigator.getDockedStatus():
            self.navigator.error('Robot failed to dock')
            
        # After navigating, dock the robot
        # self.navigator.dock()
        self.get_logger().info('Finished')
        rclpy.shutdown()

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
            # Save the current position after reaching the waypoint
            time.sleep(5.0) # 5sec
            self.record_current_position()


    def listener_callback(self, msg):
        self.get_logger().info('Received PointStamped: x = %.2f, y = %.2f' % (msg.point.x, msg.point.y))
        self.real_position = msg.point
        self.real_timestamp = msg.header.stamp

    def subscribe_thread(self):
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(self)
        executor.spin()

    # def record_current_position(self):
    #     #####################
    #     #####################

    #     # Change this to save the sensor reading and stampedpoint into SQL
    #     #####################

    #     with open(os.path.join(pose_dir, 'current_position.yaml'), 'a') as file:
    #         yaml.dump({
    #             'position': {
    #                 'x': self.real_position.x,
    #                 'y': self.real_position.y,
    #                 'z': self.real_position.z
    #             },
    #             'timestamp': {
    #                 'sec': self.real_timestamp.sec,
    #                 'nanosec': self.real_timestamp.nanosec
    #             }
    #         }, file)
    #     self.get_logger().info(f'Recorded position: x = {self.real_position.x}, y = {self.real_position.y}, z = {self.real_position.z}')

    def record_current_position(self):         
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
                'position': {                     
                    'x': self.real_position.x,                     
                    'y': self.real_position.y,                     
                    'z': self.real_position.z                 
                    },                 
                'timestamp': formatted_timestamp             
                }, file)         
        self.get_logger().info(f'Recorded position: x = {self.real_position.x}, y = {self.real_position.y}, z = {self.real_position.z}, timestamp = {formatted_timestamp}')

        

def main():
    rclpy.init()
    node = Waypointsmove()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
