import os
import threading
import yaml
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException, Buffer, TransformListener

home_dir = os.path.expanduser('~')
aaqr_dir = os.path.join(home_dir, 'aaqr')
pose_dir = os.path.join(aaqr_dir, 'pose')
os.makedirs(pose_dir, exist_ok=True)

class PoseSaver(Node):
    def __init__(self):
        super().__init__('pose_saver')
        self.map_name = self.declare_parameter('map_name', 'map').get_parameter_value().string_value
        self.filepath = os.path.join(pose_dir, f'{self.map_name}.yaml')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.spin_thread = threading.Thread(target=self.spin)
        self.spin_thread.start()
        
        self.get_logger().info("Save_poses_node to save the current pose with keyboard.")
        self.get_logger().info(f"{self.filepath}")

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)

    def save_points(self, pose_id, pose_yaml, file_path):
        while rclpy.ok():
            try:
                self.t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                self.get_logger().info("Transform 'map' to 'base_link' is available.")
                
                pose_dict = {
                    'position': {
                        'x': self.t.transform.translation.x,
                        'y': self.t.transform.translation.y,
                        'z': self.t.transform.translation.z
                    },
                    'orientation': {
                        'x': self.t.transform.rotation.x,
                        'y': self.t.transform.rotation.y,
                        'z': self.t.transform.rotation.z,
                        'w': self.t.transform.rotation.w
                    }
                }
                
                pose_yaml[pose_id] = pose_dict
                self.get_logger().info(f"x= {self.t.transform.translation.x:.2f}, y={self.t.transform.translation.y:.2f}")
                
                with open(file_path, 'w') as yaml_file:
                    yaml.dump(pose_yaml, yaml_file)
                    self.get_logger().info("pose updated")
                break
            except TransformException:
                pass

    def delete_last_pose(self, points_yaml, filepath):
        if points_yaml:
            last_pose_id = max(points_yaml.keys())
            del points_yaml[last_pose_id]
            with open(filepath, 'w') as yaml_file:
                yaml.dump(points_yaml, yaml_file)
            self.get_logger().info(f"Deleted the last saved pose (ID: {last_pose_id})")
        else:
            self.get_logger().info("No poses to delete.")

    def run(self):
        if os.path.exists(self.filepath):
            self.get_logger().info("The YAML file already exists. Quitting the node.")
            return
        else:
            points_yaml = {}
            pose_id = 1
            try:
                while rclpy.ok():
                    key = input("Press 's' to save pose, 'd' to delete last pose, 'q' to quit: ")
                    if key == 's':
                        self.save_points(pose_id, points_yaml, self.filepath)
                        print(f"==== save point {pose_id} ====")
                        pose_id += 1
                    elif key == 'd':
                        self.delete_last_pose(points_yaml, self.filepath)
                        print("=== delete last pose ===")
                        pose_id -= 1
                    elif key == 'q':
                        break
            except KeyboardInterrupt:
                pass
            finally:
                self.get_logger().info("Shutting down...")

def main():
    rclpy.init()
    node = PoseSaver()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    node.spin_thread.join()

if __name__ == '__main__':
    main()
