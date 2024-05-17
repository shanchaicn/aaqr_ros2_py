import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.subscriber = self.create_subscription(
            String, 'my_topic', self.callback, 10)
        self.sub_thread = threading.Thread(target=self.subscribe_thread)
        self.sub_thread.start()

    def callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)

    def subscribe_thread(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
