import rclpy
from rclpy.node import Node
from px4_msgs.msg import DebugVect
import time

class DebugVectAdvertiser(Node):
    def __init__(self):
        super().__init__('debug_vect_advertiser')
        self.publisher_ = self.create_publisher(DebugVect, '/fmu/in/debug_vect', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        debug_vect = DebugVect()
        debug_vect.timestamp = int(time.time() * 1e6)
        name = "test"
        name_ascii = [ord(c) for c in name] + [0] * (10 - len(name))  # Convert name to ASCII and pad to length 10
        debug_vect.name = name_ascii
        debug_vect.x = 1.0
        debug_vect.y = 2.0
        debug_vect.z = 3.0
        self.get_logger().info(f'Publishing debug_vect: time: {debug_vect.timestamp} x: {debug_vect.x} y: {debug_vect.y} z: {debug_vect.z}')
        self.publisher_.publish(debug_vect)

def main(args=None):
    print("Starting debug_vect advertiser node...")
    rclpy.init(args=args)
    node = DebugVectAdvertiser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
