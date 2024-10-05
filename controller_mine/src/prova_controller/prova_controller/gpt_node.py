import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorCombined
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

class SensorCombinedListener(Node):

    def __init__(self):
        super().__init__('sensor_combined_listener')
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.subscription = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.sensor_combined_callback,
            qos_profile
        )
    
    def sensor_combined_callback(self, msg):
        #self.get_logger().info("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
        #self.get_logger().info('RECEIVED SENSOR COMBINED DATA')
        #self.get_logger().info('=============================')
        #self.get_logger().info(f'ts: {msg.timestamp}')
        #self.get_logger().info(f'gyro_rad[0]: {msg.gyro_rad[0]}')
        #self.get_logger().info(f'gyro_rad[1]: {msg.gyro_rad[1]}')
        #self.get_logger().info(f'gyro_rad[2]: {msg.gyro_rad[2]}')
        #self.get_logger().info(f'gyro_integral_dt: {msg.gyro_integral_dt}')
        #self.get_logger().info(f'accelerometer_timestamp_relative: {msg.accelerometer_timestamp_relative}')
        #self.get_logger().info(f'accelerometer_m_s2[0]: {msg.accelerometer_m_s2[0]}')
        #self.get_logger().info(f'accelerometer_m_s2[1]: {msg.accelerometer_m_s2[1]}')
        self.get_logger().info(f'accelerometer_m_s2[2]: {msg.accelerometer_m_s2[2]}')
        #self.get_logger().info(f'accelerometer_integral_dt: {msg.accelerometer_integral_dt}')

def main(args=None):
    print("Starting sensor_combined listener node...")
    rclpy.init(args=args)
    node = SensorCombinedListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
