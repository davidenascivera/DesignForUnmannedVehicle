import rclpy
from rclpy.node import Node
import numpy as np
from px4_msgs.msg import SensorCombined
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
import transforms3d  # Updated import

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        self.get_logger().info('EKF Node initialized')

        # Create the QoS profile
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            SensorCombined, 
            '/fmu/out/sensor_combined', 
            self.imu_callback, 
            qos_profile
        )
        self.pos_sub = self.create_subscription(
            PoseStamped, 
            '/Drone/pose', 
            self.pos_quat_callback, 
            qos_profile
        )
        
        # Publisher
        self.estimate_pub = self.create_publisher(
            PoseStamped, 
            '/ekf_estimate', 
            qos_profile
        )

        # State variables
        self.state = np.zeros(6)  # [x, y, z, vx, vy, vz]
        self.P = np.eye(6) * 1.0  # Covariance matrix
        
        # Noise matrices
        self.Q = np.eye(6) * 0.01  # Process noise covariance
        self.R = np.eye(3) * (0.1 ** 2)  # Measurement noise covariance
        
        # Initialize variables
        self.last_time = None
        self.accel = np.zeros(3)
        self.orientation = np.array([0, 0, 0, 1])  # Quaternion [x, y, z, w]
        self.measurement = None
        
        # Gravity vector
        self.gravity = np.array([0, 0, 9.80665])  # Standard gravity in m/s^2

        # Define the transformation matrix from PX4 FRD to ROS FLU
        # This is a 180-degree rotation around the X-axis
        self.frd_to_flu_rotation = transforms3d.axangles.axangle2mat([1, 0, 0], np.pi)

    def imu_callback(self, msg):
        # Extract linear acceleration in body frame (FRD)
        accel_body_frd = np.array([
            msg.accelerometer_m_s2[0],
            msg.accelerometer_m_s2[1],
            msg.accelerometer_m_s2[2]
        ])

        # Transform acceleration from FRD to FLU
        accel_body_flu = self.frd_to_flu_rotation @ accel_body_frd

        # Transform acceleration to world frame (ENU)
        accel_world = self.transform_accel_to_world(accel_body_flu, self.orientation)
        
        # Subtract gravity
        self.accel = accel_world - self.gravity
        self.get_logger().info(f'Acceleration: {self.accel}')

    def pos_quat_callback(self, msg):
        # Extract position
        pos = np.array([
            msg.pose.position.x, 
            msg.pose.position.y, 
            msg.pose.position.z
        ])
        
        # Extract and normalize orientation quaternion (FLU)
        self.orientation = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        
        # Add Gaussian noise
        noisy_pos = pos + np.random.normal(0, 0.1, size=3)
        self.measurement = noisy_pos

    def transform_accel_to_world(self, accel_body, orientation):
        norm = np.linalg.norm(orientation)
        if norm == 0:
            self.get_logger().warning('Quaternion norm is zero. Using identity quaternion.')
            orientation = np.array([0, 0, 0, 1])  # Identity quaternion
        else:
            orientation = orientation / norm
        
        # Convert quaternion to rotation matrix
        # transforms3d expects quaternions in [x, y, z, w] format
        rotation_matrix = transforms3d.quaternions.quat2mat(orientation)
        
        # Transform acceleration to world frame (ENU)
        accel_world = rotation_matrix @ accel_body
        return accel_world


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
