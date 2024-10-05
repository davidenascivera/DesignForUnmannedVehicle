import rclpy
from rclpy.node import Node
import numpy as np
from px4_msgs.msg import SensorCombined
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
import transforms3d  # Updated import
import csv
import os
from datetime import datetime

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
        
        # Setup CSV file for logging
        self.setup_csv()

    def setup_csv(self):
        """
        Initializes the CSV file for logging EKF estimates.
        """
        # Define the directory to save the CSV file
        log_dir = os.path.join(os.getcwd(), 'ekf_logs')
        os.makedirs(log_dir, exist_ok=True)  # Create directory if it doesn't exist

        # Create a unique filename with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'ekf_estimates_{timestamp}.csv'
        self.csv_filepath = os.path.join(log_dir, filename)
        
        # Open the CSV file and write the header
        try:
            self.csv_file = open(self.csv_filepath, mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            header = [
                'timestamp',
                'position_x', 'position_y', 'position_z',
                'velocity_x', 'velocity_y', 'velocity_z'
            ]
            self.csv_writer.writerow(header)
            self.get_logger().info(f'Logging EKF estimates to {self.csv_filepath}')
        except Exception as e:
            self.get_logger().error(f'Failed to open CSV file: {e}')
            self.csv_file = None

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
        #self.get_logger().info(f'Acceleration: {self.accel}')

        # Perform prediction step
        self.predict()

    def pos_quat_callback(self, msg):
        # Extract position
        self.pos = np.array([
            msg.pose.position.x, 
            msg.pose.position.y, 
            msg.pose.position.z
        ])
        #self.get_logger().info(f'Noisy pos: {pos}')
        # Extract and normalize orientation quaternion (FLU)
        self.orientation = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        
        # Add Gaussian noise
        noisy_pos = self.pos + np.random.normal(0, 0.1, size=3)
        #self.get_logger().info(f'Noisy pos: {noisy_pos}')
        self.measurement = noisy_pos

        # Perform update step
        if self.measurement is not None:
            self.update()

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

    def predict(self):
        if self.last_time is None:
            self.last_time = self.get_clock().now()
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9  # Convert nanoseconds to seconds
        self.last_time = current_time

        # State transition model
        # x = [px, py, pz, vx, vy, vz]
        # x_k+1 = x_k + dt * v_k + 0.5 * dt^2 * a_k
        # v_k+1 = v_k + dt * a_k

        # Extract current state
        px, py, pz, vx, vy, vz = self.state

        # Update velocities
        vx += self.accel[0] * dt
        vy += self.accel[1] * dt
        vz += self.accel[2] * dt

        # Update positions
        px += vx * dt + 0.5 * self.accel[0] * dt * dt
        py += vy * dt + 0.5 * self.accel[1] * dt * dt
        pz += vz * dt + 0.5 * self.accel[2] * dt * dt

        # Update state vector
        self.state = np.array([px, py, pz, vx, vy, vz])

        # State transition matrix F
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        # Process noise covariance Q is already defined

        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q

    def update(self):
        # Measurement model
        # z = H x + measurement noise
        H = np.zeros((3, 6))
        H[0, 0] = 1  # Position x
        H[1, 1] = 1  # Position y
        H[2, 2] = 1  # Position z

        # Measurement residual
        z = self.measurement
        z_pred = H @ self.state
        y = z - z_pred

        # Innovation covariance
        S = H @ self.P @ H.T + self.R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state estimate
        self.state = self.state + K @ y

        # Update covariance estimate
        self.P = (np.eye(6) - K @ H) @ self.P

        # Publish the estimated pose
        self.publish_estimate()
        self.get_logger().info(f'State: {self.state}')

        # Log the estimate to CSV
        self.log_to_csv()

    def publish_estimate(self):
        estimate_msg = PoseStamped()
        estimate_msg.header.stamp = self.get_clock().now().to_msg()
        estimate_msg.header.frame_id = "map"  # Or appropriate frame

        # Set estimated position
        estimate_msg.pose.position.x = self.state[0]
        estimate_msg.pose.position.y = self.state[1]
        estimate_msg.pose.position.z = self.state[2]

        # Set orientation (could be estimated if we include orientation in the state vector)
        # For now, we'll set it to zero
        estimate_msg.pose.orientation.x = 0.0
        estimate_msg.pose.orientation.y = 0.0
        estimate_msg.pose.orientation.z = 0.0
        estimate_msg.pose.orientation.w = 1.0

        self.estimate_pub.publish(estimate_msg)

    def log_to_csv(self):
        """
        Logs the current EKF state to the CSV file.
        """
        if self.csv_file is None:
            return  # If the CSV file wasn't opened successfully, skip logging

        # Get current time in seconds since epoch for timestamp
        timestamp = self.get_clock().now().to_msg()
        # Convert ROS time to Python datetime for better readability
        ros_time = self.get_clock().now().to_msg()
        # Alternatively, use datetime.utcnow()
        # For simplicity, use seconds and nanoseconds
        timestamp_sec = ros_time.sec + ros_time.nanosec * 1e-9

        # Prepare the row data
        row = [
            timestamp_sec,
            self.state[0], self.state[1], self.state[2],
            self.pos[0], self.pos[1], self.pos[2]
        ]

        try:
            self.csv_writer.writerow(row)
            self.csv_file.flush()  # Ensure data is written to disk
        except Exception as e:
            self.get_logger().error(f'Failed to write to CSV file: {e}')

    def destroy_node(self):
        """
        Overrides the destroy_node method to ensure the CSV file is closed properly.
        """
        if hasattr(self, 'csv_file') and self.csv_file is not None:
            try:
                self.csv_file.close()
                self.get_logger().info(f'CSV file {self.csv_filepath} closed.')
            except Exception as e:
                self.get_logger().error(f'Failed to close CSV file: {e}')
        super().destroy_node()

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
