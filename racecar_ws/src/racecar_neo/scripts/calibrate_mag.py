#!/usr/bin/env python3
"""
3D Magnetometer Calibrator for RACECAR
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import MagneticField
import numpy as np
import yaml
from datetime import datetime
import time
import threading
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class MagnetometerCalibrator(Node):
    def __init__(self):
        super().__init__('magnetometer_calibrator')

        self.subscription = None
        self.message_received = threading.Event() # Use a thread-safe event

        # Define a list of QoS profiles to try
        qos_profiles = [
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT),
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE),
        ]

        for i, profile in enumerate(qos_profiles):
            try:
                self.subscription = self.create_subscription(
                    MagneticField, '/mag', self.mag_callback, profile)
                self.get_logger().info(f"Subscribed to /mag with QoS profile {i+1}.")
                break
            except Exception as e:
                self.get_logger().warn(f"QoS profile {i+1} failed: {e}")

        self.mag_data = []
        self.collecting = False
        self.hard_iron_bias = np.zeros(3)
        self.soft_iron_matrix = np.identity(3)

    def mag_callback(self, msg):
        """Callback to collect magnetometer data."""
        if not self.message_received.is_set():
            self.message_received.set() # Signal that we've received the first message
            self.get_logger().info("First message from /mag topic received!")

        if self.collecting:
            self.mag_data.append([
                msg.magnetic_field.x,
                msg.magnetic_field.y,
                msg.magnetic_field.z
            ])

    def wait_for_messages(self, timeout=10.0):
        """Wait for the first message to arrive."""
        self.get_logger().info("Waiting for first message from /mag topic...")
        return self.message_received.wait(timeout)

    def run_calibration_procedure(self):
        """Main calibration logic."""
        if not self.wait_for_messages():
            self.get_logger().error("No messages received from /mag topic. Aborting.")
            self.get_logger().error("Troubleshooting steps:")
            self.get_logger().error("1. Make sure imu_node is running.")
            self.get_logger().error("2. Run 'ros2 topic echo /mag' to check for data.")
            return

        self.get_logger().info('=' * 60)
        self.get_logger().info('RACECAR 3D Magnetometer Calibration (Structured Method)')
        self.get_logger().info('=' * 60)
        
        try:
            self.get_logger().info("\n--- Step 1: Yaw Axis (Turn like a car) ---")
            input("Press Enter to begin...")
            self.collect_data_for_step(15, "Yaw Axis")

            self.get_logger().info("\n--- Step 2: Roll Axis (Barrel Roll) ---")
            input("Press Enter to begin...")
            self.collect_data_for_step(15, "Roll Axis")

            self.get_logger().info("\n--- Step 3: Pitch Axis (Nose up/down) ---")
            input("Press Enter to begin...")
            self.collect_data_for_step(15, "Pitch Axis")
            
            self.get_logger().info('\nAll data collected. Calculating parameters...')
            self.calculate_calibration()
            self.save_calibration_file()
        except KeyboardInterrupt:
            self.get_logger().info('Calibration interrupted.')
        except Exception as e:
            self.get_logger().error(f'An error occurred: {str(e)}')

    def collect_data_for_step(self, duration, description):
        """Collects data for one specific rotational step."""
        self.collecting = True
        self.get_logger().info(f'Collecting data for {description} ({duration}s)... Please start rotating now.')
        time.sleep(duration)
        self.collecting = False
        self.get_logger().info(f'Step complete. Current sample count: {len(self.mag_data)}')

    def calculate_calibration(self):
        """
        Calculates hard and soft iron biases using ellipsoid fitting.
        """
        if len(self.mag_data) < 100:
            self.get_logger().error("Not enough data points to perform calibration. Try rotating more.")
            return

        data = np.array(self.mag_data)
        
        # --- Normalization Step ---
        norms = np.linalg.norm(data, axis=1)
        avg_norm = np.mean(norms)
        data_normalized = data / avg_norm
        self.get_logger().info(f"Data normalized with average field strength: {avg_norm:.4e} T")

        # --- Ellipsoid Fitting on normalized data ---
        D = np.zeros((data_normalized.shape[0], 9))
        D[:, 0] = data_normalized[:, 0] * 2
        D[:, 1] = data_normalized[:, 1] * 2
        D[:, 2] = data_normalized[:, 2] * 2
        D[:, 3] = data_normalized[:, 0]**2
        D[:, 4] = data_normalized[:, 1]**2
        D[:, 5] = data_normalized[:, 2]**2
        D[:, 6] = 2 * data_normalized[:, 0] * data_normalized[:, 1]
        D[:, 7] = 2 * data_normalized[:, 0] * data_normalized[:, 2]
        D[:, 8] = 2 * data_normalized[:, 1] * data_normalized[:, 2]
        
        v = np.ones(data_normalized.shape[0])
        (p, _, _, _) = np.linalg.lstsq(D, v, rcond=None)
        
        A = np.array([
            [p[3], p[6], p[7]],
            [p[6], p[4], p[8]],
            [p[7], p[8], p[5]]
        ])
        
        b = np.array([p[0], p[1], p[2]])
        hard_iron_bias_normalized = -np.linalg.inv(A) @ b
        
        # De-normalize the hard-iron bias
        self.hard_iron_bias = hard_iron_bias_normalized * avg_norm
        
        # Calculate the final soft-iron matrix
        evals, evecs = np.linalg.eig(A)
        self.soft_iron_matrix = evecs @ np.sqrt(np.diag(np.abs(evals))) @ evecs.T

        # --- Apply correction to the original data for plotting ---
        data_corrected = np.zeros_like(data)
        for i in range(len(data)):
            data_corrected[i, :] = self.soft_iron_matrix @ (data[i, :] - self.hard_iron_bias)
        
        # --- Plot the results ---
        self.plot_results(data, data_corrected)
        
    def plot_results(self, raw_data, corrected_data):
        """
        Generates 3D scatter plots for raw and corrected magnetometer data.
        """
        self.get_logger().info('Generating plots for visualization...')

        # Plot 1: Uncorrected (Raw) Data
        fig1 = plt.figure(figsize=(10, 8))
        ax1 = fig1.add_subplot(111, projection='3d')
        ax1.scatter(raw_data[:, 0], raw_data[:, 1], raw_data[:, 2], c='r', marker='.', label='Uncorrected Data')
        ax1.set_xlabel('X-axis')
        ax1.set_ylabel('Y-axis')
        ax1.set_zlabel('Z-axis')
        ax1.set_title('Uncorrected Magnetometer Data (Ellipsoid)')
        ax1.legend()
        ax1.axis('auto')

        # Plot 2: Corrected Data
        fig2 = plt.figure(figsize=(10, 8))
        ax2 = fig2.add_subplot(111, projection='3d')
        ax2.scatter(corrected_data[:, 0], corrected_data[:, 1], corrected_data[:, 2], c='b', marker='.', label='Corrected Data')
        ax2.set_xlabel('X-axis')
        ax2.set_ylabel('Y-axis')
        ax2.set_zlabel('Z-axis')
        ax2.set_title('Corrected Magnetometer Data (Sphere)')
        ax2.legend()
        ax2.axis('auto')
        
        self.get_logger().info("Displaying plots. Close the plot windows to exit the script.")
        plt.show()

    def save_calibration_file(self):
        """Saves the calibration data to a YAML file."""
        filename = f'../config/lsm9ds1_mag_cal.yaml'
        calibration_data = {
            'magnetometer.hard_iron_bias': self.hard_iron_bias.tolist(),
            'magnetometer.soft_iron_matrix': {
                'rows': 3,
                'columns': 3,
                'data': self.soft_iron_matrix.flatten().tolist()
            }
        }
        ros2_yaml_format = {'imu_node': {'ros__parameters': calibration_data}}
        
        with open(filename, 'w') as f:
            yaml.dump(ros2_yaml_format, f, default_flow_style=False, sort_keys=False)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('CALIBRATION COMPLETE')
        self.get_logger().info(f'Data saved to: {filename}')

def main(args=None):
    rclpy.init(args=args)
    calibrator = MagnetometerCalibrator()
    calibration_thread = threading.Thread(target=calibrator.run_calibration_procedure, daemon=True)
    calibration_thread.start()
    try:
        # Spin the node to process callbacks
        while calibration_thread.is_alive() and rclpy.ok():
            rclpy.spin_once(calibrator, timeout_sec=0.1)
        calibration_thread.join()
    except KeyboardInterrupt:
        calibrator.get_logger().info('Shutting down calibrator node.')
    finally:
        # Check if node exists and is valid before destroying
        if calibrator.subscription is not None:
            calibrator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
