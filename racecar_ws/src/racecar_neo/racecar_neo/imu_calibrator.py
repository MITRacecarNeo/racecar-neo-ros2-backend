#!/usr/bin/env python3
"""
LSM9DS1 IMU Calibrator for ROS2
Author: Koneshka Dey
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu
import numpy as np
import yaml
from datetime import datetime
import threading
import time
import queue

class IMUCalibrator(Node):
    def __init__(self):
        super().__init__('imu_calibrator')
        
        # Create multiple QoS profiles to try different compatibility options (needed cause /IMU wasn't being detected for some reason)
        self.qos_profiles = [
            QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST
            ),
            # Reliable(default)
            QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST
            ),
            QoSProfile(depth=10)
        ]
        self.subscription = None
        self.create_subscription_with_qos()

        self.accel_queue = queue.Queue()
        self.gyro_queue = queue.Queue()
        self.collecting = False
        self.message_received = False
        
        # Calibration results
        self.gyro_bias = None
        self.accel_bias = None
        
        self.get_logger().info('IMU Calibrator initialized. Trying to connect to /imu...')
        
    def create_subscription_with_qos(self):
        """Try different QoS profiles to match publisher"""
        for i, qos in enumerate(self.qos_profiles):
            try:
                if self.subscription:
                    self.destroy_subscription(self.subscription)
                
                self.subscription = self.create_subscription(
                    Imu,
                    '/imu',
                    self.imu_callback,
                    qos
                )
                self.get_logger().info(f'Created subscription with QoS profile {i+1}')
                break
            except Exception as e:
                self.get_logger().warn(f'QoS profile {i+1} failed: {e}')
                continue
        
    def imu_callback(self, msg):
        """Callback for IMU messages"""
        if not self.message_received:
            self.message_received = True
            self.get_logger().info('First IMU message received successfully!')
        
        if self.collecting:
            accel = [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ]
            
            gyro = [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ]
            
            self.accel_queue.put(accel)
            self.gyro_queue.put(gyro)
    
    def wait_for_messages(self, timeout=15.0):
        """Wait for IMU messages with better detection"""
        self.get_logger().info('Checking for IMU messages...')
        
        start_time = time.time()
        while not self.message_received and (time.time() - start_time) < timeout:
            time.sleep(0.1)
            
            # Show progress every 3 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 3 == 0 and int(elapsed) > 0:
                remaining = timeout - elapsed
                self.get_logger().info(f'Still waiting for messages... {remaining:.1f}s remaining')
        
        return self.message_received
    
    def collect_data(self, duration, description):
        """Collect IMU data for specified duration"""
        self.get_logger().info(f'Starting {description} - Duration: {duration}s')
        
        # Clear queues
        while not self.accel_queue.empty():
            self.accel_queue.get()
        while not self.gyro_queue.empty():
            self.gyro_queue.get()
        
        # Start collecting
        self.collecting = True
        
        start_time = time.time()
        last_update = 0
        
        while (time.time() - start_time) < duration:
            time.sleep(0.1)
            
            # Show progress every 2 seconds
            elapsed = time.time() - start_time
            if int(elapsed) != last_update and int(elapsed) % 2 == 0:
                remaining = duration - elapsed
                samples = self.accel_queue.qsize()
                self.get_logger().info(f'{description}: {remaining:.1f}s remaining, {samples} samples')
                last_update = int(elapsed)
        
        # Stop collecting
        self.collecting = False
        
        # Extract data from queues
        accel_data = []
        gyro_data = []
        
        while not self.accel_queue.empty():
            accel_data.append(self.accel_queue.get())
        while not self.gyro_queue.empty():
            gyro_data.append(self.gyro_queue.get())
        
        self.get_logger().info(f'{description} complete. Collected {len(accel_data)} samples')
        return accel_data, gyro_data
    
    def calculate_bias(self, data):
        """Calculate bias from collected data"""
        if not data:
            return [0.0, 0.0, 0.0]
        
        data_array = np.array(data)
        return data_array.mean(axis=0).tolist()
    
    def calculate_accel_bias_6pos(self, all_accel_data):
        """Calculate accelerometer bias from 6-position calibration"""
        combined_data = []
        for pos_data in all_accel_data:
            combined_data.extend(pos_data)
        
        if not combined_data:
            return [0.0, 0.0, 0.0]
        
        data_array = np.array(combined_data)
        bias = data_array.mean(axis=0)
        return bias.tolist()
    
    def run_calibration(self):
        """Run the complete calibration sequence"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('LSM9DS1 IMU CALIBRATION STARTING')
        self.get_logger().info('=' * 50)
        
        # Wait for IMU messages with better timeout
        if not self.wait_for_messages(timeout=15.0):
            self.get_logger().error('No IMU messages received!')
            self.get_logger().error('Troubleshooting steps:')
            self.get_logger().error('1. Check: ros2 topic list | grep imu')
            self.get_logger().error('2. Check: ros2 topic info /imu -v')
            self.get_logger().error('3. Check: ros2 topic echo /imu --qos-reliability best_effort')
            return
        
        try:
            # Step 1: Gyroscope Calibration
            self.get_logger().info('\n1. GYROSCOPE CALIBRATION')
            self.get_logger().info('Place IMU on stable surface - DO NOT MOVE')
            input('Press Enter to start gyroscope calibration...')
            
            _, gyro_data = self.collect_data(10, 'Gyroscope calibration')
            if not gyro_data:
                self.get_logger().error('No gyroscope data collected!')
                return
            self.gyro_bias = self.calculate_bias(gyro_data)
            
            # Step 2: Accelerometer Calibration
            self.get_logger().info('\n2. ACCELEROMETER CALIBRATION')
            self.get_logger().info('Place IMU in 6 different orientations')
            # Going through the dimensions 
            positions = [
                ('X+', 'X-axis pointing up (right side up)'),
                ('X-', 'X-axis pointing down (left side up)'),
                ('Y+', 'Y-axis pointing up (front edge up)'),
                ('Y-', 'Y-axis pointing down (back edge up)'),
                ('Z+', 'Z-axis pointing up (top face up)'),
                ('Z-', 'Z-axis pointing down (bottom face up)')
            ]
            
            all_accel_data = []
            for i, (axis, description) in enumerate(positions):
                self.get_logger().info(f'\nPosition {i+1}/6: {axis}')
                self.get_logger().info(f'Description: {description}')
                input('Press Enter when IMU is in position...')
                
                accel_data, _ = self.collect_data(5, f'Position {axis}')
                if not accel_data:
                    self.get_logger().error(f'No data collected for position {axis}!')
                    return
                all_accel_data.append(accel_data)
            
            self.accel_bias = self.calculate_accel_bias_6pos(all_accel_data)
            
            # Generate calibration results
            self.generate_calibration_yaml()
            
        except KeyboardInterrupt:
            self.get_logger().info('\nCalibration interrupted by user')
        except Exception as e:
            self.get_logger().error(f'Calibration failed: {str(e)}')
    
    def generate_calibration_yaml(self):
        """Generate YAML calibration file"""
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        identity_matrix = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ]
        
        calibration_data = {
            'imu_model': 'lsm9ds1',
            'calibration_date': timestamp,
            'ros2_topic': '/imu',
            
            'accelerometer': {
                'bias': self.accel_bias,
                'scale_matrix': identity_matrix,
                'noise_density': 1.86e-03,
                'random_walk': 4.33e-04
            },
            
            'gyroscope': {
                'bias': self.gyro_bias,
                'scale_matrix': identity_matrix,
                'noise_density': 1.87e-04,
                'random_walk': 2.66e-05
            },
            
            'update_rate': 100.0,
            'temperature_bias': 0.0,
            'temperature_scale': 1.0
        }
        
        filename = f'lsm9ds1_calibration_{datetime.now().strftime("%Y%m%d_%H%M%S")}.yaml' #trying to dump info lol
        
        try:
            with open(filename, 'w') as f:
                yaml.dump(calibration_data, f, default_flow_style=False, indent=2)
            
            self.get_logger().info('\n' + '=' * 60)
            self.get_logger().info('CALIBRATION COMPLETED')
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'Calibration saved to: {filename}')
            
            self.get_logger().info('\nCalibration Results:')
            self.get_logger().info('-' * 40)
            self.get_logger().info(f'Gyroscope Bias (rad/s): {self.gyro_bias}')
            self.get_logger().info(f'Accelerometer Bias (m/s²): {self.accel_bias}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to save calibration file: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    calibrator = IMUCalibrator()
    executor = SingleThreadedExecutor()
    executor.add_node(calibrator)
    
    #Run calibration in separate thread
    calibration_thread = threading.Thread(target=calibrator.run_calibration, daemon=True)
    calibration_thread.start()
    
    try:
        #Spin the executor in main thread
        while calibration_thread.is_alive() and rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
        
        calibration_thread.join(timeout=1.0)
        
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        calibrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
