#!/usr/local/bin/python3

# node for processing data from LSM9DS1 IMU through smbus I2C bus and publishing to IMU node

import smbus
import time
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
import numpy as np

def main(args=None):

    bus = smbus.SMBus(1)

    def twos_complement(val, bits):
        if (val & (1 << (bits - 1))) != 0:
            val = val - (1 << bits)
        return val

    # --- SENSITIVITY CONSTANT FROM LSM9DWS1 DATASHEET ---
    SENSITIVITY_ACCELEROMETER_2 = 0.000061 * -1 # negate accel readings
    SENSITIVITY_GYROSCOPE_245 = 0.00875
    SENSITIVITY_MAGNETOMETER_4 = 0.00014
    
    # --- DEVICE ADDRESSES ---
    ACCEL_GYRO_ADDR = 0x6B
    MAG_ADDR = 0x1E

    # --- ACCEL/GYRO REGISTER ADDRESSES ---
    CTRL_REG1_G = 0x10
    CTRL_REG6_XL = 0x20
    OUT_X_L_G, OUT_X_H_G = 0x18, 0x19
    OUT_Y_L_G, OUT_Y_H_G = 0x1A, 0x1B
    OUT_Z_L_G, OUT_Z_H_G = 0x1C, 0x1D
    OUT_X_L_XL, OUT_X_H_XL = 0x28, 0x29
    OUT_Y_L_XL, OUT_Y_H_XL = 0x2A, 0x2B
    OUT_Z_L_XL, OUT_Z_H_XL = 0x2C, 0x2D

    # --- MAG REGISTER ADDRESSES ---
    CTRL_REG1_M, CTRL_REG2_M, CTRL_REG3_M, CTRL_REG4_M = 0x20, 0x21, 0x22, 0x23
    OUT_X_L_M, OUT_X_H_M = 0x28, 0x29
    OUT_Y_L_M, OUT_Y_H_M = 0x2A, 0x2B
    OUT_Z_L_M, OUT_Z_H_M = 0x2C, 0x2D

    # --- Enable sensors ---
    bus.write_byte_data(ACCEL_GYRO_ADDR, CTRL_REG1_G, 0b11000011) # gyro
    bus.write_byte_data(ACCEL_GYRO_ADDR, CTRL_REG6_XL, 0b11000110) # accelerometer
    bus.write_byte_data(MAG_ADDR, CTRL_REG1_M, 0b11111100) # magnetometer
    bus.write_byte_data(MAG_ADDR, CTRL_REG2_M, 0b00000000)
    bus.write_byte_data(MAG_ADDR, CTRL_REG3_M, 0b00000000)
    bus.write_byte_data(MAG_ADDR, CTRL_REG4_M, 0b00001100)
    
    # --- ROS Init ---
    rclpy.init(args=args)
    node = rclpy.create_node('imu_node')

    # --- CALIBRATION PARAMS ---
    # Declare and load accel/gyro/mag parameters
    if not node.has_parameter('accelerometer.bias'):
        node.declare_parameter('accelerometer.bias', [0.0, 0.0, 0.0])
    if not node.has_parameter('gyroscope.bias'):
        node.declare_parameter('gyroscope.bias', [0.0, 0.0, 0.0])
    if not node.has_parameter('magnetometer.hard_iron_bias'):
        node.declare_parameter('magnetometer.hard_iron_bias', [0.0, 0.0, 0.0])
    if not node.has_parameter('magnetometer.soft_iron_matrix'):
        node.declare_parameter('magnetometer.soft_iron_matrix.data', np.identity(3).flatten().tolist())
    accel_bias = node.get_parameter('accelerometer.bias').value
    gyro_bias = node.get_parameter('gyroscope.bias').value

    hard_iron_list = node.get_parameter('magnetometer.hard_iron_bias').get_parameter_value().double_array_value
    soft_iron_flat_list = node.get_parameter('magnetometer.soft_iron_matrix.data').get_parameter_value().double_array_value
    
    mag_hard_iron_bias = np.array(hard_iron_list)
    mag_soft_iron_matrix = np.array(soft_iron_flat_list).reshape((3, 3))

    # Log parameters to ROS display
    node.get_logger().info(f"Accelerometer bias (m/s^2): {accel_bias}")
    node.get_logger().info(f"Gyroscope bias (rad/s): {gyro_bias}")
    node.get_logger().info(f"Magnetometer Hard-Iron Bias (T): {mag_hard_iron_bias.tolist()}")
    node.get_logger().info(f"Magnetometer Soft-Iron Matrix: \n{mag_soft_iron_matrix}")

    # --- ROS Publishers ---
    pub_imu = node.create_publisher(Imu, '/imu', 10)
    pub_mag = node.create_publisher(MagneticField, '/mag', 10)

    node.get_logger().info("IMU Node setup finished! Check calibration params.")

    while rclpy.ok():
        current_time = node.get_clock().now().to_msg()
        
        # --- Read Gyroscope Data ---
        out_x_g = twos_complement((bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_X_H_G) << 8) | bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_X_L_G), 16) * SENSITIVITY_GYROSCOPE_245
        out_y_g = twos_complement((bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_Y_H_G) << 8) | bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_Y_L_G), 16) * SENSITIVITY_GYROSCOPE_245
        out_z_g = twos_complement((bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_Z_H_G) << 8) | bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_Z_L_G), 16) * SENSITIVITY_GYROSCOPE_245
        out_x_g, out_y_g, out_z_g = [round(g * (math.pi/180), 10) for g in (out_x_g, out_y_g, out_z_g)]
        out_x_g -= gyro_bias[0]; out_y_g -= gyro_bias[1]; out_z_g -= gyro_bias[2]

        # --- Read Accelerometer Data ---
        out_x_xl = twos_complement((bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_X_H_XL) << 8) | bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_X_L_XL), 16) * SENSITIVITY_ACCELEROMETER_2
        out_y_xl = twos_complement((bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_Y_H_XL) << 8) | bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_Y_L_XL), 16) * SENSITIVITY_ACCELEROMETER_2
        out_z_xl = twos_complement((bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_Z_H_XL) << 8) | bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_Z_L_XL), 16) * SENSITIVITY_ACCELEROMETER_2
        out_x_xl, out_y_xl, out_z_xl = [round(a * 9.80665, 10) for a in (out_x_xl, out_y_xl, out_z_xl)]
        out_x_xl -= accel_bias[0]; out_y_xl -= accel_bias[1]; out_z_xl -= accel_bias[2]

        # --- Read Magnetometer Data ---
        out_x_m = twos_complement((bus.read_byte_data(MAG_ADDR, OUT_X_H_M) << 8) | bus.read_byte_data(MAG_ADDR, OUT_X_L_M), 16) * SENSITIVITY_MAGNETOMETER_4
        out_y_m = twos_complement((bus.read_byte_data(MAG_ADDR, OUT_Y_H_M) << 8) | bus.read_byte_data(MAG_ADDR, OUT_Y_L_M), 16) * SENSITIVITY_MAGNETOMETER_4
        out_z_m = twos_complement((bus.read_byte_data(MAG_ADDR, OUT_Z_H_M) << 8) | bus.read_byte_data(MAG_ADDR, OUT_Z_L_M), 16) * SENSITIVITY_MAGNETOMETER_4
        
        # Convert magnetometer data from Gauss to Tesla
        mag_raw = np.array([out_x_m, out_y_m, out_z_m]) * 1e-4

        # Apply magnetometer calibration
        mag_calibrated = mag_soft_iron_matrix @ (mag_raw - mag_hard_iron_bias)

        # --- Populate and Publish Imu Message ---
        imu_msg = Imu()
        imu_msg.header.stamp, imu_msg.header.frame_id = current_time, 'imu_link'
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = out_x_g, out_y_g, out_z_g
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = out_x_xl, out_y_xl, out_z_xl
    
        # --- Populate and Publish MagneticField Message ---
        mag_msg = MagneticField()
        mag_msg.header.stamp, mag_msg.header.frame_id = current_time, 'imu_link'
        mag_msg.magnetic_field.x = mag_calibrated[0]
        mag_msg.magnetic_field.y = mag_calibrated[1]
        mag_msg.magnetic_field.z = mag_calibrated[2]
        mag_msg.magnetic_field_covariance = [0.0] * 9

        pub_imu.publish(imu_msg)
        pub_mag.publish(mag_msg)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
