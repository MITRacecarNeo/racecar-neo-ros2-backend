#!/usr/local/bin/python3

# node for processing data from LSM9DS1 IMU through smbus I2C bus and publishing to IMU node

import smbus
import time
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header

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
    
    OUT_X_L_G = 0x18
    OUT_X_H_G = 0x19
    OUT_Y_L_G = 0x1A
    OUT_Y_H_G = 0x1B
    OUT_Z_L_G = 0x1C
    OUT_Z_H_G = 0x1D

    OUT_X_L_XL = 0x28
    OUT_X_H_XL = 0x29
    OUT_Y_L_XL = 0x2A
    OUT_Y_H_XL = 0x2B
    OUT_Z_L_XL = 0x2C
    OUT_Z_H_XL = 0x2D

    # --- MAG REGISTER ADDRESSES ---
    CTRL_REG1_M = 0x20
    CTRL_REG2_M = 0x21
    CTRL_REG3_M = 0x22
    CTRL_REG4_M = 0x23

    OUT_X_L_M = 0x28
    OUT_X_H_M = 0x29
    OUT_Y_L_M = 0x2A
    OUT_Y_H_M = 0x2B
    OUT_Z_L_M = 0x2C
    OUT_Z_H_M = 0x2D

    # enable gyroscope
    bus.write_byte_data(ACCEL_GYRO_ADDR, CTRL_REG1_G, 0b11000011)
    
    # enable accelerometer
    bus.write_byte_data(ACCEL_GYRO_ADDR, CTRL_REG6_XL, 0b11000110)
    
    # enable magnetometer
    bus.write_byte_data(MAG_ADDR, CTRL_REG1_M, 0b11111100)
    bus.write_byte_data(MAG_ADDR, CTRL_REG2_M, 0b00000000)
    bus.write_byte_data(MAG_ADDR, CTRL_REG3_M, 0b00000000)
    bus.write_byte_data(MAG_ADDR, CTRL_REG4_M, 0b00001100)
    
    # init ROS
    rclpy.init(args=args)
    node = rclpy.create_node('imu_node')

    # --- CALIBRATION PARAMS ---
    if not node.has_parameter('accelerometer.bias'):
        node.declare_parameter('accelerometer.bias', [0.0, 0.0, 0.0])
    if not node.has_parameter('gyroscope.bias'):
        node.declare_parameter('gyroscope.bias', [0.0, 0.0, 0.0])

    accel_bias = node.get_parameter('accelerometer.bias').value
    gyro_bias = node.get_parameter('gyroscope.bias').value

    # log to ros display
    node.get_logger().info(f"Aceelerometer bias (m/s^2): {accel_bias}")
    node.get_logger().info(f"Gyroscope bias (rad/s): {gyro_bias}")

    # finish the rest of ros setup
    pub_imu = node.create_publisher(Imu, '/imu', 10)
    pub_mag = node.create_publisher(MagneticField, '/mag', 10)

    node.get_logger().info("IMU Node setup finished! Check calibration params.")

    # Get frames, send frames through published thread 
    while rclpy.ok():
        current_time = node.get_clock().now().to_msg()
        
        # --- Read Gyroscope Data ---
        out_x_g_l = bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_X_L_G)
        out_x_g_h = bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_X_H_G)
        out_x_g = twos_complement((out_x_g_h << 8) | out_x_g_l, 16) * SENSITIVITY_GYROSCOPE_245
        ############################
        out_y_g_l = bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_Y_L_G)
        out_y_g_h = bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_Y_H_G)
        out_y_g = twos_complement((out_y_g_h << 8) | out_y_g_l, 16) * SENSITIVITY_GYROSCOPE_245
        ############################
        out_z_g_l = bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_Z_L_G)
        out_z_g_h = bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_Z_H_G)
        out_z_g = twos_complement((out_z_g_h << 8) | out_z_g_l, 16) * SENSITIVITY_GYROSCOPE_245

        # Convert gyro data from dps to rad/s
        out_x_g = round(out_x_g * (math.pi/180), 10)
        out_y_g = round(out_y_g * (math.pi/180), 10)
        out_z_g = round(out_z_g * (math.pi/180), 10)

        # Subtract biases from gyro data
        out_x_g -= gyro_bias[0]
        out_y_g -= gyro_bias[1]
        out_z_g -= gyro_bias[2]

        # --- Read Accelerometer Data ---
        out_x_xl_l = bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_X_L_XL)
        out_x_xl_h = bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_X_H_XL)
        out_x_xl = twos_complement((out_x_xl_h << 8) | out_x_xl_l, 16) * SENSITIVITY_ACCELEROMETER_2
        ############################
        out_y_xl_l = bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_Y_L_XL)
        out_y_xl_h = bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_Y_H_XL)
        out_y_xl = twos_complement((out_y_xl_h << 8) | out_y_xl_l, 16) * SENSITIVITY_ACCELEROMETER_2
        ############################
        out_z_xl_l = bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_Z_L_XL)
        out_z_xl_h = bus.read_byte_data(ACCEL_GYRO_ADDR, OUT_Z_H_XL)
        out_z_xl = twos_complement((out_z_xl_h << 8) | out_z_xl_l, 16) * SENSITIVITY_ACCELEROMETER_2

        # Convert accel data from g's to m/s^2
        out_x_xl = round(out_x_xl * 9.80665, 10)
        out_y_xl = round(out_y_xl * 9.80665, 10)
        out_z_xl = round(out_z_xl * 9.80665, 10)
        
        # Subtract biases from accel data
        out_x_xl -= accel_bias[0]
        out_y_xl -= accel_bias[1]
        out_z_xl -= accel_bias[2]

        # --- Read Magnetometer Data ---
        out_x_m_l = bus.read_byte_data(MAG_ADDR, OUT_X_L_M)
        out_x_m_h = bus.read_byte_data(MAG_ADDR, OUT_X_H_M)
        out_x_m = twos_complement((out_x_m_h << 8) | out_x_m_l, 16) * SENSITIVITY_MAGNETOMETER_4
        ############################
        out_y_m_l = bus.read_byte_data(MAG_ADDR, OUT_Y_L_M)
        out_y_m_h = bus.read_byte_data(MAG_ADDR, OUT_Y_H_M)
        out_y_m = twos_complement((out_y_m_h << 8) | out_y_m_l, 16) * SENSITIVITY_MAGNETOMETER_4
        ############################
        out_z_m_l = bus.read_byte_data(MAG_ADDR, OUT_Z_L_M)
        out_z_m_h = bus.read_byte_data(MAG_ADDR, OUT_Z_H_M)
        out_z_m = twos_complement((out_z_m_h << 8) | out_z_m_l, 16) * SENSITIVITY_MAGNETOMETER_4

        # Convert magnetometer data from Gauss to Tesla for the MagneticField message
        out_x_m = out_x_m * 1e-4 
        out_y_m = out_y_m * 1e-4
        out_z_m = out_z_m * 1e-4

        # --- Populate and Publish Imu Message ---
        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = 'imu_link' 
        imu_msg.angular_velocity.x = out_x_g
        imu_msg.angular_velocity.y = out_y_g
        imu_msg.angular_velocity.z = out_z_g
        imu_msg.linear_acceleration.x = out_x_xl
        imu_msg.linear_acceleration.y = out_y_xl
        imu_msg.linear_acceleration.z = out_z_xl
    
        # --- Populate and Publish MagneticField Message ---
        mag_msg = MagneticField()
        mag_msg.header.stamp = current_time
        mag_msg.header.frame_id = 'imu_link' 
        mag_msg.magnetic_field.x = out_x_m
        mag_msg.magnetic_field.y = out_y_m
        mag_msg.magnetic_field.z = out_z_m
        mag_msg.magnetic_field_covariance = [0.0] * 9 

        pub_imu.publish(imu_msg)
        pub_mag.publish(mag_msg)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
