#!/usr/bin/python3

import os
import launch
import launch.actions
import launch.substitutions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
            get_package_share_directory('racecar_neo'),
            'config',
            'lsm9ds1_cal.yaml')

    return launch.LaunchDescription([
	Node(
	    package='joy',
        executable='joy_node',
        name='joy_node',
	),
    Node(
	    package='racecar_neo',
        executable='gamepad',
        name='gamepad_node',
	),
	Node(
	    package='racecar_neo',
        executable='mux',
        name='mux_node'
	),
	Node(
	    package='racecar_neo',
        executable='throttle',
        name='throttle_node'
	),
	Node(
	    package='racecar_neo',
        executable='pwm',
        name='pwm_node'
	),
    Node(
	    package='racecar_neo',
        executable='camera',
        name='camera_node'
	),
    # uncomment decode_camera for rviz-compatible image
	#Node (
		#package='racecar_neo',
		#executable='decode_camera',
		#name='image_decoder'
	#),
    Node(
	    package='racecar_neo',
        executable='imu',
        name='imu_node',
	    parameters=[config],
    ),
	Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':'serial',
                         'serial_port': '/dev/ttyUSB0', 
                         'serial_baudrate':115200, 
                         'frame_id': 'laser',
                         'inverted':False, 
                         'angle_compensate':True}],
            output='screen'
    ),
    ])
