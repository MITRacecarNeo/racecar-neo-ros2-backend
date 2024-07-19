#!/usr/bin/python3

import launch
import launch.actions
import launch.substitutions
from launch_ros.actions import Node


def generate_launch_description():
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
	#Node (
		#package='racecar_neo',
		#executable='decode_camera',
		#name='image_decoder'
	#),
    Node(
	    package='racecar_neo',
        executable='imu',
        name='imu_node'
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
