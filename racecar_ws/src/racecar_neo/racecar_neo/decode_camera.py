#!/usr/local/bin/python3

# node for decoding jpg color frames and publishing rgb8 color frames

import rclpy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np
import time

def main(args=None):
	rclpy.init(args=args)
	node = rclpy.create_node('image_decoder')
	bridge = CvBridge()

	# [FUNCTION] Callback function activates when message is recieved thru sub node
	def image_callback(msg):
		try:
			cv_image = np.frombuffer(msg.data, np.uint8)
			cv_image = cv2.imdecode(cv_image, cv2.IMREAD_COLOR)
			cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
			rgb_image_msg = bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')
			decoded_publisher.publish(rgb_image_msg)
		except Exception as e:
			node.get_logger().error(f"Failed to decode image: {e}")

	image_subscriber = node.create_subscription(Image, '/camera', image_callback, 10)
	decoded_publisher = node.create_publisher(Image, '/camera/decoded', 10)

	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass

	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
