#!/usr/local/bin/python3

# node for natively publishing RGB color frames from Logitech BRIO 4k camera

import rclpy
from sensor_msgs.msg import Image

import cv2
import numpy as np
import time

def main(args=None):
    
    # init ROS
    rclpy.init(args=args)
    node = rclpy.create_node('camera_node')
    pub_cam = node.create_publisher(Image,'/camera', 1)
    print("Topics set up!")

    ##############################################
    ### Set up the camera, or quit if we can't ###
    ##############################################

    # Loop through camera IDs until 9 to find all potential cams
    # Timeout of 3s per camera ID (the 3s is arbitrary)
    final_cam_id = None
    final_cap = None
    for curr_cam_id in range(10):
        # Start the timer
        start_time = time.time()

        # Check a bunch of times
        while time.time() - start_time < 3:
            # Get the capture device and read a frame
            curr_cap = cv2.VideoCapture(curr_cam_id)
            got_frame, frame = curr_cap.read()

            # We could not connect, so release and try again
            if not got_frame:
                node.get_logger().warn(f"Webcam at /dev/video{curr_cam_id} does not exist, trying again!!!")
                curr_cap.release()

            # We could connect, do not release and break the while loop
            if got_frame:
                node.get_logger().info(f"Webcam found at /dev/video{curr_cam_id}. Camera Node Successful!")
                final_cam_id = curr_cam_id
                final_cap = curr_cap
                break

        # Break the for loop if we are connected
        if final_cam_id is not None:
            break

    # We could not connect, so give up, shutdown, and quit the method
    if final_cam_id is None:
        node.get_logger().warn("Camera node shutting down!")
        rclpy.shutdown()
        return

    ################################################
    ### Running loop for when we have the camera ###
    ################################################

    # Get frames, send frames through published thread 
    try:
        while rclpy.ok():        
            got_frame, frame = final_cap.read()
            
            if got_frame:
                color_image = frame

                # Resize Frame (50%)
                # color_image = cv2.resize(color_image, (320, 240))
                # BGR TO RGB
                color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

                # Create Image ROS2 message
                msg = Image()
                msg.height = color_image.shape[0]
                msg.width = color_image.shape[1]
                msg.encoding = 'rgb8'
                msg.is_bigendian = 0
                msg.step = 3 * msg.width
                msg.data = np.array(color_image).tostring()
            
                # Publish color image
                pub_cam.publish(msg)
            else:
                node.get_logger().warn("Frame not found! Is the camera unplugged?", throttle_duration_sec=1.0)
    except KeyboardInterrupt as e:
        # Shutting down, release resource and ros shutdown
        final_cap.release()
        node.get_logger().warn("Camera cleanly exited.")
        rclpy.shutdown()

if __name__ == '__main__':
    main()

