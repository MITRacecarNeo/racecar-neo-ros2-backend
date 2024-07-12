#!/usr/local/bin/python3

# node for ensuring commanded speed does not exceed throttle

# stdlib imports
import sys

# import ros libraries
import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile

from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

# get param file values
CAR_THROTTLE_FORWARD = float(0.0425)  # rclpy.parameter.Parameter('car_throttle_forward',type_=DOUBLE, 0.25)
CAR_THROTTLE_BACKWARD = float(0.06)  # rclpy.param.Parameter('car_throttle_backward',type_=DOUBLE, 0.25)
CAR_THROTTLE_TURN = float(0.25)  # rclpy.param.Parameter('car_throttle_turn',type_=DOUBLE, 1.0)

DRIVE_MAX_SPEED = float(0.25)


def main(args=None):
    # init ROS
    rclpy.init(args=args)
    node = rclpy.create_node('throttle_node')

    try:
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        pub = node.create_publisher(AckermannDriveStamped, '/motor', qos_profile)

        # callback that throttles / scales max speed and angle
        def drive_callback(msg):
            # Messages sent by `/drive` and republished in `/mux_out` have max `drive.speed` of `DRIVE_MAX_SPEED`.
            # Scale this back up to [-1.0, 1.0], then multiply by throttle to get full range of motion.
            msg.drive.speed /= DRIVE_MAX_SPEED
            msg.drive.speed = max(min(msg.drive.speed, 1.0), -1.0)

            # Scale by `THROTTLE_FORWARD` on positive speed, `THROTTLE_BACKWARD` on negative speed
            if msg.drive.speed > 0:
                msg.drive.speed *= CAR_THROTTLE_FORWARD

            if msg.drive.speed < 0:
                msg.drive.speed *= CAR_THROTTLE_BACKWARD

            # `drive.steering_angle` is already within [-1.0, 1.0], so just multiply by throttle for
            # full range of motion.
            msg.drive.steering_angle = max(min(msg.drive.steering_angle, 1.0), -1.0)
            msg.drive.steering_angle *= CAR_THROTTLE_TURN

            pub.publish(msg)

        node.create_subscription(AckermannDriveStamped, '/mux_out', drive_callback, qos_profile)

        # wait before shutdown
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
