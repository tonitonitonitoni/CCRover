import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from cv_bridge import CvBridge

class LEDLineFollower(Node):
    def __init__(self):
        super().__init__('led_line_follower')
        self.bridge = CvBridge()
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Replace with your camera topic
            self.image_callback,
            10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10) # Replace with your robot's cmd_vel topic
        self.declare_parameters(
            namespace='',
            parameters=[
                ('hsv_lower_hue', 100), # Example values, tune for your LEDs
                ('hsv_lower_saturation', 100),
                ('hsv_lower_value', 100),
                ('hsv_upper_hue', 130),
                ('hsv_upper_saturation', 255),
                ('hsv_upper_value', 255),
                ('speed', 0.1),
                ('max_turn', 0.5)
            ])
        self.hsv_lower = np.array([
            self.get_parameter('hsv_lower_hue').value,
            self.get_parameter('hsv_lower_saturation').value,
            self.get_parameter('hsv_lower_value').value
            ])
        self.hsv_upper = np.array([
            self.get_parameter('hsv_upper_hue').value,
            self.get_parameter('hsv_upper_saturation').value,
            self.get_parameter('hsv_upper_value').value
            ])
        self.speed = self.get_parameter('speed').value
        self.max_turn = self.get_parameter('max_turn').value

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour (assuming it's the LED line)
            largest_contour = max(contours, key=cv2.contourArea)

            # Calculate the centroid of the contour
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # Calculate the error (deviation from the center of the image)
                error = cX - cv_image.shape[1] / 2

                # Control the robot
                twist = Twist()
                twist.linear.x = self.speed
                twist.angular.z = -float(error) / (cv_image.shape[1] / 2) * self.max_turn # Normalize error and scale by max turn
                self.velocity_publisher.publish(twist)
            else:
                self.get_logger().warn("No LED detected")
                twist = Twist()
                self.velocity_publisher.publish(twist) #stop if no LED is detected
        else:
            self.get_logger().warn("No LED detected")
            twist = Twist()
            self.velocity_publisher.publish(twist) #stop if no LED is detected


def main(args=None):
    rclpy.init(args=args)
    led_line_follower = LEDLineFollower()
    rclpy.spin(led_line_follower)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
