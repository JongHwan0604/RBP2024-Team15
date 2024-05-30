import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from glob import glob

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header

class DetermineColor(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.image_sub = self.create_subscription(Image, '/color', self.callback, 10)
        self.color_pub = self.create_publisher(Header, '/rotate_cmd', 10)
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            # Convert the ROS image message to OpenCV format
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # Prepare the rotate_cmd message
            msg = Header()
            msg.stamp = data.header.stamp
            msg.frame_id = '0'  # Default: STOP

            # Determine the dominant color
            dominant_color = self.determine_dominant_color(image)
            if dominant_color == 'G':
                msg.frame_id = '0'  # Example action: STOP for green
            elif dominant_color == 'B':
                msg.frame_id = '1'  # Example action: STOP for black

            # Publish the rotate_cmd message
            self.color_pub.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))

    def determine_dominant_color(self, image):
        img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges in HSV
        green_mask = cv2.inRange(img_hsv, (36, 25, 25), (70, 255, 255))
        black_mask = cv2.inRange(img_hsv, (0, 0, 0), (180, 255, 30))

        green_pixels = cv2.countNonZero(green_mask)
        black_pixels = cv2.countNonZero(black_mask)

        # Determine the dominant color
        if green_pixels > black_pixels:
            return 'G'
        elif black_pixels > 0:
            return 'B'
        else:
            return 'None'

if __name__ == '__main__':
    rclpy.init()
    detector = DetermineColor()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

