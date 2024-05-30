import rclpy
import numpy as np
import cv2

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header

class DetermineColor(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.callback, 10)
        self.color_pub = self.create_publisher(Header, '/rotate_cmd', 10)
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            # Listen to image topic
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # Prepare rotate_cmd msg
            msg = Header()
            msg.stamp = data.header.stamp
            msg.frame_id = '0'  # default: STOP

            try:
                # Convert to HSV color space
                hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                # Define the range for black color in HSV space
                lower_black = np.array([0, 0, 0], dtype=np.uint8)
                upper_black = np.array([15, 50, 50], dtype=np.uint8)

                Rtotal, Gtotal, Btotal = 0, 0, 0

                # Create a copy of the image to draw the boundaries
                visualization_image = image.copy()

                for row_idx, row in enumerate(hsv_image):
                    # Find the positions of black color in the row
                    black_indices = np.where(np.all(np.logical_and(lower_black <= row, row <= upper_black), axis=-1))[0]

                    if len(black_indices) >= 2:
                        left_black_index = black_indices[0]
                        right_black_index = black_indices[-1]

                        # Extract colors within the black boundary
                        inner_colors = row[left_black_index:right_black_index]

                        if inner_colors.size > 0:
                            # Calculate the number of pixels for each color channel
                            Rtotal += np.sum((inner_colors[:, 0] > 170) | (inner_colors[:, 0] < 10))
                            Gtotal += np.sum((inner_colors[:, 0] > 50) & (inner_colors[:, 0] < 70))
                            Btotal += np.sum((inner_colors[:, 0] > 110) & (inner_colors[:, 0] < 130))

                            # Draw the black boundary on the visualization image
                            cv2.line(visualization_image, (left_black_index, row_idx), (right_black_index, row_idx), (0, 255, 0), 1)
                        else:
                            self.get_logger().warn('Inner colors array is empty. Possible error in black boundary detection.')
                    else:
                        self.get_logger().warn('Black indices array has less than 2 elements. Possible error in black boundary detection.')

                # Determine the frame_id based on the color analysis
                if Rtotal > Gtotal and Rtotal > Btotal:
                    msg.frame_id = '-1'  # CW
                elif Gtotal > Rtotal and Gtotal > Btotal:
                    msg.frame_id = '0'   # STOP
                else:
                    msg.frame_id = '+1'  # CCW

                # Visualize the image with black boundaries (optional)
                cv2.imshow('Black Boundaries Visualization', visualization_image)
                cv2.waitKey(1)

            except Exception as e:
                self.get_logger().error(f'Error during color analysis: {e}')
                msg.frame_id = '+1'

            # Publish color state
            self.color_pub.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {e}')
        except Exception as e:
            self.get_logger().error(f'An unexpected error occurred: {e}')

if __name__ == '__main__':
    rclpy.init()
    detector = DetermineColor()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
