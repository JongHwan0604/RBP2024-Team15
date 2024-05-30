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
            # listen image topic
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # prepare rotate_cmd msg
            # DO NOT DELETE THE BELOW THREE LINES!
            msg = Header()
            msg.stamp = data.header.stamp
            msg.frame_id = '0'  # default: STOP

            # determine background color
            try:
                hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                # lower_black, upper_black is threshold of color black
                lower_black = np.array([0, 0, 0], dtype=np.uint8)
                upper_black = np.array([15, 50, 50], dtype=np.uint8)

                Rtotal, Gtotal, Btotal = 0, 0, 0

                for row_idx, row in enumerate(hsv_image):
                    # Find the indices of black regions in the row
                    black_indices = np.where(np.all(np.logical_and(lower_black <= row, row <= upper_black), axis=-1))[0]

                    if len(black_indices) >= 2:
                        left_black_index = black_indices[0]
                        right_black_index = black_indices[-1]

                        # 시각화: 검은색 픽셀을 초록색으로 변경
                        image[row_idx, left_black_index] = [0, 255, 0]
                        image[row_idx, right_black_index] = [0, 255, 0]

                        # Extract colors within the boundary
                        inner_colors = row[left_black_index:right_black_index]

                        # Count pixels in the color ranges
                        Rtotal += np.sum((inner_colors[:, 0] > 170) | (inner_colors[:, 0] < 10))
                        Gtotal += np.sum((inner_colors[:, 0] > 50) & (inner_colors[:, 0] < 70))
                        Btotal += np.sum((inner_colors[:, 0] > 110) & (inner_colors[:, 0] < 130))

                # Determine the predominant color
                if Rtotal > Gtotal and Rtotal > Btotal:
                    msg.frame_id = '-1'  # CW
                elif Gtotal > Rtotal and Gtotal > Btotal:
                    msg.frame_id = '0'   # STOP
                else:
                    msg.frame_id = '+1'  # CCW

            except Exception as e:
                self.get_logger().error(f"Color determination failed: {e}")
                msg.frame_id = '+1'

            # publish color_state
            self.color_pub.publish(msg)

            # 시각화된 이미지를 퍼블리시하기 위한 추가 기능
            visualized_image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            visualized_image_msg.header = data.header
            self.color_pub.publish(visualized_image_msg)

        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {e}')

if __name__ == '__main__':
    rclpy.init()
    detector = DetermineColor()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()
