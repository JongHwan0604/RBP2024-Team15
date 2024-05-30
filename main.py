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
            # 이미지 토픽 수신
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # rotate_cmd 메시지 준비
            msg = Header()
            msg = data.header
            msg.frame_id = '0'  # 기본값: STOP

            # 배경 색상 결정
            dominant_color = self.determine_dominant_color(image)
            if dominant_color == 'G':
                # 초록색이 지배색이라면, 모니터로 간주할 수 있습니다.
                # 여기서 원하는 로직으로 처리하세요. 예시로 frame_id를 '0'으로 설정합니다.
                msg.frame_id = '0'  # STOP

            # color_state 발행
            self.color_pub.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error('이미지 변환 실패: %s' % e)

    def determine_dominant_color(self, image):
        img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 초록색 필터
        green_mask = cv2.inRange(img_hsv, (36, 25, 25), (70, 255,255))
        green = cv2.bitwise_and(image, image, mask=green_mask)
        green_pixels = cv2.countNonZero(green_mask)

        # 여기에 다른 색상 필터를 추가할 수 있습니다.

        # 지배색 결정
        # 예시로 초록색만을 체크합니다. 필요에 따라 다른 색상도 추가하여 비교할 수 있습니다.
        dominant_color = 'G' if green_pixels > 0 else 'None'
        return dominant_color

if __name__ == '__main__':
    rclpy.init()
    detector = DetermineColor()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()
