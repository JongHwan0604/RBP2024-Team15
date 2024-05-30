import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image
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
            # 이미지 토픽을 듣습니다.
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # rotate_cmd 메시지를 준비합니다.
            msg = Header()
            msg = data.header
            msg.frame_id = '0'  # 기본값: 정지

            # 배경 색상을 결정합니다.
            # TODO
            # 색상을 결정하고 frame_id에 +1, 0, -1을 할당합니다.
            determined_color = self.determine_color(image)
            if determined_color == 'R':
                msg.frame_id = '+1'  # 반시계 방향
            elif determined_color == 'G':
                msg.frame_id = '0'   # 정지
            elif determined_color == 'B':
                msg.frame_id = '-1'  # 시계 방향
            
            # color_state를 발행합니다.
            self.color_pub.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error('이미지 변환 실패: %s' % e)

    def determine_color(self, img):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 기울어진 직사각형을 포함한 모든 직사각형 탐지
        B = cv2.inRange(img_hsv, np.array([0, 0, 0]), np.array([180, 255, 50]))
        contours, _ = cv2.findContours(B, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 기울어진 직사각형을 필터링합니다.
        filtered_contours = self.filter_contours(contours)

        if not filtered_contours:
            return None

        CR = {'R': 0, 'G': 0, 'B': 0}
        for contour in filtered_contours:
            mask = np.zeros(img_hsv.shape[:2], dtype=np.uint8)
            cv2.drawContours(mask, [contour], -1, 255, -1)
            masked_img_hsv = cv2.bitwise_and(img_hsv, img_hsv, mask=mask)
            
            # 색상 영역을 정의합니다.
            red_mask = cv2.inRange(masked_img_hsv, (0, 50, 50), (10, 255, 255)) | cv2.inRange(masked_img_hsv, (170, 50, 50), (180, 255, 255))
            green_mask = cv2.inRange(masked_img_hsv, (50, 50, 50), (70, 255, 255))
            blue_mask = cv2.inRange(masked_img_hsv, (110, 50, 50), (130, 255, 255))
            
            # 색상별 픽셀 수를 계산합니다.
            CR['R'] += cv2.countNonZero(red_mask)
            CR['G'] += cv2.countNonZero(green_mask)
            CR['B'] += cv2.countNonZero(blue_mask)
        
        # 가장 많은 픽셀을 가진 색상을 결정합니다.
        dominant_color = max(CR, key=CR.get)
        return dominant_color

    def filter_contours(self, contours):
        filtered_contours = []
        for contour in contours:
            if cv2.contourArea(contour) > 2000:
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                if len(box) == 4:
                    filtered_contours.append(contour)
        return filtered_contours

if __name__ == '__main__':
    rclpy.init()
    node = DetermineColor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()