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
        self.image_sub = self.create_subscription(Image, '/color', self.callback, 10)
        self.color_pub = self.create_publisher(Header, '/rotate_cmd', 10)
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            # 이미지 토픽을 듣고
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            msg = Header()
            msg = data.header
            msg.frame_id = '0'  # 기본값: 정지
            
            # 이미지를 HSV로 변환
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # 전체 이미지에서 가장 많이 차지하는 색상 찾기
            flat_hsv = hsv_image.reshape(-1, hsv_image.shape[-1])
            unique, counts = np.unique(flat_hsv, return_counts=True, axis=0)
            dominant_color = unique[np.argmax(counts)]
            
            # H 값에 따라 명령 결정
            hue = dominant_color[0]
            if 0 <= hue < 30 or 150 <= hue <= 180:  # 빨간색 범위
                msg.frame_id = '-1'  # CW
            elif 30 <= hue < 90:  # 초록색 범위
                msg.frame_id = '0'   # 정지
            else:  # 나머지는 파란색으로 간주
                msg.frame_id = '+1'  # CCW
            
            self.color_pub.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error('이미지 변환 실패: %s' % e)

if __name__ == '__main__':
    rclpy.init()
    detector = DetermineColor()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()
