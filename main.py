import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header

class MonitorEdgeDetector(Node):
    def __init__(self):
        super().__init__('monitor_edge_detector')
        self.image_sub = self.create_subscription(Image, '/color', self.callback, 10)
        self.edge_pub = self.create_publisher(Header, '/rotate_cmd', 10)
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
            
            # Value(명도)가 낮은 영역 찾기 (검은색 영역)
            low_value_area = hsv_image[:,:,2] < 30  # Value 임곗값 조정 필요
            
            # 검은색 영역이 충분히 큰 경우(예: 모니터 테두리로 간주)
            if np.sum(low_value_area) > (hsv_image.shape[0] * hsv_image.shape[1] * 0.1):  # 검은색 영역 비율 조정 필요
                msg.frame_id = '1'  # 검은색 모니터 테두리 감지
            else:
                msg.frame_id = '0'  # 검은색 모니터 테두리 미감지
            
            self.edge_pub.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error('이미지 변환 실패: %s' % e)

if __name__ == '__main__':
    rclpy.init()
    detector = MonitorEdgeDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()
