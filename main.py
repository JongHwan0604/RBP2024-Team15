import rclpy
import numpy as np
import cv2
from glob import glob

from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
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
            msg = data.header
            msg.frame_id = '0'  # default: STOP
    
            # determine background color
            # TODO 
            # determine the color and assing +1, 0, or, -1 for frame_id
            # msg.frame_id = '+1' # CCW 
            # msg.frame_id = '0'  # STOP
            # msg.frame_id = '-1' # CW 
            try:

                hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
      # lower_black, upper_black is thershold of color black
                lower_black = np.array([0, 0, 0], dtype=np.uint8)
                upper_black = np.array([80, 80, 50], dtype=np.uint8)

                Rtotal, Gtotal, Btotal = 0, 0, 0

                for row in hsv_image:
                # 검은색의 위치 찾기
                    black_indices = np.where(np.all(np.logical_and(lower_black <= row, row <= upper_black), axis=-1))[0]

                    if len(black_indices) >= 2:
                        left_black_index = black_indices[0]
                        right_black_index = black_indices[-1]
                        
                        # 테두리 내부의 색상 추출
                        inner_colors = row[left_black_index:right_black_index]
                        
                        
                        # HSV 기준에 맞는 색상만을 추출하여 픽셀 수 및 각 색상 채널별 픽셀 수 계산
                        Rtotal += np.sum(((inner_colors[:, 0] > 163) | (inner_colors[:, 0] < 18)))
                        Gtotal += np.sum(((inner_colors[:, 0] > 53) & (inner_colors[:, 0] < 78)))
                        Btotal += np.sum(((inner_colors[:, 0] > 103) & (inner_colors[:, 0] < 138)))




                if Rtotal > Gtotal and Rtotal > Btotal:
                    msg.frame_id = '-1'
                elif Gtotal > Rtotal and Gtotal > Btotal:
                    msg.frame_id = '0'
                else:
                    msg.frame_id = '+1'
            
            except:
                msg.frame_id = '+1'
                

            # publish color_state
            self.color_pub.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error('Failed to convert image: %s' % e)


if __name__ == '__main__':
    rclpy.init()
    detector = DetermineColor()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()
