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
            

 

    def DetermineColor(filename):
    img_0 = cv2.imread(filename)    
    
        if img_0 is None:
            return None

    img_1 = cv2.cvtColor(img_0, cv2.COLOR_BGR2HSV)

    B=cv2.inRange(img_1, np.array([0,0,0]), np.array([180,255,50]))
    C,_=cv2.findContours(B, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    def DetermineColor(contours):
        filtered_contours = []
        for contour in contours:
            if cv2.contourArea(contour) > 2000:
                enf = cv2.arcLength(contour, True)
                rms = cv2.approxPolyDP(contour, 0.02*enf, True)
            if len(rms) == 4:
                    filtered_contours.append(contour)
                return filtered_contours
    
    mps = filter_contours(C)
        
    if not mps:
        return None

    CR = {'R': 0, 'G': 0, 'B': 0}
    for mp in mps:
        mask = np.zeros(img_1.shape[:2], dtype = np.uint8)
        cv2.drawContours(mask, [mp], -1, 255, -1)
        mrh = cv2.bitwise_and(img_1, img_1, mask=mask)
        
        rm = cv2.inRange(mrh, (0, 50, 50), (10, 255, 255)) | cv2.inRange(mrh, (170, 50, 50), (180, 255, 255))
        gm = cv2.inRange(mrh, (50, 50, 50), (70, 255, 255))
        bm = cv2.inRange(mrh, (110, 50, 50), (130, 255, 255))
        
        CR['R'] += cv2.countNonZero(rm)
        CR['G'] += cv2.countNonZero(gm)
        CR['B'] += cv2.countNonZero(bm)
        
    DC = max(CR, key=CR.get)
    return DC

if __name__ == '__main__':
    result=[]
    for filename in sorted(glob('public_imgs/*.PNG')):
        DC = DetermineColor(filename)
        result.append(DC)
    print(result)

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
