import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class DriverDetection(Node):
    def __init__(self):
        super().__init__('driver_detector')
        self.cap = cv2.VideoCapture("/home/patorik/Videos/Teszt.MP4")
        self.image_face_pub = self.create_publisher(Image, 'image_face', 1)
        self.image_eyes_pub = self.create_publisher(Image, 'image_eyes', 1)
        self.image_hands_pub = self.create_publisher(Image, 'image_hands', 1)
        self.br = CvBridge()

    def detect(self):
        ret, frame = self.cap.read()
        while True:
            ret, frame = self.cap.read()

            if not ret:
                print("Can't recieve frame (stream end?). Ending...")
                break

            cv2.imshow('Frame',frame)
            if cv2.waitKey(1) == ord('q'):
                break

            self.image_face_pub.publish(self.br.cv2_to_imgmsg(frame))
            self.image_eyes_pub.publish(self.br.cv2_to_imgmsg(frame))
            self.image_hands_pub.publish(self.br.cv2_to_imgmsg(frame))
        
        self.cap.release()
        cv2.destroyAllWindows()

    
def main(args=None):
    rclpy.init(args=args)

    detector = DriverDetection()
    detector.detect()

if __name__ == '__main__':
    main()