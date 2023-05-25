import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from .PoseEstimationModule import PoseDetector

import time

class DriverDetection(Node):
    def __init__(self):
        super().__init__('driver_detector')
        self.cap = cv2.VideoCapture(0)

        self.poseDetector = PoseDetector()
        # self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.image_face_pub = self.create_publisher(Image, 'image_face', 1)
        self.image_eyes_pub = self.create_publisher(Image, 'image_eyes', 1)
        self.image_hands_pub = self.create_publisher(Image, 'image_hands', 1)
        self.br = CvBridge()

    def detect(self):
        cTime = pTime = 0
        ret, frame = self.cap.read()
        while True:
            ret, frame = self.cap.read()

            if not ret:
                print("Can't recieve frame (stream end?). Ending...")
                break


            self.poseDetector.findPose(frame)
            cTime = time.time()
            fps = 1/(cTime-pTime)
            pTime = cTime

            cv2.rectangle(frame, (0,0), (125, 30), (0, 0, 0), cv2.FILLED)
            cv2.putText(frame, f"FPS: {str(int(fps))}", (0, 25), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

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