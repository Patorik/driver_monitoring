import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from .PoseEstimationModule import PoseDetector
from .FaceMeshDetector import FaceMeshDetector
from .HandTrackingModule import HandDetector

import time

class DriverDetection(Node):
    def __init__(self):
        super().__init__('driver_detector')
        self.cap = cv2.VideoCapture(0)

        self.poseDetector = PoseDetector()
        self.faceMeshDetector = FaceMeshDetector()
        self.handDetector = HandDetector()

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.image_face_pub = self.create_publisher(Image, 'image_face', 1)
        self.image_iris_pub = self.create_publisher(Image, 'image_iris', 1)
        self.image_pose_pub = self.create_publisher(Image, 'image_pose', 1)
        self.image_hand_pub = self.create_publisher(Image, 'image_hand', 1)
        self.iris_coords_pub = self.create_publisher(Float32MultiArray, 'iris_coordinates', 1)
        self.eye_keypoints_pub = self.create_publisher(Float32MultiArray, 'eye_keypoints_coords', 1)
        self.gaze_keypoints_pub = self.create_publisher(Float32MultiArray, 'gaze_keypoints_coords', 1)
        self.br = CvBridge()

    def detect(self):
        cTime = pTime = 0
        ret, frame = self.cap.read()
        while True:
            ret, frame = self.cap.read()
            cv2.flip(frame, 1)

            if not ret:
                print("Can't recieve frame (stream end?). Ending...")
                break

            image_face = self.faceMeshDetector.findFaceMesh(frame)
            image_pose = self.poseDetector.findPose(frame)
            image_iris = self.faceMeshDetector.findIris(frame)
            image_hands = self.handDetector.findHands(frame)
            cTime = time.time()
            fps = 1/(cTime-pTime)
            pTime = cTime

            cv2.rectangle(frame, (0,0), (125, 30), (0, 0, 0), cv2.FILLED)
            cv2.putText(frame, f"FPS: {str(int(fps))}", (0, 25), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

            # cv2.imshow('Frame',frame)
            if cv2.waitKey(1) == ord('q'):
                break

            iris_coordinates = Float32MultiArray()
            iris_coordinates.data = self.faceMeshDetector.getIrisPosition(image_iris)
            # print(self.faceMeshDetector.getIrisPosition(image_iris))
            face_keypoint_coordinates = Float32MultiArray()
            face_data = self.faceMeshDetector.getFaceKeypointPositions(image_face)
            if len(face_data) == 12:
                face_keypoint_coordinates.data = self.faceMeshDetector.getFaceKeypointPositions(image_face)
                cv2.circle(image_face, (int(face_data[0]),int(face_data[1])), 5, (0, 100, 255), -1)
                cv2.circle(image_face, (int(face_data[2]),int(face_data[3])), 5, (0, 100, 255), -1)
                cv2.circle(image_face, (int(face_data[4]),int(face_data[5])), 5, (0, 100, 255), -1)
                cv2.circle(image_face, (int(face_data[6]),int(face_data[7])), 5, (0, 100, 255), -1)
                cv2.circle(image_face, (int(face_data[8]),int(face_data[9])), 5, (0, 100, 255), -1)
                cv2.circle(image_face, (int(face_data[10]),int(face_data[11])), 5, (0, 100, 255), -1)
            
            eye_coordinates = Float32MultiArray()
            eye_data = self.faceMeshDetector.getEyePosition(image_face)
            if len(eye_data) == 16:
                eye_coordinates.data = self.faceMeshDetector.getEyePosition(image_face)
                right_eye_top = (int(eye_coordinates.data[0]), int(eye_coordinates.data[1]))
                right_eye_bottom = (int(eye_coordinates.data[2]), int(eye_coordinates.data[3]))
                left_eye_top = (int(eye_coordinates.data[8]), int(eye_coordinates.data[9]))
                left_eye_bottom = (int(eye_coordinates.data[10]), int(eye_coordinates.data[11]))
                cv2.line(image_face, right_eye_top, right_eye_bottom, (0, 255, 0), 2)
                cv2.line(image_face, left_eye_top, left_eye_bottom, (0, 255, 0), 2)
                self.image_face_pub.publish(self.br.cv2_to_imgmsg(image_face))

            self.image_hand_pub.publish(self.br.cv2_to_imgmsg(image_hands))
            self.image_pose_pub.publish(self.br.cv2_to_imgmsg(image_pose))
            self.image_iris_pub.publish(self.br.cv2_to_imgmsg(image_iris))
            self.iris_coords_pub.publish(iris_coordinates)
            self.eye_keypoints_pub.publish(eye_coordinates)
            self.gaze_keypoints_pub.publish(face_keypoint_coordinates)
        
        self.cap.release()
        cv2.destroyAllWindows()

    
def main(args=None):
    rclpy.init(args=args)

    detector = DriverDetection()
    detector.detect()

if __name__ == '__main__':
    main()