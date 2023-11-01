import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

import math
import numpy as np

class GazeTracker(Node):
    def __init__(self):
        super().__init__('driver_detector')
        self.face_3d = np.array([
            [0.0, 0.0, 0.0],            # Nose tip
            [0.0, -330.0, -65.0],       # Chin
            [-225.0, 170.0, -135.0],    # Left eye left corner
            [225.0, 170.0, -135.0],     # Right eye right corner
            [-150.0, -150.0, -125.0],   # Left Mouth corner
            [150.0, -150.0, -125.0]     # Right mouth corner
        ], dtype=np.float64)

        # Reposition left eye corner to be the origin
        self.leye_3d = np.array(self.face_3d)
        self.leye_3d[:,0] += 225
        self.leye_3d[:,1] -= 175
        self.leye_3d[:,2] += 135

        # Reposition right eye corner to be the origin
        self.reye_3d = np.array(self.face_3d)
        self.reye_3d[:,0] -= 225
        self.reye_3d[:,1] -= 175
        self.reye_3d[:,2] += 135

        self.face_keypoints_sub = self.create_subscription(Float32MultiArray, 'gaze_keypoints_coords', self.gazeKeypointsCallback, 1)

    def gazeKeypointsCallback(self, msg):
        """
        Callback function for gaze keypoints
        """

        face_2d = []
        nose = msg.data[0:2]
        chin = msg.data[2:4]
        left_eye_corner = msg.data[4:6]
        right_eye_corner = msg.data[6:8]
        left_mouth_corner = msg.data[8:10]
        right_mouth_corner = msg.data[10:12]

        face_2d.append(nose)
        face_2d.append(chin)
        face_2d.append(left_eye_corner)
        face_2d.append(right_eye_corner)
        face_2d.append(left_mouth_corner)
        face_2d.append(right_mouth_corner)

        face_2d_head = np.array([
            nose,
            chin,
            left_eye_corner,
            right_eye_corner,
            left_mouth_corner,
            right_mouth_corner
        ], dtype=np.float64)

        face_2d = np.asarray(face_2d)

       


def main(args=None):
    rclpy.init(args=args)

    gaze_estimator = GazeTracker()
    rclpy.spin(gaze_estimator)

if __name__ == '__main__':
    main()
