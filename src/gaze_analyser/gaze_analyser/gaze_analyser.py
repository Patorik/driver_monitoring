import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

import math
import numpy as np

import cv2

class GazeTracker(Node):
    def __init__(self):
        super().__init__('driver_detector')

        self.resolution = (1280, 720)

        # Threshold of how close scores should be to average between frames
        self.threshold = 0.3
        
        # Gaze Score multiplier (Higher multiplier = Gaze affects headpose estimation more)
        self.x_score_multiplier = 4
        self.y_score_multiplier = 4

        self.last_lx, self.last_rx = 0, 0
        self.last_ly, self.last_ry = 0, 0

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
        self.face_direction_vertical = self.create_publisher(String, 'face_direction_vertical', 1)
        self.face_direction_horizontal = self.create_publisher(String, 'face_direction_horizontal', 1)

    def gazeKeypointsCallback(self, msg):
        """
        Callback function for gaze keypoints
        """

        if len(msg.data) < 32:
            return
        
        face_2d = []
        nose = msg.data[0:2]
        chin = msg.data[2:4]
        left_eye_corner = msg.data[4:6]
        right_eye_corner = msg.data[6:8]
        left_mouth_corner = msg.data[8:10]
        right_mouth_corner = msg.data[10:12]
        left_eye_left = msg.data[12:14]
        left_eye_right = msg.data[14:16]
        left_eye_bottom = msg.data[16:18]
        left_eye_top = msg.data[18:20]
        right_eye_right = msg.data[20:22]
        right_eye_left = msg.data[22:24]
        right_eye_bottom = msg.data[24:26]
        right_eye_top = msg.data[26:28]
        left_iris = msg.data[28:30]
        right_iris = msg.data[30:32]

        face_2d.append(nose)
        face_2d.append(chin)
        face_2d.append(left_eye_corner)
        face_2d.append(right_eye_corner)
        face_2d.append(left_mouth_corner)
        face_2d.append(right_mouth_corner)
        face_2d.append(left_eye_left)
        face_2d.append(left_eye_right)
        face_2d.append(left_eye_bottom)
        face_2d.append(left_eye_top)
        face_2d.append(right_eye_right)
        face_2d.append(right_eye_left)
        face_2d.append(right_eye_bottom)
        face_2d.append(right_eye_top)

        face_2d_head = np.array([
            nose,
            chin,
            left_eye_corner,
            right_eye_corner,
            left_mouth_corner,
            right_mouth_corner
        ], dtype=np.float64)

        face_2d = np.asarray(face_2d)

          # Calculate left x gaze score
        if (left_eye_left[0] - left_eye_right[0]) != 0:
            lx_score = (left_iris[0] - left_eye_right[0]) / (left_eye_left[0] - left_eye_right[0])
            if abs(lx_score - self.last_lx) < self.threshold:
                lx_score = (lx_score + self.last_lx) / 2
            self.last_lx = lx_score

        # Calculate left y gaze score
        if (left_eye_bottom[1] - left_eye_top[1]) != 0:
            ly_score = (left_iris[1] - left_eye_top[1]) / (left_eye_bottom[1] - left_eye_top[1])
            if abs(ly_score - self.last_ly) < self.threshold:
                ly_score = (ly_score + self.last_ly) / 2
            self.last_ly = ly_score
        
        # Calculate right x gaze score
        if (right_eye_right[0] - right_eye_left[0]) != 0:
            rx_score = (right_iris[0] - right_eye_left[0]) / (right_eye_right[0] - right_eye_left[0])
            if abs(rx_score - self.last_rx) < self.threshold:
                rx_score = (rx_score + self.last_rx) / 2
            self.last_rx = rx_score

        # Calculate right y gaze score
        if (right_eye_bottom[1] - right_eye_top[1]) != 0:
            ry_score = (right_iris[1] - right_eye_bottom[1]) / (right_eye_bottom[1] - right_eye_top[1])
            if abs(ry_score - self.last_ry) < self.threshold:
                ry_score = (ry_score + self.last_ry) / 2
            self.last_ry = ry_score

        # The camera matrix
        focal_length = 1 * self.resolution[0]
        cam_matrix = np.array([ [focal_length, 0, self.resolution[1] / 2],
                                [0, focal_length, self.resolution[0] / 2],
                                [0, 0, 1]])

        # Distortion coefficients 
        dist_coeffs = np.zeros((4, 1), dtype=np.float64)

        # Solve PnP
        _, l_rvec, l_tvec = cv2.solvePnP(self.leye_3d, face_2d_head, cam_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
        _, r_rvec, r_tvec = cv2.solvePnP(self.reye_3d, face_2d_head, cam_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

        # Adjust headpose vector with gaze score
        l_gaze_rvec = np.array(l_rvec)
        l_gaze_rvec[2][0] -= (lx_score-.5) * self.x_score_multiplier
        l_gaze_rvec[0][0] += (ly_score-.5) * self.y_score_multiplier

        r_gaze_rvec = np.array(r_rvec)
        r_gaze_rvec[2][0] -= (rx_score-.5) * self.x_score_multiplier
        r_gaze_rvec[0][0] += (ry_score-.5) * self.y_score_multiplier

        # --- Projection ---

        # Get left eye corner as integer
        l_corner = face_2d_head[2].astype(np.int32)

        # Project axis of rotation for left eye
        axis = np.float32([[-100, 0, 0], [0, 100, 0], [0, 0, 300]]).reshape(-1, 3)
        l_axis, _ = cv2.projectPoints(axis, l_rvec, l_tvec, cam_matrix, dist_coeffs)
        l_gaze_axis, _ = cv2.projectPoints(axis, l_gaze_rvec, l_tvec, cam_matrix, dist_coeffs)        
    
        # Get left eye corner as integer
        r_corner = face_2d_head[3].astype(np.int32)

        # Get left eye corner as integer
        r_axis, _ = cv2.projectPoints(axis, r_rvec, r_tvec, cam_matrix, dist_coeffs)
        r_gaze_axis, _ = cv2.projectPoints(axis, r_gaze_rvec, r_tvec, cam_matrix, dist_coeffs)

        r_corner_end = np.ravel(r_axis[2]).astype(np.int32)
        r_distance_x = math.sqrt((r_corner_end[0] - r_corner[0])**2)
        r_distance_y = math.sqrt((r_corner_end[1] - r_corner[1])**2)
        r_pixels = (r_corner_end[0] - r_corner[0]), (r_corner_end[1] - r_corner[1])
        
        r_direction = []

        if r_pixels[0]>0:
            r_direction.append("LEFT")
        else:
            r_direction.append("RIGHT")

        if r_pixels[1]<0:
            r_direction.append("UP")
        else:
            r_direction.append("DOWN")

        l_corner_end = np.ravel(l_axis[2]).astype(np.int32)
        l_distance_x = math.sqrt((l_corner_end[0] - l_corner[0])**2)
        l_distance_y = math.sqrt((l_corner_end[1] - l_corner[1])**2)
        l_pixels = (l_corner_end[0] - l_corner[0]), (l_corner_end[1] - l_corner[1])

        l_direction = []

        if l_pixels[0]>0:
            l_direction.append("LEFT")
        else:
            l_direction.append("RIGHT")

        if l_pixels[1]<0:
            l_direction.append("UP")
        else:
            l_direction.append("DOWN")
        
        result_direction = []
        if l_distance_x > 50 or r_distance_x > 50:
            if l_direction[0] == "LEFT" and r_direction[0] == "LEFT":
                result_direction.append("LEFT")
            elif r_direction[0] == "RIGHT" and r_direction[0] == "RIGHT":
                result_direction.append("RIGHT")
        else:
            result_direction.append("MIDDLE")
            
        if l_distance_y > 30 or r_distance_y > 30:
            if l_direction[1] == "UP" and r_direction[1] == "UP":
                result_direction.append("UP")
            elif r_direction[1] == "DOWN" and r_direction[1] == "DOWN":
                result_direction.append("DOWN")
        else:
            result_direction.append("MIDDLE")

        result_horizontal_publish = String()
        result_vertical_publish = String()
        result_horizontal_publish.data = result_direction[0]
        result_vertical_publish.data = result_direction[1]

        if len(result_direction) == 2:
            self.face_direction_horizontal.publish(result_horizontal_publish)
            self.face_direction_vertical.publish(result_vertical_publish)
        

def main(args=None):
    rclpy.init(args=args)

    gaze_estimator = GazeTracker()
    rclpy.spin(gaze_estimator)

if __name__ == '__main__':
    main()
