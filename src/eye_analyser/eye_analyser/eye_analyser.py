import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

import math
import numpy as np

class Analyser(Node):
    def __init__(self):
        super().__init__('driver_detector')
        
        # Variables
        self.right_eye_coords = []
        self.left_eye_coords = []
        self.blink_count_left_eye = 0
        self.blink_count_right_eye = 0
        self.blink_count_both = 0
        self.both_eye_closed = False
        self.count_closed_left_eye_frames = 0
        self.count_closed_right_eye_frames = 0
        self.count_closed_both_eyes_frames = 0

        self.min_frames_if_disturbed = 10
        self.min_frames_if_sleeping = 20
        self.right_eye_closed = Bool()
        self.left_eye_closed = Bool()

        # Subscribers
        self.iris_coords_sub = self.create_subscription(Float32MultiArray, 'iris_coordinates', self.irisCoordsCallback, 1)
        self.eye_keypoints_sub = self.create_subscription(Float32MultiArray, 'eye_keypoints_coords', self.eyeKeyPointsCallback, 1)
        self.right_eye_closed_pub = self.create_publisher(Bool, 'right_eye_closed', 1)
        self.left_eye_closed_pub = self.create_publisher(Bool, 'left_eye_closed', 1)
        self.is_disturbed_pub = self.create_publisher(Bool, 'is_eye_disturbed', 1)
        self.is_sleeping_pub = self.create_publisher(Bool, 'is_sleeping', 1)
        

    def irisCoordsCallback(self, msg):
        """
        Callback function for iris
        """

    def eyeKeyPointsCallback(self, msg):
        """
        Callback function for eye's keypoint
        """
        # self.get_logger().info('I heard: "%s"' % msg.data)
        self.left_eye_coords = msg.data[8:16]
        self.right_eye_coords = msg.data[0:8]
        # print(f"Left eye:{self.left_eye_coords[0]}")
        # print(f"Right eye:{self.right_eye_coords}")
        self.detectBlink()
        self.detectDisturbedEye()
        self.detectSleep(self.count_closed_both_eyes_frames)

    def detectBlink(self, min_ratio=8.0):
        """
        Detects if the person is blinking. 
        """
        # Initialize coordinates of parts of the eyes
        right_eye_top = self.right_eye_coords[0:2]
        right_eye_bottom = self.right_eye_coords[2:4]
        right_eye_left = self.right_eye_coords[4:6]
        right_eye_right = self.right_eye_coords[6:8]
        left_eye_top = self.left_eye_coords[0:2]
        left_eye_bottom = self.left_eye_coords[2:4]
        left_eye_left = self.left_eye_coords[4:6]
        left_eye_right = self.left_eye_coords[6:8]

        # Finding distances for right eye
        right_horizontal_distance = self.distanceBetweenPoints(right_eye_left, right_eye_right)
        right_vertical_distance = self.distanceBetweenPoints(right_eye_top, right_eye_bottom)
        # Finding distances for left eye
        left_horizontal_distance = self.distanceBetweenPoints(left_eye_left, left_eye_right)
        left_vertical_distance = self.distanceBetweenPoints(left_eye_top, left_eye_bottom)
        # Finding ratio of LEFT and Right Eyes
        right_eye_ratio = right_horizontal_distance/right_vertical_distance
        left_eye_ratio = left_horizontal_distance/left_vertical_distance

        # print(f"Left eye:{left_eye_ratio}")
        # print(f"Right eye:{right_eye_ratio}")

        # Counting blinks for right eye
        if right_eye_ratio >= min_ratio:
            self.right_eye_closed.data = True
        else:
            if self.right_eye_closed.data == True:
                self.blink_count_right_eye += 1
            self.right_eye_closed.data = False
        # Counting blinks for left eye
        if left_eye_ratio >= min_ratio:
            self.left_eye_closed.data = True
        else:
            if self.left_eye_closed.data == True:
                self.blink_count_left_eye += 1
            self.left_eye_closed.data = False

        # Detecting if one of the eyes are closed
        if self.left_eye_closed.data:
            self.count_closed_left_eye_frames += 1
        else:
            self.count_closed_left_eye_frames = 0

        if self.right_eye_closed.data:
            self.count_closed_right_eye_frames += 1
        else:
            self.count_closed_right_eye_frames = 0

        # Detecting if both eyes closed
        if self.left_eye_closed.data and self.right_eye_closed.data:
            if self.both_eye_closed:
                self.count_closed_both_eyes_frames +=1
            self.both_eye_closed = True
        else:
            if self.both_eye_closed:
                self.blink_count_both +=1
            self.count_closed_both_eyes_frames = 0
            self.both_eye_closed = False

        self.right_eye_closed_pub.publish(self.right_eye_closed)
        self.left_eye_closed_pub.publish(self.left_eye_closed)
        
    def estimateGazeDirection(self):
        """
        Estimate where the analyzed person is looking at.
        """

    def detectDisturbedEye(self, target_eye, count_closed_both_eyes_frames, min_frames_if_sleeping=10):
        """
        Detects if an eye is disturbed based on blinking time.
        """
        is_disturbed = Bool()
        if count_closed_both_eyes_frames >= min_frames_if_sleeping:
            is_disturbed.data = True
        else:
            is_disturbed.data = False
        self.is_disturbed_pub.publish(is_disturbed)

    def detectSleep(self, count_closed_both_eyes_frames, min_frames_if_sleeping=30):
        """
        Detects if the person is sleeping based on blinking time.
        """
        is_sleeping = Bool()
        if count_closed_both_eyes_frames >= min_frames_if_sleeping:
            is_sleeping.data = True
        else:
            is_sleeping.data = False
        self.is_sleeping_pub.publish(is_sleeping)


    def distanceBetweenPoints(self, point_a, point_b):
        """
        Returns Euclidean distance between two points.
        """
        x1, y1 = point_a
        x2, y2 = point_b
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

def main(args=None):
    rclpy.init(args=args)

    analyser = Analyser()
    rclpy.spin(analyser)

if __name__ == '__main__':
    main()