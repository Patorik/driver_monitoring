import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

import math

class Analyser(Node):
    def __init__(self):
        super().__init__('driver_detector')
        
        # Variables
        self.right_eye_coords = []
        self.left_eye_coords = []

        # Subscribers
        self.iris_coords_sub = self.create_subscription(Float32MultiArray, 'iris_coordinates', self.irisCoordsCallback, 1)
        self.right_eye_keypoints_sub = self.create_subscription(Float32MultiArray, 'right_eye_keypoints_coords', self.rightEyeKeyPointsCallback, 1)
        self.left_eye_keypoints_sub = self.create_subscription(Float32MultiArray, 'left_eye_keypoints_coords', self.leftEyeKeyPointsCallback, 1)
        self.right_eye_closed_pub = self.create_publisher(Bool, 'right_eye_closed')
        self.left_eye_closed_pub = self.create_publisher(Bool, 'left_eye_closed')
        self.is_disturbed_pub = self.create_publisher(Bool, 'is_eye_disturbed')
        self.is_sleeping_pub = self.create_publisher(Bool, 'is_sleeping')
        

    def irisCoordsCallback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
    def rightEyeKeyPointsCallback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.right_eye_coords = msg.data
    
    def leftEyeKeyPointsCallback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.left_eye_coords = msg.data

    def analyze(self):
        self.detectBlink(self.left_eye_coords, self.right_eye_coords)
        pass

    def detectBlink(self, left_eye_coords, right_eye_coords):
        """
        Detects if the person is blinking. 
        """
        
        
    def estimateGazeDirection(self):
        """
        Estimate where the analyzed person is looking at.
        """

    def detectDisturbedEye(self):
        """
        Detects if an eye is disturbed based on blinking time.
        """

    def detectSleep(self):
        """
        Detects if the person is sleeping based on blinking time.
        """

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
    analyser.analyze()