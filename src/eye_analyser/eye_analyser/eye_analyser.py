import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

class Analyser(Node):
    def __init__(self):
        super().__init__('driver_detector')
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
    
    def leftEyeKeyPointsCallback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def analyze(self):
        pass

    def detectBlink(self):
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

def main(args=None):
    rclpy.init(args=args)

    analyser = Analyser()
    analyser.analyze()