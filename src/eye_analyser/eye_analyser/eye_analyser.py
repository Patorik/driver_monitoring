import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

class Analyser(Node):
    def __init__(self):
        super().__init__('driver_detector')

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