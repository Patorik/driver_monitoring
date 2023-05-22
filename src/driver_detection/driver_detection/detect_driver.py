import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

class DriverDetection(Node):
    def __init__(self):
        super().__init__('driver_detector')
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
        self.image_face_pub = self.create_publisher(Image, 'test_topic', 1)
        self.image_eyes_pub = self.create_publisher(Image, 'test_topic', 1)
        self.image_hands_pub = self.create_publisher(Image, 'test_topic', 1)
        # timer_period = 0.1
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello World: %d' % self.i
    #     self.publisher.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1

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


            face_msg = self.convertToROS2Image(frame)
            self.image_face_pub.publish(face_msg)
        
        self.cap.release()
        cv2.destroyAllWindows()

    def convertToROS2Image(self, frame):
        msg = Image()
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.height = np.shape(frame)[0]
        msg.width = np.shape(frame)[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = np.shape(frame)[2] * np.shape(frame)[1]
        msg.data = np.array(frame).tobytes()
        return msg

    
def main(args=None):
    rclpy.init(args=args)

    detector = DriverDetection()
    detector.detect()

    # rclpy.spin(publish_image)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # publish_image.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()