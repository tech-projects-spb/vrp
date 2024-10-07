import rclpy
from rclpy.node import Node
from sensor_msgs import Image
import cv2
import numpy as np
from cv_bridge import CvBridge


class LedShowNode(Node):
    def __init__(self, name='led_show'):
        super().__init__(name)
        self.left_img_pub_ = self.create_publisher(Image, '/booblik/sensors/LED/left', 10)
        self.right_img_pub_ = self.create_publisher(Image, '/booblik/sensors/LED/right', 10)
        self.bridge = CvBridge()

        self.timer = self.create_timer(2, self.publish_images)
    
    def publish_images(self):
        ...




def main(args=None):
    rclpy.init()
    task = LedShowNode(args=args)
    rclpy.spin(task)
    rclpy.shutdown()

if __name__ == '__main__':
    main()