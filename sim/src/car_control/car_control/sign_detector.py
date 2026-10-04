import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class SignDetector(Node):
    """Detects red (stop) and blue (slow) signs by colour and size in the camera image."""

    def __init__(self):
        super().__init__('sign_detector')
        self.declare_parameter('min_area', 1500)  # pixels; bigger = sign must be closer
        self.min_area = self.get_parameter('min_area').value

        self.sign_pub = self.create_publisher(String, 'sign', 10)
        self.create_subscription(Image, 'camera/image_raw', self.on_image, 10)
        self.last_sign = 'none'
        self.get_logger().info(f'Sign detector running, min area {self.min_area} px')

    @staticmethod
    def largest_area(mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return max((cv2.contourArea(c) for c in contours), default=0.0)

    def on_image(self, msg):
        if msg.encoding != 'rgb8':
            return
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        img = raw[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        # Red wraps around the hue circle, so it needs two ranges
        red = cv2.inRange(hsv, (0, 100, 50), (10, 255, 255)) | \
              cv2.inRange(hsv, (170, 100, 50), (180, 255, 255))
        blue = cv2.inRange(hsv, (100, 100, 50), (130, 255, 255))

        red_area = self.largest_area(red)
        blue_area = self.largest_area(blue)

        if red_area >= self.min_area and red_area >= blue_area:
            sign = 'stop'
        elif blue_area >= self.min_area:
            sign = 'slow'
        else:
            sign = 'none'

        if sign != self.last_sign:
            self.get_logger().info(f'Sign: {sign} (red {red_area:.0f} px, blue {blue_area:.0f} px)')
            self.last_sign = sign
        self.sign_pub.publish(String(data=sign))


def main():
    rclpy.init()
    node = SignDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()