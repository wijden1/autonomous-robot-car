import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class LaneDetector(Node):
    """Finds the yellow line in the camera image and publishes its offset (-1 left .. +1 right)."""

    def __init__(self):
        super().__init__('lane_detector')
        self.declare_parameter('crop_ratio', 0.4)  # use the lower 40 % of the image
        self.crop_ratio = self.get_parameter('crop_ratio').value

        self.offset_pub = self.create_publisher(Float32, 'lane/offset', 10)
        self.debug_pub = self.create_publisher(Image, 'lane/debug', 10)
        self.create_subscription(Image, 'camera/image_raw', self.on_image, 10)

        self.frames = 0
        self.t0 = time.monotonic()
        self.get_logger().info('Lane detector running')

    def on_image(self, msg):
        if msg.encoding != 'rgb8':
            self.get_logger().warn(f'Unexpected encoding {msg.encoding}', throttle_duration_sec=5.0)
            return

        # ROS image -> numpy array (height x width x 3)
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        img = raw[:, :msg.width * 3].reshape(msg.height, msg.width, 3)

        # Look only at the lower part of the image (the floor right in front of the car)
        top = int(msg.height * (1.0 - self.crop_ratio))
        roi = img[top:, :]

        # Yellow pixels -> white, everything else -> black
        hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, (20, 100, 100), (40, 255, 255))

        # Centre of the yellow area
        m = cv2.moments(mask)
        out = Float32()
        if m['m00'] > 50 * 255:  # at least about 50 yellow pixels
            cx = m['m10'] / m['m00']
            out.data = float((cx - msg.width / 2) / (msg.width / 2))
        else:
            out.data = math.nan  # line lost
        self.offset_pub.publish(out)

        # Black/white debug image, viewable in rqt_image_view
        debug = Image()
        debug.header = msg.header
        debug.height, debug.width = mask.shape
        debug.encoding = 'mono8'
        debug.step = mask.shape[1]
        debug.data = mask.tobytes()
        self.debug_pub.publish(debug)

        # Log the processing rate every 5 seconds
        self.frames += 1
        elapsed = time.monotonic() - self.t0
        if elapsed > 5.0:
            self.get_logger().info(f'Processing {self.frames / elapsed:.1f} frames/s')
            self.frames = 0
            self.t0 = time.monotonic()


def main():
    rclpy.init()
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()