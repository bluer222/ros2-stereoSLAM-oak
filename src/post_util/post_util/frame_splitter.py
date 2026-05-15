import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
import numpy as np


class FrameSplitter(Node):
    """Subscribe to a side-by-side stereo image and split into left/right images."""

    def __init__(self):
        super().__init__('frame_splitter')

        # Parameters
        self.declare_parameter('input_topic', '/camera/dual/image_raw')
        self.declare_parameter('left_topic', '/camera/left/image_raw')
        self.declare_parameter('right_topic', '/camera/right/image_raw')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        left_topic = self.get_parameter('left_topic').get_parameter_value().string_value
        right_topic = self.get_parameter('right_topic').get_parameter_value().string_value

        # QoS profile matching the input
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.history = QoSHistoryPolicy.KEEP_LAST

        # Subscriber
        self.sub = self.create_subscription(Image, input_topic, self.image_callback, qos)

        # Publishers
        self.pub_left = self.create_publisher(Image, left_topic, qos)
        self.pub_right = self.create_publisher(Image, right_topic, qos)

        self.bridge = CvBridge()

        self.get_logger().info(f'Subscribing to {input_topic}')
        self.get_logger().info(f'Publishing left to {left_topic}, right to {right_topic}')

    def image_callback(self, msg):
        """Process incoming side-by-side stereo image."""
        try:
            # Convert ROS image to numpy array
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Get image dimensions
            height, width = frame.shape[:2]
            mid_width = width // 2

            # Split horizontally (left is first half, right is second half)
            left_frame = frame[:, :mid_width]
            right_frame = frame[:, mid_width:]

            # Convert back to ROS Image messages
            left_msg = self.bridge.cv2_to_imgmsg(left_frame, encoding=msg.encoding)
            right_msg = self.bridge.cv2_to_imgmsg(right_frame, encoding=msg.encoding)

            # Copy header info
            left_msg.header = msg.header
            right_msg.header = msg.header

            # Publish
            self.pub_left.publish(left_msg)
            self.pub_right.publish(right_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing frame: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FrameSplitter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
