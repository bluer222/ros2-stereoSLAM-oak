import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
import numpy as np
from pathlib import Path
import configparser


class StereoRectifier(Node):
    """Rectify stereo image pairs using calibration data."""

    def __init__(self):
        super().__init__('stereo_rectifier')

        self.declare_parameter('left_topic', '/camera/left/image_raw')
        self.declare_parameter('right_topic', '/camera/right/image_raw')
        self.declare_parameter('left_rect_topic', '/camera/left/image_rect')
        self.declare_parameter('right_rect_topic', '/camera/right/image_rect')
        self.declare_parameter('calibration_file', '')

        self.left_topic = self.get_parameter('left_topic').get_parameter_value().string_value
        self.right_topic = self.get_parameter('right_topic').get_parameter_value().string_value
        self.left_rect_topic = self.get_parameter('left_rect_topic').get_parameter_value().string_value
        self.right_rect_topic = self.get_parameter('right_rect_topic').get_parameter_value().string_value
        self.calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.history = QoSHistoryPolicy.KEEP_LAST

        self.sub_left = self.create_subscription(Image, self.left_topic, self.on_left_image, qos)
        self.sub_right = self.create_subscription(Image, self.right_topic, self.on_right_image, qos)

        self.pub_left_rect = self.create_publisher(Image, self.left_rect_topic, qos)
        self.pub_right_rect = self.create_publisher(Image, self.right_rect_topic, qos)

        self.bridge = CvBridge()
        self.left_frame = None
        self.left_header = None

        self.map_left_x = None
        self.map_left_y = None
        self.map_right_x = None
        self.map_right_y = None

        self._load_calibration()

    # ------------------------------------------------------------------ #
    #  Calibration loading
    # ------------------------------------------------------------------ #

    def _load_calibration(self):
        """Load calibration from a ZED .conf file."""

        if not self.calibration_file or not Path(self.calibration_file).exists():
            self.get_logger().warn('No valid calibration file provided — images will be published unrectified.')
            return

        try:
            config = configparser.ConfigParser()
            config.read(self.calibration_file)

            width, height = 1280, 720
            res = 'HD'

            def stereo(key, fallback=0.0):
                return float(config['STEREO'].get(key, fallback))

            def left(key, fallback=0.0):
                return float(config[f'LEFT_CAM_{res}'].get(key, fallback))

            def right(key, fallback=0.0):
                return float(config[f'RIGHT_CAM_{res}'].get(key, fallback))

            T = np.array([[-stereo('Baseline', 0)],
                          [stereo(f'TY_{res}', 0)],
                          [stereo(f'TZ_{res}', 0)]])

            R_vec = np.array([stereo(f'RX_{res}', 0),
                              stereo(f'CV_{res}', 0),
                              stereo(f'RZ_{res}', 0)])
            R, _ = cv2.Rodrigues(R_vec)

            K_left = np.array([[left('fx'), 0, left('cx')],
                               [0, left('fy'), left('cy')],
                               [0, 0, 1]])
            K_right = np.array([[right('fx'), 0, right('cx')],
                                [0, right('fy'), right('cy')],
                                [0, 0, 1]])

            d_left = np.array([[left('k1')], [left('k2')],
                               [left('p1')], [left('p2')], [left('k3')]])
            d_right = np.array([[right('k1')], [right('k2')],
                                [right('p1')], [right('p2')], [right('k3')]])

            R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
                K_left, d_left, K_right, d_right,
                (width, height), R, T,
                flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
                newImageSize=(width, height)
            )

            self.map_left_x, self.map_left_y = cv2.initUndistortRectifyMap(
                K_left, d_left, R1, P1, (width, height), cv2.CV_32FC1)
            self.map_right_x, self.map_right_y = cv2.initUndistortRectifyMap(
                K_right, d_right, R2, P2, (width, height), cv2.CV_32FC1)

            self.get_logger().info(f'Loaded .conf calibration ({res}): fx={left("fx"):.1f}')

        except Exception as e:
            self.get_logger().error(f'Failed to load .conf calibration: {e}')

    # ------------------------------------------------------------------ #
    #  Callbacks
    # ------------------------------------------------------------------ #

    def on_left_image(self, msg):
        self.left_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.left_header = msg.header

    def on_right_image(self, msg):
        right_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        if self.left_frame is None:
            self.get_logger().info('Has right frame, waiting for left', throttle_duration_sec=5.0)
            return

        left = self.left_frame.copy()
        right = right_frame

        encoding = 'mono8' if left.ndim == 2 else 'bgr8'

        if self.map_left_x is not None:
            left = cv2.remap(left, self.map_left_x, self.map_left_y, cv2.INTER_LINEAR)
            right = cv2.remap(right, self.map_right_x, self.map_right_y, cv2.INTER_LINEAR)
        else:
            self.get_logger().warn('No calibration maps — publishing raw images.', throttle_duration_sec=5.0)

        left_msg = self.bridge.cv2_to_imgmsg(left, encoding=encoding)
        left_msg.header = self.left_header

        right_msg = self.bridge.cv2_to_imgmsg(right, encoding=encoding)
        right_msg.header = msg.header

        self.pub_left_rect.publish(left_msg)
        self.pub_right_rect.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StereoRectifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()