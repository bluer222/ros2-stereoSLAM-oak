import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
import cv2
import numpy as np
from pathlib import Path
import configparser


class StereoCameraNode(Node):
    """
    Subscribe to a side-by-side stereo image, split into left/right,
    rectify using ZED .conf calibration, and publish rectified images
    alongside CameraInfo messages compatible with stereo_image_proc.
    """

    def __init__(self):
        super().__init__('stereo_camera_node')

        # ------------------------------------------------------------------ #
        #  Parameters
        # ------------------------------------------------------------------ #
        self.declare_parameter('input_topic',       '/camera/dual/image_raw')
        self.declare_parameter('left_rect_topic',   '/stereo/left/image_rect')
        self.declare_parameter('right_rect_topic',  '/stereo/right/image_rect')
        self.declare_parameter('left_info_topic',   '/stereo/left/camera_info')
        self.declare_parameter('right_info_topic',  '/stereo/right/camera_info')
        self.declare_parameter('calibration_file',  '/workspace/SN15095.conf')
        self.declare_parameter('image_width',       1280)
        self.declare_parameter('image_height',      720)
        self.declare_parameter('resolution',        'HD')   # HD | FHD | VGA etc.

        p = self.get_parameter

        input_topic      = p('input_topic').get_parameter_value().string_value
        left_rect_topic  = p('left_rect_topic').get_parameter_value().string_value
        right_rect_topic = p('right_rect_topic').get_parameter_value().string_value
        left_info_topic  = p('left_info_topic').get_parameter_value().string_value
        right_info_topic = p('right_info_topic').get_parameter_value().string_value
        self.calibration_file = p('calibration_file').get_parameter_value().string_value
        self.width  = p('image_width').get_parameter_value().integer_value
        self.height = p('image_height').get_parameter_value().integer_value
        self.res    = p('resolution').get_parameter_value().string_value

        # ------------------------------------------------------------------ #
        #  QoS
        # ------------------------------------------------------------------ #
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.history     = QoSHistoryPolicy.KEEP_LAST

        # ------------------------------------------------------------------ #
        #  Pub / Sub
        # ------------------------------------------------------------------ #
        self.sub = self.create_subscription(
            Image, input_topic, self._image_callback, qos)

        self.pub_left_rect  = self.create_publisher(Image,      left_rect_topic,  qos)
        self.pub_right_rect = self.create_publisher(Image,      right_rect_topic, qos)
        self.pub_left_info  = self.create_publisher(CameraInfo, left_info_topic,  qos)
        self.pub_right_info = self.create_publisher(CameraInfo, right_info_topic, qos)

        # ------------------------------------------------------------------ #
        #  State
        # ------------------------------------------------------------------ #
        self.bridge = CvBridge()

        self.map_left_x  = None
        self.map_left_y  = None
        self.map_right_x = None
        self.map_right_y = None

        self.left_info_msg  = CameraInfo()
        self.right_info_msg = CameraInfo()

        self._load_calibration()

        self.get_logger().info(f'Subscribing to {input_topic}')
        self.get_logger().info(
            f'Publishing rectified: {left_rect_topic}, {right_rect_topic}')
        self.get_logger().info(
            f'Publishing camera_info: {left_info_topic}, {right_info_topic}')

    # ------------------------------------------------------------------ #
    #  Calibration
    # ------------------------------------------------------------------ #

    def _load_calibration(self):
        """Load calibration from a ZED .conf file and build rectification maps."""

        path = Path(self.calibration_file) if self.calibration_file else None

        if not path or not path.exists():
            self.get_logger().warn(
                'No valid calibration file — images published unrectified, '
                'CameraInfo will contain identity values.')
            return

        try:
            config = configparser.ConfigParser()
            config.read(path)

            res = self.res
            w, h = self.width, self.height

            def stereo(key, fallback=0.0):
                return float(config['STEREO'].get(key, fallback))

            def left(key, fallback=0.0):
                return float(config[f'LEFT_CAM_{res}'].get(key, fallback))

            def right(key, fallback=0.0):
                return float(config[f'RIGHT_CAM_{res}'].get(key, fallback))

            # Translation vector (baseline stored as positive mm in ZED conf)
            T = np.array([[-stereo('Baseline', 0) / 1000.0],
                        [stereo(f'TY_{res}',   0) / 1000.0],
                        [stereo(f'TZ_{res}',   0) / 1000.0]])

            # Rotation vector → matrix
            R_vec = np.array([stereo(f'RX_{res}', 0),
                              stereo(f'CV_{res}',  0),
                              stereo(f'RZ_{res}',  0)])
            R, _ = cv2.Rodrigues(R_vec)

            K_left = np.array([[left('fx'),  0,          left('cx')],
                               [0,           left('fy'), left('cy')],
                               [0,           0,          1         ]], dtype=np.float64)
            K_right = np.array([[right('fx'), 0,           right('cx')],
                                [0,           right('fy'), right('cy')],
                                [0,           0,           1          ]], dtype=np.float64)

            d_left  = np.array([[left('k1')],  [left('k2')],
                                [left('p1')],  [left('p2')],  [left('k3')]])
            d_right = np.array([[right('k1')], [right('k2')],
                                [right('p1')], [right('p2')], [right('k3')]])

            R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
                K_left, d_left, K_right, d_right,
                (w, h), R, T,
                flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
                newImageSize=(w, h)
            )

            self.map_left_x,  self.map_left_y  = cv2.initUndistortRectifyMap(
                K_left,  d_left,  R1, P1, (w, h), cv2.CV_32FC1)
            self.map_right_x, self.map_right_y = cv2.initUndistortRectifyMap(
                K_right, d_right, R2, P2, (w, h), cv2.CV_32FC1)

            # ---- Build CameraInfo templates --------------------------------
            # stereo_image_proc expects:
            #   K  — 3×3 original camera matrix (row-major, 9 elements)
            #   D  — distortion coefficients
            #   R  — rectification rotation (row-major, 9 elements)
            #   P  — 3×4 projection matrix  (row-major, 12 elements)
            #   distortion_model = 'plumb_bob'

            self.left_info_msg  = self._build_camera_info(
                w, h, K_left,  d_left,  R1, P1)
            self.right_info_msg = self._build_camera_info(
                w, h, K_right, d_right, R2, P2)

            self.get_logger().info(
                f'Loaded .conf calibration ({res}): '
                f'fx_left={left("fx"):.1f}, baseline={stereo("Baseline"):.1f} mm')

        except Exception as e:
            self.get_logger().error(f'Failed to load calibration: {e}')

    @staticmethod
    def _build_camera_info(
            width: int, height: int,
            K: np.ndarray, D: np.ndarray,
            R: np.ndarray, P: np.ndarray) -> CameraInfo:
        """Populate a sensor_msgs/CameraInfo from OpenCV calibration matrices."""
        info = CameraInfo()
        info.width  = width
        info.height = height
        info.distortion_model = 'plumb_bob'

        info.k = K.flatten().tolist()           # 9 elements
        info.d = D.flatten().tolist()           # 5 elements (k1 k2 p1 p2 k3)
        info.r = R.flatten().tolist()           # 9 elements
        info.p = P.flatten().tolist()           # 12 elements (3×4)
        return info

    # ------------------------------------------------------------------ #
    #  Main callback
    # ------------------------------------------------------------------ #

    def _image_callback(self, msg: Image):
        """Split side-by-side frame, rectify, publish images + camera_info."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        # ---- Split -------------------------------------------------------
        h, w = frame.shape[:2]
        mid = w // 2
        left_raw  = frame[:, :mid]
        right_raw = frame[:, mid:]

        # ---- Rectify (or pass through) -----------------------------------
        if self.map_left_x is not None:
            left_rect  = cv2.remap(left_raw,  self.map_left_x,  self.map_left_y,  cv2.INTER_LINEAR)
            right_rect = cv2.remap(right_raw, self.map_right_x, self.map_right_y, cv2.INTER_LINEAR)
        else:
            self.get_logger().warn(
                'No calibration maps — publishing raw splits.',
                throttle_duration_sec=5.0)
            left_rect, right_rect = left_raw, right_raw

        # ---- Determine encoding ------------------------------------------
        encoding = 'mono8' if left_rect.ndim == 2 else 'bgr8'

        # ---- Publish images ----------------------------------------------
        try:
            left_img_msg          = self.bridge.cv2_to_imgmsg(left_rect,  encoding=encoding)
            left_img_msg.header   = msg.header
            right_img_msg         = self.bridge.cv2_to_imgmsg(right_rect, encoding=encoding)
            right_img_msg.header  = msg.header

            self.pub_left_rect.publish(left_img_msg)
            self.pub_right_rect.publish(right_img_msg)
        except Exception as e:
            self.get_logger().error(f'Image publish failed: {e}')
            return

        # ---- Publish camera_info (stamp must match image for sync) -------
        left_info         = self.left_info_msg
        left_info.header  = msg.header
        right_info        = self.right_info_msg
        right_info.header = msg.header

        self.pub_left_info.publish(left_info)
        self.pub_right_info.publish(right_info)


# --------------------------------------------------------------------------- #
#  Entry point
# --------------------------------------------------------------------------- #

def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()