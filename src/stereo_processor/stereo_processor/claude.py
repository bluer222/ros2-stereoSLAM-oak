import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import message_filters
import cv2
import numpy as np
from pathlib import Path
import copy
import onnxruntime as ort


class StereoRGBD(Node):
    """
    Compute depth from rectified stereo image pairs using an ONNX model.

    Derives focal length and baseline from CameraInfo topics.

    Publishes:
      <depth_topic>              — 16UC1 depth image (mm)
      <depth_topic>/camera_info  — matching CameraInfo for downstream consumers
    """

    def __init__(self):
        super().__init__('stereo_rgbd')

        # ------------------------------------------------------------------ #
        #  Parameters
        # ------------------------------------------------------------------ #
        self.declare_parameter('left_rect_topic',  '/stereo/left/image_rect')
        self.declare_parameter('right_rect_topic', '/stereo/right/image_rect')
        self.declare_parameter('left_info_topic',  '/stereo/left/camera_info')
        self.declare_parameter('depth_topic',      '/stereo/depth/image_rect')
        self.declare_parameter('model_path',       '')
        self.declare_parameter('sync_slop',        0.02)
        self.declare_parameter('sync_queue_size',  10)

        p = self.get_parameter
        left_rect_topic  = p('left_rect_topic').get_parameter_value().string_value
        right_rect_topic = p('right_rect_topic').get_parameter_value().string_value
        left_info_topic  = p('left_info_topic').get_parameter_value().string_value
        self.depth_topic = p('depth_topic').get_parameter_value().string_value
        model_path       = p('model_path').get_parameter_value().string_value
        sync_slop        = p('sync_slop').get_parameter_value().double_value
        sync_queue_size  = p('sync_queue_size').get_parameter_value().integer_value

        # ------------------------------------------------------------------ #
        #  QoS
        # ------------------------------------------------------------------ #
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.history     = QoSHistoryPolicy.KEEP_LAST

        # ------------------------------------------------------------------ #
        #  Calibration state (populated from CameraInfo subscriptions)
        # ------------------------------------------------------------------ #
        self.focal_length = None
        self.baseline_m   = None
        self._left_info   = None

        self.sub_left_info = self.create_subscription(
            CameraInfo, left_info_topic, self._on_left_camera_info, qos)

        # ------------------------------------------------------------------ #
        #  Synchronized image subscribers
        # ------------------------------------------------------------------ #
        self._sub_left  = message_filters.Subscriber(self, Image, left_rect_topic,  qos_profile=qos)
        self._sub_right = message_filters.Subscriber(self, Image, right_rect_topic, qos_profile=qos)

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._sub_left, self._sub_right],
            queue_size=sync_queue_size,
            slop=sync_slop,
        )
        self._sync.registerCallback(self._on_stereo_pair)

        # ------------------------------------------------------------------ #
        #  Publishers
        # ------------------------------------------------------------------ #
        self.pub_depth      = self.create_publisher(Image,      self.depth_topic,                  qos)
        self.pub_depth_info = self.create_publisher(CameraInfo, self.depth_topic + '/camera_info', qos)

        # ------------------------------------------------------------------ #
        #  ONNX model
        # ------------------------------------------------------------------ #
        self.bridge            = CvBridge()
        self.ort_session       = None
        self.model_input_names = None

        self._load_model(model_path)

        self.get_logger().info(
            f'Subscribing to {left_rect_topic}, {right_rect_topic}, {left_info_topic}')
        self.get_logger().info(
            f'Publishing depth to {self.depth_topic} + {self.depth_topic}/camera_info')

    # ------------------------------------------------------------------ #
    #  CameraInfo callbacks
    # ------------------------------------------------------------------ #

    def _on_left_camera_info(self, msg: CameraInfo):
        if self.focal_length is not None:
            return

        fx = msg.p[0]   # P[0,0]
        if fx == 0.0:
            return

        self._left_info   = msg
        self.focal_length = fx

        right_info_topic = self.get_parameter('left_info_topic') \
                               .get_parameter_value().string_value \
                               .replace('left', 'right')

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.history     = QoSHistoryPolicy.KEEP_LAST

        self._sub_right_info = self.create_subscription(
            CameraInfo, right_info_topic, self._on_right_camera_info, qos)

        self.get_logger().info(
            f'Left CameraInfo received: fx={fx:.1f}. '
            f'Waiting for right CameraInfo on {right_info_topic}...')

    def _on_right_camera_info(self, msg: CameraInfo):
        if self.baseline_m is not None:
            return

        fx = msg.p[0]   # P[0,0]
        Tx = msg.p[3]   # P[0,3] = -fx * baseline

        if fx == 0.0 or Tx == 0.0:
            return

        self.baseline_m = abs(Tx) / fx

        self.get_logger().info(
            f'Right CameraInfo received: '
            f'baseline={self.baseline_m * 1000:.1f} mm, fx={fx:.1f} px')

        self.destroy_subscription(self._sub_right_info)

    # ------------------------------------------------------------------ #
    #  Model loading
    # ------------------------------------------------------------------ #

    def _load_model(self, model_path: str):
        if not model_path:
            self.get_logger().error('model_path parameter is required.')
            raise RuntimeError('No model_path provided.')

        if not Path(model_path).exists():
            self.get_logger().error(f'Model not found: {model_path}')
            raise RuntimeError(f'Model not found: {model_path}')

        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        self.ort_session = ort.InferenceSession(model_path, providers=providers)

        active = self.ort_session.get_providers()[0]
        self.model_input_names = [i.name for i in self.ort_session.get_inputs()]

        self.get_logger().info(
            f'Loaded ONNX model: {model_path} '
            f'(provider: {active}, inputs: {self.model_input_names})')

    # ------------------------------------------------------------------ #
    #  Disparity computation
    # ------------------------------------------------------------------ #

    def _compute_disparity(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        h, w = left.shape[:2]

        def to_tensor(img, scale=1.0):
            if img.ndim == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            else:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # this was missing
            if scale != 1.0:
                img = cv2.resize(img, (int(w * scale), int(h * scale)))
            t = img.astype(np.float32) / 255.0
            return t.transpose(2, 0, 1)[np.newaxis]

        # CREStereo-style: half-res init pair + full-res pair
        tensors = [
            to_tensor(left,  0.5),
            to_tensor(right, 0.5),
            to_tensor(left,  1.0),
            to_tensor(right, 1.0),
        ]

        n     = len(self.model_input_names)
        feeds = dict(zip(self.model_input_names, tensors[:n]))

        output = self.ort_session.run(None, feeds)[0]
        self.get_logger().info(
            f'Model output shape: {output.shape}', throttle_duration_sec=5.0)

        disp = np.abs(output).squeeze()
        if disp.ndim == 3:
            disp = disp[0]
        elif disp.ndim != 2:
            raise ValueError(f'Unexpected disparity shape: {output.shape}')

        return disp.astype(np.float32)

    # ------------------------------------------------------------------ #
    #  Depth CameraInfo builder
    # ------------------------------------------------------------------ #

    def _build_depth_info(self, header) -> CameraInfo:
        info        = copy.deepcopy(self._left_info)
        info.header = header
        info.d      = [0.0] * len(info.d)  # rectified — no distortion
        p           = list(info.p)
        p[3]        = 0.0                  # zero Tx — depth is single viewpoint
        info.p      = p
        return info

    # ------------------------------------------------------------------ #
    #  Synchronised stereo callback
    # ------------------------------------------------------------------ #

    def _on_stereo_pair(self, left_msg: Image, right_msg: Image):
        if self.focal_length is None or self.baseline_m is None:
            self.get_logger().warn(
                'CameraInfo not yet received — skipping frame.',
                throttle_duration_sec=2.0)
            return

        try:
            left  = self.bridge.imgmsg_to_cv2(left_msg,  desired_encoding='bgr8')
            right = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge failed: {e}')
            return

        try:
            disparity = self._compute_disparity(left, right)
        except Exception as e:
            self.get_logger().error(f'Disparity computation failed: {e}')
            return

        depth_mm        = np.zeros(disparity.shape, dtype=np.float32)
        valid           = disparity > 0
        depth_mm[valid] = (self.focal_length * self.baseline_m * 1000.0) / disparity[valid]
        depth_mm        = np.clip(depth_mm, 0, 65535).astype(np.uint16)

        valid_px = depth_mm[depth_mm > 0]
        if len(valid_px):
            self.get_logger().info(
                f'Depth — min: {valid_px.min()} mm  max: {valid_px.max()} mm  '
                f'mean: {valid_px.mean():.0f} mm  '
                f'valid: {100 * len(valid_px) / depth_mm.size:.1f}%',
                throttle_duration_sec=1.0)
        else:
            self.get_logger().warn('No valid depth values.', throttle_duration_sec=2.0)

        depth_msg        = self.bridge.cv2_to_imgmsg(depth_mm, encoding='16UC1')
        depth_msg.header = left_msg.header
        self.pub_depth.publish(depth_msg)

        if self._left_info is not None:
            self.pub_depth_info.publish(self._build_depth_info(left_msg.header))


def main(args=None):
    rclpy.init(args=args)
    node = StereoRGBD()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()