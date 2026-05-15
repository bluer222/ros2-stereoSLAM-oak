import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
import numpy as np

try:
    import onnxruntime as ort
except ImportError as e:
    raise SystemExit(
        'onnxruntime is required. Install with: pip install onnxruntime-gpu'
    ) from e


# Disparity values below this are treated as invalid (sub-pixel noise floor).
# CREStereo combined_iter5 reliably produces values > ~0.5 px on real structure.
MIN_VALID_DISPARITY = 0.5

# Disparity gradient threshold for confidence filtering:
# pixels where the local disparity changes faster than this (in px/px) are
# likely on an occlusion boundary or in a low-texture region — mask them out.
MAX_DISPARITY_GRADIENT = 2.0


class StereoRGBD(Node):
    """Compute depth from rectified stereo image pairs using CREStereo (ONNX)."""

    def __init__(self):
        super().__init__('stereo_rgbd')

        # ------------------------------------------------------------------ #
        #  Parameters
        # ------------------------------------------------------------------ #
        self.declare_parameter('left_rect_topic',  '/stereo/left/image_rect')
        self.declare_parameter('right_rect_topic', '/stereo/right/image_rect')
        self.declare_parameter('left_info_topic',  '/stereo/left/camera_info')
        self.declare_parameter('depth_topic',      '/stereo/depth/image_rect')
        self.declare_parameter('disparity_topic',  '/stereo/disparity')
        self.declare_parameter('model_path',       '')
        # Sync window: max time difference (seconds) between left and right frames
        self.declare_parameter('sync_slop',        0.02)
        # Confidence gate: 0.0 = keep everything, higher = stricter gradient mask
        self.declare_parameter('disparity_gradient_threshold', MAX_DISPARITY_GRADIENT)
        self.declare_parameter('min_valid_disparity', MIN_VALID_DISPARITY)

        left_rect_topic  = self.get_parameter('left_rect_topic').value
        right_rect_topic = self.get_parameter('right_rect_topic').value
        left_info_topic  = self.get_parameter('left_info_topic').value
        depth_topic      = self.get_parameter('depth_topic').value
        disparity_topic  = self.get_parameter('disparity_topic').value
        model_path       = self.get_parameter('model_path').value
        sync_slop        = self.get_parameter('sync_slop').value

        self._grad_threshold = self.get_parameter('disparity_gradient_threshold').value
        self._min_disp       = self.get_parameter('min_valid_disparity').value

        # ------------------------------------------------------------------ #
        #  QoS
        # ------------------------------------------------------------------ #
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.history     = QoSHistoryPolicy.KEEP_LAST

        # ------------------------------------------------------------------ #
        #  Calibration state (filled once from camera_info)
        # ------------------------------------------------------------------ #
        self.focal_length = None  # px, from P[0,0] of rectified left
        self.baseline_m   = None  # metres, from -P[0,3]/P[0,0] of rectified right
        self._info_sub    = self.create_subscription(
            CameraInfo, left_info_topic, self._on_camera_info, qos
        )

        # ------------------------------------------------------------------ #
        #  Synchronised stereo subscribers
        # ------------------------------------------------------------------ #
        self._sub_left  = Subscriber(self, Image, left_rect_topic,  qos_profile=qos)
        self._sub_right = Subscriber(self, Image, right_rect_topic, qos_profile=qos)
        self._sync = ApproximateTimeSynchronizer(
            [self._sub_left, self._sub_right],
            queue_size=5,
            slop=sync_slop,
        )
        self._sync.registerCallback(self._on_stereo_pair)

        # ------------------------------------------------------------------ #
        #  Publishers
        # ------------------------------------------------------------------ #
        self.pub_depth     = self.create_publisher(Image,          depth_topic,     qos)
        self.pub_disparity = self.create_publisher(DisparityImage, disparity_topic, qos)

        self.bridge = CvBridge()

        # ------------------------------------------------------------------ #
        #  ONNX model
        # ------------------------------------------------------------------ #
        self.ort_session       = None
        self.model_input_names = None
        self._load_model(model_path)

    # ------------------------------------------------------------------ #
    #  Camera info — only need one message; unsubscribe after
    # ------------------------------------------------------------------ #

    def _on_camera_info(self, msg: CameraInfo):
        # P is a 3x4 row-major matrix.
        # For the rectified LEFT camera:  P[0,0] = f,  P[0,3] = 0
        # For the rectified RIGHT camera: P[0,3] = -f * baseline  (negative)
        # We only subscribe to the left info topic.  baseline comes from the
        # right camera's P[0,3], which is NOT in this message.
        # The standard approach when you only have the left info: read
        # baseline from the right camera_info topic too.  However, many ZED /
        # OAK-D drivers publish baseline_m via the left info's P matrix being
        # identical to the right's P but with Tx=-f*B filled in on the left
        # topic as well. Guard for both cases.

        P = np.array(msg.p).reshape(3, 4)
        self.focal_length = P[0, 0]

        # Tx in the left camera projection matrix is 0 for a rectified pair;
        # a non-zero value here means the driver is encoding the full stereo
        # geometry in the left info (some drivers do this).
        if abs(P[0, 3]) > 1e-6 and self.focal_length > 0:
            # baseline = -Tx / f
            self.baseline_m = -P[0, 3] / self.focal_length
        else:
            # Need to wait for right camera_info — subscribe to it once.
            self.get_logger().info(
                'Left camera_info has Tx=0; subscribing to right camera_info for baseline.'
            )
            qos = QoSProfile(depth=10)
            qos.reliability = QoSReliabilityPolicy.RELIABLE
            qos.history     = QoSHistoryPolicy.KEEP_LAST
            right_info_topic = self.get_parameter('left_info_topic').value.replace(
                'left', 'right'
            )
            self._right_info_sub = self.create_subscription(
                CameraInfo, right_info_topic, self._on_right_camera_info, qos
            )

        if self.baseline_m is not None:
            self.get_logger().info(
                f'Calibration ready: f={self.focal_length:.1f}px  '
                f'baseline={self.baseline_m * 1000:.1f}mm'
            )
            # Unsubscribe — we have everything we need
            self.destroy_subscription(self._info_sub)

    def _on_right_camera_info(self, msg: CameraInfo):
        P = np.array(msg.p).reshape(3, 4)
        f = P[0, 0]
        tx = P[0, 3]  # should be -f * baseline for the right camera
        if f > 0 and abs(tx) > 1e-6:
            self.baseline_m = (-tx / f)
            self.get_logger().info(
                f'Calibration ready: f={self.focal_length:.1f}px  '
                f'baseline={self.baseline_m * 1000:.1f}mm'
            )
            self.destroy_subscription(self._right_info_sub)

    # ------------------------------------------------------------------ #
    #  Model loading
    # ------------------------------------------------------------------ #

    def _load_model(self, model_path: str):
        if not model_path:
            raise SystemExit('model_path parameter is required.')

        from pathlib import Path
        if not Path(model_path).exists():
            raise SystemExit(f'Model not found: {model_path}')

        # GPU only — no CPU fallback by design
        providers = ['CUDAExecutionProvider']
        self.ort_session = ort.InferenceSession(model_path, providers=providers)

        active = self.ort_session.get_providers()[0]
        if active != 'CUDAExecutionProvider':
            raise SystemExit(
                f'CUDA execution provider not available — active: {active}. '
                'Install onnxruntime-gpu and ensure CUDA is accessible.'
            )

        self.model_input_names = [i.name for i in self.ort_session.get_inputs()]
        self.get_logger().info(
            f'Loaded ONNX model: {model_path}\n'
            f'  Provider : {active}\n'
            f'  Inputs   : {self.model_input_names}'
        )

        # Validate we have exactly 4 inputs for raft stereo
        if len(self.model_input_names) != 2:
            self.get_logger().warn(
                f'Expected 4 model inputs (half-L, half-R, full-L, full-R), '
                f'got {len(self.model_input_names)}. Proceeding anyway.'
            )

    # ------------------------------------------------------------------ #
    #  Disparity computation
    # ------------------------------------------------------------------ #

    def _compute_disparity_onnx(self, left_bgr: np.ndarray, right_bgr: np.ndarray) -> np.ndarray:
        """Run CREStereo combined_iter5 inference.

        Returns a float32 disparity map in pixels (same size as input).
        Invalid pixels are set to 0.
        """
        h, w = left_bgr.shape[:2]
        
        def to_tensor(img, scale=1.0):
            mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
            std  = np.array([0.229, 0.224, 0.225], dtype=np.float32)
            if scale != 1.0:
                img = cv2.resize(img, (int(w * scale), int(h * scale)),
                                interpolation=cv2.INTER_LINEAR)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            img = (img - mean) / std
            return img.transpose(2, 0, 1)[np.newaxis]

        # CREStereo combined_iter5: [half_left, half_right, full_left, full_right]
        feeds = {
            self.model_input_names[0]: to_tensor(left_bgr,  1.0),
            self.model_input_names[1]: to_tensor(right_bgr, 1.0),
        }

        output = self.ort_session.run(None, feeds)[0]  # (1, 1, H, W) or (1, H, W)

        disp = np.abs(output.squeeze())
        if disp.ndim == 3:
            disp = disp[0]
        elif disp.ndim != 2:
            raise ValueError(f'Unexpected disparity output shape: {output.shape}')

        h, w = left_bgr.shape[:2]
        disp = cv2.resize(disp, (w, h), interpolation=cv2.INTER_LINEAR)
        return disp.astype(np.float32)

    # ------------------------------------------------------------------ #
    #  Confidence / validity filtering
    # ------------------------------------------------------------------ #
    '''
    def _build_valid_mask(self, disp: np.ndarray) -> np.ndarray:
        """Return a boolean mask of pixels worth keeping.

        Two-stage gate:
          1. Hard floor — sub-pixel noise (< min_valid_disparity)
          2. Disparity gradient — occlusion edges and textureless blobs
             produce implausibly steep gradients; mask those out.
        """
        # Stage 1: absolute floor
        mask = disp >= self._min_disp

        # Stage 2: gradient-based confidence
        # Compute gradient magnitude of the disparity map.
        # Large gradients at true depth discontinuities are fine; the problem
        # is *extended* low-texture regions where the network produces slowly
        # drifting, unreliable disparity.  We use a mild Sobel rather than a
        # pixel-to-pixel diff to avoid punishing legitimate edges.
        sobel_x = cv2.Sobel(disp, cv2.CV_32F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(disp, cv2.CV_32F, 0, 1, ksize=3)
        grad_mag = np.sqrt(sobel_x**2 + sobel_y**2)

        # Dilate the high-gradient mask to erode valid pixels near edges
        # (handles occlusion boundaries where disparity is ambiguous)
        high_grad = (grad_mag > self._grad_threshold).astype(np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        high_grad_dilated = cv2.dilate(high_grad, kernel)

        mask &= high_grad_dilated == 0
        return mask'''

    def _build_valid_mask(self, disp: np.ndarray) -> np.ndarray:
        return disp >= self._min_disp

    # ------------------------------------------------------------------ #
    #  Stereo callback (synchronised)
    # ------------------------------------------------------------------ #

    def _on_stereo_pair(self, left_msg: Image, right_msg: Image):
        if self.focal_length is None or self.baseline_m is None:
            self.get_logger().warn(
                'Calibration not yet received — skipping frame.',
                throttle_duration_sec=2.0,
            )
            return

        left_bgr  = self.bridge.imgmsg_to_cv2(left_msg,  desired_encoding='bgr8')
        right_bgr = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')

        disparity = self._compute_disparity_onnx(left_bgr, right_bgr)

        # Add this after computing disparity
        h, w = disparity.shape
        center_strip = disparity[h//3:2*h//3, :]  # middle third of image
        self.get_logger().info(
            f'Center strip — min={center_strip.min():.3f}  '
            f'p5={np.percentile(center_strip, 5):.3f}  '
            f'p25={np.percentile(center_strip, 25):.3f}',
            throttle_duration_sec=1.0,
        )
        self.get_logger().info(
            f'Raw disparity — min={disparity.min():.2f}  max={disparity.max():.2f}  '
            f'mean={disparity.mean():.2f}',
            throttle_duration_sec=1.0,
        )

        valid = self._build_valid_mask(disparity)

        # ---- depth image (uint16, millimetres) ----
        depth_mm = np.zeros(disparity.shape, dtype=np.float32)
        depth_mm[valid] = (self.focal_length * self.baseline_m * 1000.0) / disparity[valid]
        depth_mm = np.clip(depth_mm, 0, 65535).astype(np.uint16)

        valid_depths = depth_mm[valid]
        if len(valid_depths):
            self.get_logger().info(
                f'Depth — min={valid_depths.min()}mm  max={valid_depths.max()}mm  '
                f'mean={valid_depths.mean():.0f}mm  '
                f'valid={100 * valid.sum() / valid.size:.1f}%',
                throttle_duration_sec=1.0,
            )
        else:
            self.get_logger().warn('No valid depth values after filtering.', throttle_duration_sec=2.0)

        # Use left stamp (the synchronised pair shares a common logical timestamp)
        header = left_msg.header

        depth_ros = self.bridge.cv2_to_imgmsg(depth_mm, encoding='16UC1')
        depth_ros.header = header
        self.pub_depth.publish(depth_ros)

        # ---- disparity image (stereo_msgs/DisparityImage) ----
        disp_filtered = np.where(valid, disparity, 0.0).astype(np.float32)
        disp_ros                    = DisparityImage()
        disp_ros.header             = header
        disp_ros.image              = self.bridge.cv2_to_imgmsg(disp_filtered, encoding='32FC1')
        disp_ros.image.header       = header
        disp_ros.f                  = float(self.focal_length)
        disp_ros.t                  = float(self.baseline_m)
        disp_ros.min_disparity      = float(self._min_disp)
        disp_ros.max_disparity      = float(disparity.max())
        disp_ros.delta_d            = 1.0 / 16.0  # sub-pixel precision
        self.pub_disparity.publish(disp_ros)


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