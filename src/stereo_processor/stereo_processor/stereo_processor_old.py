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

MIN_VALID_DISPARITY = 0.5

class StereoRGBD(Node):
    """Compute depth from OAK-D S2 PoE aligned stereo pairs using CREStereo."""

    def __init__(self):
        super().__init__('stereo_rgbd')

        # Updated topics for OAK-D S2 PoE
        self.declare_parameter('left_rect_topic',  '/oak/stereo/left/image_rect')
        self.declare_parameter('right_rect_topic', '/oak/stereo/right/image_rect')
        self.declare_parameter('rgb_info_topic',   '/oak/rgb/camera_info')
        self.declare_parameter('depth_topic',      '/stereo/depth/image_rect')
        self.declare_parameter('disparity_topic',  '/stereo/disparity')
        self.declare_parameter('model_path',       '')
        self.declare_parameter('sync_slop',        0.05) # Increased slop for PoE overhead
        self.declare_parameter('min_valid_disparity', MIN_VALID_DISPARITY)

        left_rect_topic  = self.get_parameter('left_rect_topic').value
        right_rect_topic = self.get_parameter('right_rect_topic').value
        rgb_info_topic   = self.get_parameter('rgb_info_topic').value
        depth_topic      = self.get_parameter('depth_topic').value
        disparity_topic  = self.get_parameter('disparity_topic').value
        model_path       = self.get_parameter('model_path').value
        sync_slop        = self.get_parameter('sync_slop').value
        self._min_disp   = self.get_parameter('min_valid_disparity').value

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.history     = QoSHistoryPolicy.KEEP_LAST

        # Use RGB camera info for alignment
        self.focal_length = None
        self.baseline_m   = 0.075 # OAK-D S2 fixed baseline is 75mm
        self._info_sub    = self.create_subscription(
            CameraInfo, rgb_info_topic, self._on_camera_info, qos
        )

        self._sub_left  = Subscriber(self, Image, left_rect_topic,  qos_profile=qos)
        self._sub_right = Subscriber(self, Image, right_rect_topic, qos_profile=qos)
        self._sync = ApproximateTimeSynchronizer(
            [self._sub_left, self._sub_right],
            queue_size=10,
            slop=sync_slop,
        )
        self._sync.registerCallback(self._on_stereo_pair)

        self.pub_depth     = self.create_publisher(Image,          depth_topic,     qos)
        self.pub_disparity = self.create_publisher(DisparityImage, disparity_topic, qos)

        self.bridge = CvBridge()
        self.ort_session = None
        self.model_input_names = None
        self._load_model(model_path)

    def _on_camera_info(self, msg: CameraInfo):
        # Read focal length from the RGB projection matrix to match perspective
        P = np.array(msg.p).reshape(3, 4)
        self.focal_length = P[0, 0]
        
        if self.focal_length > 0:
            self.get_logger().info(
                f'RGB Alignment Ready: f={self.focal_length:.1f}px '
                f'baseline={self.baseline_m * 1000:.1f}mm'
            )
            self.destroy_subscription(self._info_sub)

    def _load_model(self, model_path: str):
        if not model_path:
            raise SystemExit('model_path parameter is required.')

        providers = ['CUDAExecutionProvider']
        self.ort_session = ort.InferenceSession(model_path, providers=providers)
        self.model_input_names = [i.name for i in self.ort_session.get_inputs()]
        self.get_logger().info(f'Loaded ONNX model: {model_path}')

    def _compute_disparity_onnx(self, left_img, right_img) -> np.ndarray:
        h, w = left_img.shape[:2]
        
        def to_tensor(img, scale=1.0):
            mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
            std  = np.array([0.229, 0.224, 0.225], dtype=np.float32)
            
            if scale != 1.0:
                img = cv2.resize(img, (int(w * scale), int(h * scale)),
                                interpolation=cv2.INTER_LINEAR)
            
            # Broadcast Monochrome to 3-channel RGB for CREStereo
            if len(img.shape) == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            elif img.shape[2] == 3:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            img = img.astype(np.float32) / 255.0
            img = (img - mean) / std
            return img.transpose(2, 0, 1)[np.newaxis]

        feeds = {
            self.model_input_names[0]: to_tensor(left_img,  0.5),
            self.model_input_names[1]: to_tensor(right_img, 0.5),
            self.model_input_names[2]: to_tensor(left_img,  1.0),
            self.model_input_names[3]: to_tensor(right_img, 1.0),
        }

        output = self.ort_session.run(None, feeds)[0]
        disp = np.abs(output).squeeze()
        if disp.ndim == 3: disp = disp[0]
        return disp.astype(np.float32)

    def _on_stereo_pair(self, left_msg: Image, right_msg: Image):
        if self.focal_length is None:
            return

        # OAK-D mono sensors are usually 'mono8' or 'passthrough'
        left_cv  = self.bridge.imgmsg_to_cv2(left_msg,  desired_encoding='passthrough')
        right_cv = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='passthrough')

        disparity = self._compute_disparity_onnx(left_cv, right_cv)
        valid = disparity >= self._min_disp

        # Depth aligned to the RGB frame using the RGB focal length
        depth_mm = np.zeros(disparity.shape, dtype=np.float32)
        depth_mm[valid] = (self.focal_length * self.baseline_m * 1000.0) / disparity[valid]
        depth_mm = np.clip(depth_mm, 0, 65535).astype(np.uint16)

        header = left_msg.header
        # Important: frame_id should ideally be set to the RGB optical frame in launch
        depth_ros = self.bridge.cv2_to_imgmsg(depth_mm, encoding='16UC1')
        depth_ros.header = header
        self.pub_depth.publish(depth_ros)

        disp_ros = DisparityImage()
        disp_ros.header = header
        disp_ros.image = self.bridge.cv2_to_imgmsg(np.where(valid, disparity, 0.0).astype(np.float32), encoding='32FC1')
        disp_ros.f = float(self.focal_length)
        disp_ros.t = float(self.baseline_m)
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