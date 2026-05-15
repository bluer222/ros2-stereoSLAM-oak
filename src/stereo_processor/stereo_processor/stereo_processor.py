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
        self.declare_parameter('depth_info_topic', '/stereo/depth/camera_info')
        self.declare_parameter('filter_threshold', 2.5) # Edge filtering threshold

        left_rect_topic  = self.get_parameter('left_rect_topic').value
        right_rect_topic = self.get_parameter('right_rect_topic').value
        rgb_info_topic   = self.get_parameter('rgb_info_topic').value
        depth_topic      = self.get_parameter('depth_topic').value
        disparity_topic  = self.get_parameter('disparity_topic').value
        model_path       = self.get_parameter('model_path').value
        sync_slop        = self.get_parameter('sync_slop').value
        self._min_disp   = self.get_parameter('min_valid_disparity').value
        depth_info_topic = self.get_parameter("depth_info_topic").value
        self.filter_threshold = self.get_parameter("filter_threshold").value

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.history     = QoSHistoryPolicy.KEEP_LAST

        # Use RGB camera info for alignment
        self.focal_length = None
        self.baseline_m   = 0.075 # OAK-D S2 fixed baseline is 75mm
        self._current_info_msg = None
        self._info_sub    = self.create_subscription(
            CameraInfo, rgb_info_topic, self._on_camera_info, qos
        )

        self._sub_left  = Subscriber(self, Image, left_rect_topic,  qos_profile=qos)
        self._sub_right = Subscriber(self, Image, right_rect_topic, qos_profile=qos)
        self._sync = ApproximateTimeSynchronizer(
            [self._sub_left, self._sub_right],
            queue_size=2,
            slop=sync_slop,
        )
        self._sync.registerCallback(self._on_stereo_pair)

        self.pub_depth     = self.create_publisher(Image,          depth_topic,     qos)
        self.pub_disparity = self.create_publisher(DisparityImage, disparity_topic, qos)
        self.pub_info = self.create_publisher(CameraInfo, depth_info_topic, qos)

        self.bridge = CvBridge()
        self.ort_session = None
        self.model_input_names = None
        self._load_model(model_path)

    def _on_camera_info(self, msg: CameraInfo):
        # Use the RGB camera's focal length (fx)
        self.focal_length = msg.p[0] 
        self.baseline_m = 0.075 # Fixed OAK-D S2 baseline
        self._current_info_msg = msg
        
        self.get_logger().info(f'Aligned to RGB frame. Focal: {self.focal_length}px')
        self.destroy_subscription(self._info_sub)

    def _load_model(self, model_path: str):
        if not model_path:
            raise SystemExit('model_path parameter is required.')

        providers = ['CUDAExecutionProvider']
        self.ort_session = ort.InferenceSession(model_path, providers=providers)
        self.model_input_names = [i.name for i in self.ort_session.get_inputs()]
        self.get_logger().info(f'Loaded ONNX model: {model_path}')

    def _compute_disparity_onnx(self, left_img, right_img) -> np.ndarray:
        # 1. CROP: 1280x800 -> 1280x720
        crop_v = (800 - 720) // 2
        left_crop = left_img[crop_v:crop_v+720, :]
        right_crop = right_img[crop_v:crop_v+720, :]
        
        def to_tensor(img, scale=1.0):
            mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
            std  = np.array([0.229, 0.224, 0.225], dtype=np.float32)
            
            h, w = img.shape[:2]
            if scale != 1.0:
                img = cv2.resize(img, (int(w * scale), int(h * scale)))
            
            if len(img.shape) == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            else:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                
            img = img.astype(np.float32) / 255.0
            img = (img - mean) / std
            return img.transpose(2, 0, 1)[np.newaxis].astype(np.float32)

        feeds = {
            self.model_input_names[0]: to_tensor(left_crop,  0.5),
            self.model_input_names[1]: to_tensor(right_crop, 0.5),
            self.model_input_names[2]: to_tensor(left_crop,  1.0),
            self.model_input_names[3]: to_tensor(right_crop, 1.0),
        }

        # Run inference
        output = self.ort_session.run(None, feeds)[0]
        disp_cropped = np.abs(output).squeeze()
        if disp_cropped.ndim == 3: 
            disp_cropped = disp_cropped[0]

        # 2. PAD: Back to 800px to match RGB image
        disp_full = np.zeros((800, 1280), dtype=np.float32)
        disp_full[crop_v:crop_v+720, :] = disp_cropped
        return disp_full

    def filter_edges(self, disparity, threshold):
        # Compute the gradient (rate of change) of the disparity
        grad_x = cv2.Sobel(disparity, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(disparity, cv2.CV_32F, 0, 1, ksize=3)
        grad_mag = np.sqrt(grad_x**2 + grad_y**2)

        # Create a mask where the change is too sharp
        # Higher threshold = keep more edges; Lower = more aggressive removal
        mask = grad_mag < threshold
        return mask
        
    def _on_stereo_pair(self, left_msg: Image, right_msg: Image):
        if self.focal_length is None:
            return

        # OAK-D mono sensors are usually 'mono8' or 'passthrough'
        left_cv  = self.bridge.imgmsg_to_cv2(left_msg,  desired_encoding='passthrough')
        right_cv = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='passthrough')

        disparity = self._compute_disparity_onnx(left_cv, right_cv)
        valid_mask = self.filter_edges(disparity, self.filter_threshold) 

        # Set invalid edge pixels to 0 disparity (which becomes 0/NaN depth)
        disparity[~valid_mask] = 0

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

        # Use the RGB info we stored earlier
        if self._current_info_msg:
            depth_info = self._current_info_msg
            depth_info.header = header # Ensure timestamp matches the depth image
            self.pub_info.publish(depth_info)

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