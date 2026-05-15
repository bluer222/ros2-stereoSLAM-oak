import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import os
import glob
import time



class ZedRepublisher(Node):
    """Capture frames from a V4L2/FFmpeg source, publish as dual half and half  images"""

    def find_camera_by_usb_name(self, usb_name, index=0):
        """Find a video device by USB device name (e.g., 'zed').
        
        Searches /dev/v4l/by-id/ for symlinks containing the USB name.
        Returns a list of all matching device paths.
        """
        try:
            devices = sorted(glob.glob('/dev/v4l/by-id/*'))
            matches = []
            for device_path in devices:
                if usb_name.lower() in device_path.lower():
                    # Resolve to the actual device node
                    real_device = os.path.realpath(device_path)
                    matches.append(real_device)
            
            if matches:
                self.get_logger().info(f'Found {len(matches)} ZED camera(s): {matches}')
                return matches
        except Exception as e:
            self.get_logger().warn(f'Error searching for USB camera: {e}')
        
        return []

    def open_camera(self, device):
        """Attempt to open a camera device and configure it.
        
        Returns True if successful, False otherwise.
        """
        try:
            cap = cv2.VideoCapture(device)
            if not cap or not cap.isOpened():
                self.get_logger().warn(f'Failed to open video device {device}')
                return False
            
            # Try to set resolution; many UVC devices ignore these settings depending on format
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            # Test if we can grab a frame
            ret, _ = cap.read()
            if not ret:
                self.get_logger().warn(f'Failed to grab test frame from {device}')
                cap.release()
                return False
            
            self.get_logger().info(f'Successfully opened video device {device} @ {self.width}x{self.height} {self.fps}fps')
            return True, cap
        except Exception as e:
            self.get_logger().warn(f'Exception opening camera {device}: {e}')
            return False

    def __init__(self):
        super().__init__('zed_republisher')

        # Parameters
        self.declare_parameter('device', '0')  # Try USB detection first; fallback to camera index
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('width', 2560)  # ZED outputs side-by-side stereo (double width)
        self.declare_parameter('height', 720)
        self.declare_parameter('zed_index', 0)  # Which ZED device if multiple are found (0=first, 1=second, etc.)
        self.declare_parameter('output', '/camera/dual/image_raw')

        self.device_param = self.get_parameter('device').get_parameter_value().string_value
        self.fps = float(self.get_parameter('fps').get_parameter_value().double_value)
        self.width = int(self.get_parameter('width').get_parameter_value().integer_value)
        self.height = int(self.get_parameter('height').get_parameter_value().integer_value)
        self.zed_index = int(self.get_parameter('zed_index').get_parameter_value().integer_value)
        self.output = self.get_parameter('output').get_parameter_value().string_value

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.history = QoSHistoryPolicy.KEEP_LAST

        self.pub = self.create_publisher(Image, self.output, qos)
        self.bridge = CvBridge()

        # Camera failover state tracking
        self.available_devices = []
        self.current_device_index = 0
        self.cap = None
        self.frame_failure_count = 0
        self.max_frame_failures = 10  # Try N times before switching devices
        self.last_rescan_time = time.time()
        self.rescan_interval = 2.0  # Rescan every 2 seconds if all devices fail

        # Initial camera setup
        self._initialize_camera()

        self.timer = self.create_timer(0.0, self.timer_callback)

    def _initialize_camera(self):
        """Initialize camera: search for ZED devices and try to open one."""
        # Close existing capture
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.cap = None

        # Find all ZED cameras via USB
        self.available_devices = self.find_camera_by_usb_name('zed')
        
        if not self.available_devices:
            self.get_logger().warn('No ZED cameras found, waiting to rescan...')
            self.last_rescan_time = time.time()
            return
        
        # Try to open a camera
        for attempt_idx in range(len(self.available_devices)):
            device = self.available_devices[attempt_idx]
            result = self.open_camera(device)
            if result and result != False:
                self.cap = result[1]
                self.current_device_index = attempt_idx
                self.frame_failure_count = 0
                return
            
        self.last_rescan_time = time.time()
        self.get_logger().error(f'Found {len(self.available_devices)} ZED camera(s) but failed to open any')

    def timer_callback(self):
        #WAIT
        current_time = time.time()
        if current_time - self.last_rescan_time < self.rescan_interval:
            return

        # Check if we need to restart camera initialization
        if not self.cap or not self.cap.isOpened():
            self.get_logger().warn('Camera is not open, reinitializing...')
            self._initialize_camera()
            return

        # cap.read() is a blocking call (it waits for the next frame from hardware)
        # This naturally throttles the loop to the camera's actual FPS
        ret, frame = self.cap.read()
        
        if not ret:
            self.frame_failure_count += 1
            self.get_logger().warn(f'Failed to grab frame ({self.frame_failure_count}/{self.max_frame_failures})')
            
            # If we've failed too many times, try switching to the next device
            if self.frame_failure_count >= self.max_frame_failures:
                self.get_logger().warn('Frame failure threshold reached, attempting to switch devices...')
                self._switch_to_next_device()
            return
        
        # Reset failure count on successful frame
        self.frame_failure_count = 0

        # Convert and publish
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        self.pub.publish(msg)

    def _switch_to_next_device(self):
        """Try the next available device, or rescan if all have been tried."""
        if not self.available_devices:
            self.get_logger().error('No available devices to switch to')
            return
        
        # Try next device
        next_index = (self.current_device_index + 1) % len(self.available_devices)
        
        if next_index == 0:
            # We've cycled through all devices, wait before rescanning
            current_time = time.time()
            self.get_logger().info('Rescanning for ZED cameras...')
            self.last_rescan_time = current_time
            self._initialize_camera()
            return
        
        device = self.available_devices[next_index]
        self.get_logger().info(f'Switching to device {next_index}: {device}')
        
        self.current_device_index = next_index

        result = self.open_camera(device)
        if result and result != False:
            if self.cap and self.cap.isOpened():
                self.cap.release()
            self.cap = result[1]
            self.frame_failure_count = 0
        else:
            self.get_logger().warn(f'Failed to open device {device}, will try next on next failure')

     
    def destroy_node(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = ZedRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
