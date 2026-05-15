#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import json
import sys
import os

# Try to import onnxruntime for ONNX model support
try:
    import onnxruntime as ort
    ONNXRUNTIME_AVAILABLE = True
except ImportError:
    ONNXRUNTIME_AVAILABLE = False


class CrabDetectorNode(Node):
    def __init__(self):
        super().__init__('crab_detector_node')
        
        self.get_logger().info('Initializing Crab Detector Node')
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Initialize detector
        try:
            # Try to load the custom EGC ONNX model if it exists
            custom_model_path = '/workspace/src/EuropeanGreenCrabDetection/green_crab_ws/src/GroundingDINO/weights/egc_yolov8.onnx'
            if self._file_exists(custom_model_path) and ONNXRUNTIME_AVAILABLE:
                self.get_logger().info(f'Loading custom EGC ONNX model from {custom_model_path}')
                self.detector_session = ort.InferenceSession(custom_model_path)
                self.detector_type = 'onnx'
                self.get_logger().info('ONNX detector initialized successfully')
            else:
                # Use basic OpenCV detector if ONNX not available
                if not ONNXRUNTIME_AVAILABLE:
                    self.get_logger().warn('onnxruntime not available, using basic OpenCV detector')
                else:
                    self.get_logger().warn('Custom EGC ONNX model not found, using basic OpenCV detector')
                self.detector_session = None
                self.detector_type = 'opencv'
                self.get_logger().info('OpenCV detector will be used for basic object detection')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize detector: {e}')
            self.detector_session = None
            self.detector_type = 'opencv'
        
        # Subscribe to RGB image topic
        self.image_subscription = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers for output
        self.detections_publisher = self.create_publisher(
            String,
            '/egc/detections_json',
            10
        )
        
        self.annotated_image_publisher = self.create_publisher(
            Image,
            '/egc/image_annotated',
            10
        )
        
        # State management
        self.latest_frame = None
        self.latest_frame_lock = threading.Lock()
        self.frame_ready = threading.Event()
        self.should_detect = False
        
        self.get_logger().info('Crab Detector Node ready. Press SPACE to detect on the next frame.')
        
        # Start keyboard listener thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()
    
    def _file_exists(self, path):
        """Check if a file exists"""
        try:
            with open(path, 'r'):
                pass
            return True
        except:
            return False
    
    def image_callback(self, msg: Image):
        """Callback for incoming images"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            with self.latest_frame_lock:
                self.latest_frame = cv_image
                self.frame_ready.set()
            
            # If detection is requested, perform it
            if self.should_detect:
                self.should_detect = False
                self.get_logger().info('Processing frame for crab detection...')
                self.process_frame(cv_image)
        
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def process_frame(self, cv_image):
        """Process a frame through the detector"""
        try:
            if self.detector_type == 'onnx' and self.detector_session is not None:
                self._process_frame_onnx(cv_image)
            else:
                self._process_frame_opencv(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error during inference: {e}')
            import traceback
            traceback.print_exc()
    
    def _process_frame_onnx(self, cv_image):
        """Process frame using ONNX model"""
        try:
            # Prepare input for ONNX model
            h, w = cv_image.shape[:2]
            input_name = self.detector_session.get_inputs()[0].name
            
            # Resize and normalize if needed (adjust based on model requirements)
            input_image = cv2.resize(cv_image, (640, 640))
            input_image = input_image.astype(np.float32) / 255.0
            input_image = np.transpose(input_image, (2, 0, 1))
            input_image = np.expand_dims(input_image, 0)
            
            # Run inference
            outputs = self.detector_session.run(None, {input_name: input_image})
            
            # Extract detections (format depends on model, this is a generic YOLOv8 example)
            detections = self._parse_detections(outputs, cv_image)
            
            # Create annotated image
            annotated_image = self._draw_detections(cv_image, detections)
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            self.annotated_image_publisher.publish(annotated_msg)
            
            # Publish detections as JSON
            detections_json = json.dumps({'detections': detections}, default=str)
            json_msg = String()
            json_msg.data = detections_json
            self.detections_publisher.publish(json_msg)
            
            self.get_logger().info(f'ONNX Detection complete. Found {len(detections)} objects.')
        
        except Exception as e:
            self.get_logger().error(f'Error in ONNX detection: {e}')
    
    def _process_frame_opencv(self, cv_image):
        """Process frame using basic OpenCV detection (edge detection + contours)"""
        try:
            # Convert to HSV for better color separation
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Create a mask for objects (this is a simple approach)
            # Detects darker objects with higher saturation
            lower = np.array([0, 50, 50])
            upper = np.array([180, 255, 200])
            mask = cv2.inRange(hsv, lower, upper)
            
            # Apply morphological operations to clean up
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Convert contours to detections
            detections = []
            for contour in contours:
                area = cv2.contourArea(contour)
                # Filter by size to avoid noise
                if area > 100:
                    x, y, w, h = cv2.boundingRect(contour)
                    # Only detect reasonably sized objects
                    if 50 < w < cv_image.shape[1] - 50 and 50 < h < cv_image.shape[0] - 50:
                        detections.append({
                            'bbox': [float(x), float(y), float(x + w), float(y + h)],
                            'confidence': 0.6,
                            'class_name': 'potential_crab',
                            'area': float(area)
                        })
            
            # Create annotated image
            annotated_image = self._draw_detections(cv_image, detections)
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            self.annotated_image_publisher.publish(annotated_msg)
            
            # Publish detections as JSON
            detections_json = json.dumps({'detections': detections}, default=str)
            json_msg = String()
            json_msg.data = detections_json
            self.detections_publisher.publish(json_msg)
            
            self.get_logger().info(f'OpenCV Detection complete. Found {len(detections)} objects.')
        
        except Exception as e:
            self.get_logger().error(f'Error in OpenCV detection: {e}')
    
    def _parse_detections(self, outputs, original_image):
        """Parse ONNX model outputs into detection format"""
        # This is a placeholder - format depends on your specific ONNX model
        # Adjust based on your model's output structure
        detections = []
        try:
            if len(outputs) > 0:
                output = outputs[0]
                # Assuming output format: [batch, num_detections, 6] for [x, y, w, h, conf, class]
                if output.ndim == 3:
                    for det in output[0]:
                        if det[4] > 0.5:  # confidence threshold
                            x, y, w, h = det[:4]
                            h_img, w_img = original_image.shape[:2]
                            detections.append({
                                'bbox': [float(x - w/2), float(y - h/2), 
                                        float(x + w/2), float(y + h/2)],
                                'confidence': float(det[4]),
                                'class_id': int(det[5]),
                                'class_name': 'crab'
                            })
        except Exception as e:
            self.get_logger().warn(f'Could not parse ONNX outputs: {e}')
        
        return detections
    
    def _draw_detections(self, cv_image, detections):
        """Draw bounding boxes on image"""
        annotated = cv_image.copy()
        
        for detection in detections:
            try:
                bbox = detection['bbox']
                x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
                
                # Clamp to image bounds
                x1 = max(0, min(x1, cv_image.shape[1]))
                y1 = max(0, min(y1, cv_image.shape[0]))
                x2 = max(0, min(x2, cv_image.shape[1]))
                y2 = max(0, min(y2, cv_image.shape[0]))
                
                # Draw rectangle
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Add label
                conf = detection.get('confidence', 0)
                class_name = detection.get('class_name', 'object')
                label = f'{class_name}: {conf:.2f}'
                cv2.putText(annotated, label, (x1, max(y1 - 10, 20)),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            except Exception as e:
                self.get_logger().warn(f'Error drawing detection: {e}')
        
        return annotated
    
    def keyboard_listener(self):
        """Listen for keyboard input in a separate thread"""
        self.get_logger().info('Keyboard listener started. Press SPACE to trigger detection.')
        
        try:
            import keyboard
            
            def on_space():
                self.should_detect = True
                self.get_logger().info('Detection triggered!')
            
            keyboard.on_press_key('space', lambda _: on_space())
            
            # Keep the listener running
            keyboard.wait()
        
        except ImportError:
            self.get_logger().warn('keyboard module not available, trying stdin method')
            self._stdin_listener()
    
    def _stdin_listener(self):
        """Fallback stdin listener when keyboard module is not available"""
        self.get_logger().info('Using stdin for input. Type "d" and press ENTER to detect.')
        while rclpy.ok():
            try:
                # Read with timeout to avoid blocking
                import select
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    user_input = sys.stdin.readline().strip().lower()
                    if user_input == 'd':
                        self.should_detect = True
                        self.get_logger().info('Detection triggered!')
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    
    node = CrabDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

