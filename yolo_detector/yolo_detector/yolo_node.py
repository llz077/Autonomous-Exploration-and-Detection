import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from yolo_detector_msgs.msg import BoundingBox
 
from cv_bridge import CvBridge
from ultralytics import YOLO
 
from ament_index_python.packages import get_package_share_directory
import os
import time
 
class YOLODetector(Node):
 
    def __init__(self):
        super().__init__('yolo_detector')
        # Create publishers
        self.publisher_ = self.create_publisher(Image, '/image_output_topic', 10)
        self.publisher_compressed_ = self.create_publisher(CompressedImage, '/image_output_topic/compressed', 10)
        self.bounding_box_publisher_ = self.create_publisher(BoundingBox, '/bounding_boxes', 10)
        
        # Declare a ROS parameter
        self.declare_parameter('rate_limit', 1.0)
        self.declare_parameter('compressed_input', True)

        # Get Parameter value
        rate_limit = self.get_parameter('rate_limit').get_parameter_value().double_value
        self.compressed_input = self.get_parameter('compressed_input').get_parameter_value().bool_value

        self.get_logger().info("Detection Rate Limit: %.2f"%rate_limit)
        self.get_logger().info("Using Compressed Input: %r"%self.compressed_input)
        
        self.last_detection_time = time.time()
        self.detection_interval = 1.0/rate_limit # minimum time between detections

        # Create Subscribers
        if self.compressed_input:
            self.subscriber = self.create_subscription(
                CompressedImage, '/image_input_topic/compressed', self.image_callback, 10)
        else:
            self.subscriber = self.create_subscription(
                Image, '/image_input_topic', self.image_callback, 10)
 
        # Get package installation directory
        pkg_dir = get_package_share_directory("yolo_detector")
        # Full path to model file (Within installation directory)
        model_path = os.path.join(pkg_dir, 'models', 'yolov8n.pt')
        # Initialize the YOLOv8 model
        self.get_logger().info("Loading YOLO model: " + model_path)
        self.model = YOLO(model_path)
        # Bridge to convert ROS <-> OpenCV
        self.bridge = CvBridge()
 
        self.get_logger().info("YOLO Detector Node Started!")
 
    def image_callback(self, msg):
        # Control detection frame rate
        if (self.detection_interval > (time.time() - self.last_detection_time)):
            # If a detection_interval has not yet passed since last detection, ignore image
            return
        self.last_detection_time = time.time()
 
        # Convert the incoming ROS image to an RGB NumPy array
        if self.compressed_input:
            frame_rgb = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='rgb8')
        else:
            frame_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        # Run YOLO inference on this RGB image
        results = self.model(frame_rgb, verbose=False)
        # results[0].plot(show=False) returns an annotated RGB image
        annotated_img_rgb = results[0].plot(show=False)
        # Convert the annotated RGB image back to a ROS Image message
        output_msg = self.bridge.cv2_to_imgmsg(annotated_img_rgb, encoding='rgb8')
        output_msg_compressed = self.bridge.cv2_to_compressed_imgmsg(annotated_img_rgb)
        # Publish the detection results
        output_msg.header = msg.header
        output_msg_compressed.header = msg.header
        self.publisher_.publish(output_msg)
        self.publisher_compressed_.publish(output_msg_compressed)

        # Publish Bounding Box
        result = results[0] # Single image input only
        for x in range (0, len(result.boxes)):
            detection_msg = BoundingBox()
            box = result.boxes[x]

            detection_msg.label = result.names[int(box.cls[0])]
            detection_msg.confidence = float(box.conf[0])
            detection_msg.x = int(box.xywh[0][0])
            detection_msg.y = int(box.xywh[0][1])
            detection_msg.width = int(box.xywh[0][2])
            detection_msg.height = int(box.xywh[0][3])

            detection_msg.image_width = result.orig_shape[1]
            detection_msg.image_height = result.orig_shape[0]

            self.bounding_box_publisher_.publish(detection_msg)
        
        # Check execution time
        if (self.detection_interval < (time.time() -self.last_detection_time)):
            # The detection process itself took a longer time than allocated detection_interval
            # The hardware is not capable enough to run detection at given rate
            self.get_logger().warn(
                "Possible : Rate limit too high! Should be less than : %.2f Hz"%(
                    (1/(time.time()-self.last_detection_time))))
 
def main(args=None):
    rclpy.init(args=args)
 
    yolo_detector = YOLODetector()
 
    rclpy.spin(yolo_detector)
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
