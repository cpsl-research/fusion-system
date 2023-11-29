import jetson.inference
import jetson.utils
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import yaml

class CameraPerception(Node):
    def __init__(self):
        super().__init__('camera_perception')
        self.bridge = CvBridge()
        self.config = self.read_yaml_config('../fusion_system_bringup/config/perception.yml')

        # Initialize the DetectNet object with parameters from YAML
        model = self.config['perception_model']['name']
        threshold = self.config['perception_model']['thresholds']['high']
        self.net = jetson.inference.detectNet(model, threshold=threshold)

        self.subscription = self.create_subscription(
            Image,
            'camera_image',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(Image, 'camera_dets', 10)


    def read_yaml_config(self, config_file):
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        return config

    def image_callback(self, msg):
        # Convert ROS image to CUDA (use jetson.utils)
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cuda_img = jetson.utils.cudaFromNumpy(img)

        # Use DetectNet to detect objects
        detections = self.net.Detect(cuda_img, overlay="box,labels,conf")

        # Convert CUDA image back to ROS Image message
        output_frame = jetson.utils.cudaToNumpy(cuda_img)
        new_msg = self.bridge.cv2_to_imgmsg(output_frame, "bgr8")

        # Publish the new message
        self.publisher_.publish(new_msg)
        self.get_logger().info('Published processed image with detections')

def main(args=None):
    rclpy.init(args=args)
    camera_perception = CameraPerception()
    rclpy.spin(camera_perception)
    camera_perception.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

