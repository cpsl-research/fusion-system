import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPerception(Node):
    def __init__(self):
        super().__init__('camera_perception')
        self.subscription = self.create_subscription(
            Image,
            'camera_image',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(Image, 'camera_dets', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # convert ros image to opencv image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # add a box and texts in image to test
        cv2.rectangle(cv_image, (50, 50), (200, 200), (0, 255, 0), 3)
        cv2.putText(cv_image, 'CPSL_Fusion', (60, 130), cv2.FONT_HERSHEY_SIMPLEX, 
                    1, (0, 255, 0), 2, cv2.LINE_AA)

        # convert opencv image back to ros image
        new_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")

        # publish image msg 
        self.publisher_.publish(new_msg)
        self.get_logger().info('Published processed image')

def main(args=None):
    rclpy.init(args=args)
    camera_perception = CameraPerception()
    rclpy.spin(camera_perception)
    camera_perception.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
