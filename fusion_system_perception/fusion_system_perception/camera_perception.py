import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraPerception(Node):
    def __init__(self):
        super().__init__('camera_perception')
        self.subscription = self.create_subscription(
            Image,
            'camera_image',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        # 处理接收到的图像消息
        self.get_logger().info('Received image')

def main(args=None):
    rclpy.init(args=args)
    camera_perception = CameraPerception()
    rclpy.spin(camera_perception)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
