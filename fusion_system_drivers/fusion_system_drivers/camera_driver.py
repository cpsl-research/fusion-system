import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraDriver(Node):
    def __init__(self):
        super().__init__('camera_driver')
        self.publisher_ = self.create_publisher(Image, 'camera_image', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = Image()
        # 填充msg的数据
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing image')

def main(args=None):
    rclpy.init(args=args)
    camera_driver = CameraDriver()
    rclpy.spin(camera_driver)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
