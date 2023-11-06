import rclpy

from .displays import Display


class DoNothingDisplay(Display):
    def __init__(self):
        super().__init__("do_nothing_display")
        self.get_logger().info('Doing nothing!')


def main(args=None):
    rclpy.init(args=args)

    display = DoNothingDisplay()

    try:
        rclpy.spin(display)
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        display.destroy_node()
        rclpy.shutdown()
