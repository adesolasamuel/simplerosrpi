import rclpy
from rclpy.node import Node
from gpiozero import DistanceSensor

from std_msgs.msg import Float32

sensor = DistanceSensor(echo=18, trigger=17)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('rpi_publisher_node')
        self.publisher_ = self.create_publisher(Float32, 'simplerosrpi', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.0

    def timer_callback(self):
        msg = Float32()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%f"' % msg.data)
        self.i = sensor.distance * 100


def main(args=None):
    rclpy.init(args=args)

    rpi_publisher_node = MinimalPublisher()

    rclpy.spin(rpi_publisher_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rpi_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
