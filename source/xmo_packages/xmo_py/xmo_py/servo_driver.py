from ast import Subscript
import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit

from xmo_interfaces.msg import ServoPosition


class ServoDriver(Node):

    def __init__(self):
        super().__init__('servo_driver')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('number_channels', 8),
                ('subscription_nodes', ['one', 'two'])
            ])

        param_number_channels = self.get_parameter('number_channels')
        param_subscription_nodes = self.get_parameter('subscription_nodes')

        for topicName in param_subscription_nodes.value:
            self.subscription = self.create_subscription(
                ServoPosition,
                topicName,
                self.listener_callback,
                10)

        self.subscription  # prevent unused variable warning

        self.get_logger().info("channels: %d, subscription_nodes: %s" %
                           (param_number_channels.value,
                            str(param_subscription_nodes.value),))

        self.kit = ServoKit(channels=param_number_channels.value)

    def listener_callback(self, msg):
        angle = float(msg.angle) + 90
        # self.get_logger().info('I heard: "%s"' % angle)
        self.kit.servo[msg.channel].angle = angle


def main(args=None):
    rclpy.init(args=args)

    servo_driver = ServoDriver()

    rclpy.spin(servo_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    servo_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
