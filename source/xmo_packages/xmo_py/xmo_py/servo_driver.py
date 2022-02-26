import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit

from geometry_msgs.msg import Twist


class ServoDriver(Node):

    def __init__(self):
        super().__init__('servo_driver')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.kit = ServoKit(channels=16)

    def listener_callback(self, msg):
        angle = float(msg.linear.x) + 90
        # self.get_logger().info('I heard: "%s"' % angle)
        self.kit.servo[0].angle = angle


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
