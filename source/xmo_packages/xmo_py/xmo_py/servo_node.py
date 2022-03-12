from ast import Subscript
import rclpy
from rclpy.node import Node

from xmo_interfaces.msg import ServoPosition


class ServoNode(Node):
    servo_types = ["standard", "continuous"]

    def __init__(self):
        super().__init__('servo_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('channel', -1),
                ('servo_type', 'bogus')
            ])

        self_name = self.get_name()
        param_channel = self.get_parameter('channel')
        param_servo_type = self.get_parameter('servo_type')

        if (param_channel.value < 0):
            raise Exception("Sorry, no channels below zero")

        if not param_servo_type.value in self.servo_types:
            raise Exception("Invalid servo type specificed")

        publishing_topic = self_name + '_pub'
        self.create_publisher(ServoPosition, publishing_topic, 10)

        # self.subscription  # prevent unused variable warning

        self.get_logger().info("self_name: %s, channel: %d, servo_type: %s" %
                           (self_name,
                            param_channel.value,
                            str(param_servo_type.value),))

    # def listener_callback(self, msg):
    #     angle = float(msg.angle) + 90
    #     # self.get_logger().info('I heard: "%s"' % angle)
    #     self.kit.servo[msg.channel].angle = angle

def myFun(*argv):
    for arg in argv:
        print (arg)

def main(args=None):
    rclpy.init(args=args)
    
    print("blah blah blah blah")

    servo_node = ServoNode()

    rclpy.spin(servo_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    servo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
