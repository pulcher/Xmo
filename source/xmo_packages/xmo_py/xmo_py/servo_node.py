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
                ('servo_type', 'bogus'),
                ('min', 0),
                ('max', 180),
                ('neutral', 90),
                ('subscription_nodes', ['test'])
            ])

        self_name = self.get_name()
        param_channel = self.get_parameter('channel')
        param_servo_type = self.get_parameter('servo_type')
        param_min = self.get_parameter('min')
        param_max = self.get_parameter('max')
        param_neutral = self.get_parameter("neutral")
        param_subscription_nodes = self.get_parameter('subscription_nodes',)

        if (param_channel.value < 0):
            raise Exception("Sorry, no channels below zero")

        if not param_servo_type.value in self.servo_types:
            raise Exception("Invalid servo type specificed")

        publishing_topic = self_name + '_pub'
        self.create_publisher(ServoPosition, publishing_topic, 10)

        for topicName in param_subscription_nodes.value:
            self.subscription = self.create_subscription(
                ServoPosition,
                topicName,
                self.listener_callback,
                10)
        self.subscription  # prevent unused variable warning

        self.get_logger().info("self_name: %s, channel: %d, servo_type: %s" %
                           (self_name,
                            param_channel.value,
                            str(param_servo_type.value),))

    def listener_callback(self, msg):
        angle = float(msg.angle) + 90

        if (angle < param_min.value):
            angle = param_min.value

        if (angle > param_max.value):
            angle = param_max.value

        msg = ServoPosition()
        msg.channel = param_channel.value
        msg.angle = angle

        self.get_logger().info('%s sent channel: %d angle: %f' % self_name,  param_channel, angle)
        self.publisher_.publish(msg)

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
