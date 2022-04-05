from ast import Subscript
import rclpy
from rclpy.node import Node

from xmo_interfaces.msg import ServoPosition
from geometry_msgs.msg import Twist
from xmo_shared_py.servo_names import ServoNames


class AckermanNode(Node):

    def __init__(self):
        super().__init__('ackerman_drive_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('publish_nodes', ['test1', 'test2'])
            ])

        self.self_name = self.get_name()
        self.param_publish_nodes = self.get_parameter('publish_nodes',)

        self.publishDictionary_ = {}

        for topicName in self.param_publish_nodes.value:
            # ackerman_drive_lf_steer_node_pub
            publishing_topic = self.self_name + "_" + topicName + '_pub'
            self.publishDictionary_.update( { topicName: self.create_publisher(ServoPosition, publishing_topic, 10)} )
            
            self.get_logger().info("self.name: %s, publishing: %s" % (self.self_name, publishing_topic))

        self.subscription = self.create_subscription(
                 Twist,
                 "cmd_vel",
                 self.listener_callback,
                 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # left side
        self.send_servo_msg(self, ServoNames.lf_steer_node, msg.angular.x)
        self.send_servo_msg(self, ServoNames.lf_drive_node, msg.angular.y)
        self.send_servo_msg(self, ServoNames.lm_steer_node, msg.angular.x)
        self.send_servo_msg(self, ServoNames.lm_drive_node, msg.angular.y)
        self.send_servo_msg(self, ServoNames.lr_steer_node, msg.angular.x)
        self.send_servo_msg(self, ServoNames.lr_drive_node, msg.angular.y)

        # right side

    def send_servo_msg(self, topic_name, angle):
            pub_msg = ServoPosition()
            pub_msg.angle = angle

            self.get_logger().info("%s sent servo_node: %s angle: %f" % (self.self_name, topic_name,  pub_msg.angle))
            publisher = self.publishDictionary_.get(topic_name)
            publisher.publish(pub_msg)

def main(args=None):
    rclpy.init(args=args)

    ackerman_drive_node = AckermanNode()

    rclpy.spin(ackerman_drive_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ackerman_drive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
