import asyncio
import math
from ast import Subscript
import rclpy
from rclpy.node import Node

from xmo_interfaces.msg import ServoPosition
from geometry_msgs.msg import Twist
from xmo_shared_py.servo_names import ServoNames


class CameraPositionNode(Node):

    def __init__(self):
        super().__init__('camera_position_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('publish_nodes', ['test1'])
            ])

        self.self_name = self.get_name()
        self.param_publish_nodes = self.get_parameter('publish_nodes')
        self.wheel_angles = dict()
        self.publishDictionary_ = {}

        for topicName in self.param_publish_nodes.value:
            # camera_posisiton_camera_x_node_pub
            publishing_topic = self.self_name + '_' + topicName + '_pub'
            self.publishDictionary_.update( { topicName: self.create_publisher(ServoPosition, publishing_topic, 10)} )
            
            self.get_logger().info("self.name: %s, topicName: %s, publishing: %s" % (self.self_name, topicName, publishing_topic))

        self.subscription = self.create_subscription(
                 Twist,
                 "cmd_vel",
                 self.listener_callback,
                 10)
        self.subscription  # prevent unused variable warning

    async def listener_callback(self, msg):

        self.get_logger().debug("msg.linear.x: %f, msg.linear.y: %f" % (msg.linear.x, msg.linear.y))
        self.calc_servo_angles(msg.linear)

        await self.send_servo_msg(ServoNames.camera_x_node)
        await self.send_servo_msg(ServoNames.camera_y_node)

    async def send_servo_msg(self, topic_name):
        pub_msg = ServoPosition()
        pub_msg.angle = self.get_servo_angle(topic_name)

        # self.get_logger().info("%s sent topic_name: %s angle: %f" % (self.self_name, topic_name,  pub_msg.angle))
        publisher = self.publishDictionary_.get(topic_name)
        publisher.publish(pub_msg)

    def calc_servo_angles(self, vector_message):
        self.wheel_angles[ServoNames.camera_x_node] = vector_message.x
        self.wheel_angles[ServoNames.camera_y_node] = vector_message.y
    
    def get_servo_angle(self, servo_node_name):
        return self.wheel_angles.get(servo_node_name, 0.00)     # if it isn't in the dictionary got neutral

def main(args=None):
    rclpy.init(args=args)

    camera_position_node = CameraPositionNode()

    rclpy.spin(camera_position_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_position_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
