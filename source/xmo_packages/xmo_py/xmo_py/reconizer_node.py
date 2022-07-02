import asyncio
import math
from ast import Subscript
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header


class RecognizerNode(Node):

    def __init__(self):
        super().__init__('recognizer_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('publish_nodes', ['bounding_box'])
            ])

        self.self_name = self.get_name()
        self.param_publish_nodes = self.get_parameter('publish_nodes')
        self.publishDictionary_ = {}

        for topicName in self.param_publish_nodes.value:
            publishing_topic = self.self_name + '_' + topicName + '_pub'
            self.publishDictionary_.update( { topicName: self.create_publisher(String, publishing_topic, 10)} )
            
            self.get_logger().info("self.name: %s, topicName: %s, publishing: %s" % (self.self_name, topicName, publishing_topic))

        self.subscription = self.create_subscription(
                 Image,
                 "image",
                 self.listener_callback,
                 10)
        self.subscription  # prevent unused variable warning

    async def listener_callback(self, msg):
        message = "msg.encoding: %s" % (msg.encoding)
        self.get_logger().debug(message)
        
        pub_msg = String()
        pub_msg.data = message
        
        publisher = self.publishDictionary_.get("bounding_box")
        publisher.publish(pub_msg)



def main(args=None):
    rclpy.init(args=args)

    recognizer_node = RecognizerNode()

    rclpy.spin(recognizer_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    recognizer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
