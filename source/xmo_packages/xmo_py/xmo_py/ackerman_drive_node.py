import asyncio
import math
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

        # setup the wheel_angle dictionary
        self.wheel_angles = dict()      # new empty dictionary

        # set some physical values
        self.wheel_center_to_drive_center = 160
        self.wheel_center_width = 220

        # Set the min and max radius for the robot in millimeters
        self.radius_min = 110
        self.radius_max = 9166


    async def listener_callback(self, msg):

        self.calc_servo_angles(msg.angular)

        # front wheel steering
        await self.send_servo_msg(ServoNames.lf_steer_node)
        await self.send_servo_msg(ServoNames.rf_steer_node)

        await self.send_servo_msg(ServoNames.lf_drive_node)
        await self.send_servo_msg(ServoNames.rf_drive_node)

        # middle wheel steering
        await self.send_servo_msg(ServoNames.lm_steer_node)
        await self.send_servo_msg(ServoNames.rm_steer_node)

        await self.send_servo_msg(ServoNames.lm_drive_node)
        await self.send_servo_msg(ServoNames.rm_drive_node)

        # rear wheel steering
        await self.send_servo_msg(ServoNames.lr_steer_node)
        await self.send_servo_msg(ServoNames.rr_steer_node)

        await self.send_servo_msg(ServoNames.lr_drive_node)
        await self.send_servo_msg(ServoNames.rr_drive_node)

    async def send_servo_msg(self, topic_name):
        pub_msg = ServoPosition()
        pub_msg.angle = self.get_servo_angle(topic_name)

        # self.get_logger().info("%s sent servo_node: %s angle: %f" % (self.self_name, topic_name,  pub_msg.angle))
        publisher = self.publishDictionary_.get(topic_name)
        publisher.publish(pub_msg)

    def calc_servo_angles(self, vector_message):
        # which is the inside?
        inside_wheel = self.calc_inside_wheel_angle(0 - abs(vector_message.x))

        if vector_message.x < 0.0:
            self.calc_turn_left(vector_message, inside_wheel)
        else:
            self.calc_turn_right(vector_message, inside_wheel)             
    
    def get_servo_angle(self, servo_node_name):
        return self.wheel_angles.get(servo_node_name, 0.00)     # if it isn't in the dictionary got neutral

    def calc_turn_left(self, vector_message, inside_wheel_angle):
        # calculate the steering angles
        self.wheel_angles[ServoNames.lf_steer_node] = vector_message.x
        self.wheel_angles[ServoNames.rf_steer_node] = inside_wheel_angle

        self.wheel_angles[ServoNames.lm_steer_node] = 0.0
        self.wheel_angles[ServoNames.rm_steer_node] = 0.0
        
        self.wheel_angles[ServoNames.lr_steer_node] = -vector_message.x
        self.wheel_angles[ServoNames.rr_steer_node] = -inside_wheel_angle

        # calculate the drive wheel angles
        self.wheel_angles[ServoNames.lf_drive_node] = -vector_message.y
        self.wheel_angles[ServoNames.lm_drive_node] = -vector_message.y
        self.wheel_angles[ServoNames.lr_drive_node] = -vector_message.y

        self.wheel_angles[ServoNames.rf_drive_node] = vector_message.y
        self.wheel_angles[ServoNames.rm_drive_node] = vector_message.y
        self.wheel_angles[ServoNames.rr_drive_node] = vector_message.y   

    def calc_turn_right(self, vector_message, inside_wheel_angle):
        # calculate the steering angles
        self.wheel_angles[ServoNames.lf_steer_node] = -inside_wheel_angle
        self.wheel_angles[ServoNames.rf_steer_node] = vector_message.x

        self.wheel_angles[ServoNames.lm_steer_node] = 0.0
        self.wheel_angles[ServoNames.rm_steer_node] = 0.0

        self.wheel_angles[ServoNames.lr_steer_node] = inside_wheel_angle
        self.wheel_angles[ServoNames.rr_steer_node] = -vector_message.x

        # calculate the drive wheel angles
        self.wheel_angles[ServoNames.lf_drive_node] = -vector_message.y
        self.wheel_angles[ServoNames.lm_drive_node] = -vector_message.y
        self.wheel_angles[ServoNames.lr_drive_node] = -vector_message.y

        self.wheel_angles[ServoNames.rf_drive_node] = vector_message.y
        self.wheel_angles[ServoNames.rm_drive_node] = vector_message.y
        self.wheel_angles[ServoNames.rr_drive_node] = vector_message.y 

    def calc_inside_wheel_angle(self, outside_angle):
        # this is all going in the calc_wheel_angles method
        if outside_angle != 0.0:

            denominator_2 = self.wheel_center_to_drive_center/math.tan(math.radians(outside_angle))

            # generate the inside angle from the joystick angle
            denominator = denominator_2 - self.wheel_center_width

            inside_angle =  math.degrees(math.atan( self.wheel_center_to_drive_center/denominator ))

            self.get_logger().info("angle: %f, tan: %f, denominator: %f, denominator_2: %f, inside_angle: %f" %
                    (outside_angle, math.tan(math.radians(outside_angle)), denominator, denominator_2, inside_angle))

        else:
            inside_angle = outside_angle

        return inside_angle

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
