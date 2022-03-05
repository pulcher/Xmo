from xmo_interfaces.srv import GetConfiguration

import rclpy
from rclpy.node import Node

class ConfigurationService(Node):

    def __init__(self):
        super().__init__('configuration_service')
        self.srv = self.create_service(GetConfiguration, 'get_configuration', self.get_configuration_callback)

    # def add_three_ints_callback(self, request, response):
        # response.sum = request.a + request.b + request.c
        # self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c))

        # return response

    def get_configuration_callback(self, request, response):
        response.current_configuration = "{name: blah}"
        self.get_logger().info('Incoming request\nsection_name %s' % (request.section_name))

        return response


def main(args=None):
    rclpy.init(args=args)

    configuration_service = ConfigurationService()

    rclpy.spin(configuration_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
    