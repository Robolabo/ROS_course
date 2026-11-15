# The Objective:
# To create a ROS 2 node that provides a service to add two integers and return the sum.

# 1. Import necessary libraries and custom message/service types
import rclpy
from rclpy.node import Node
from ....srv import ....

# 2. Define the AddTwoIntsServer class
class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        
        # 3. TODO: Create the service
        # The service name is 'add_two_ints'
        # The service type is your custom srv interface
        # The callback function is 'handle_request'
        self.service = self.create_service(... , 'add_two_ints', self.handle_request)

    # 4. TODO: Define the handle_request method to process the request and return the response
    def handle_request(self, request, response):
        # Add the parameter value to sum
        # Add a and b (first part of the service interface)
        a = request. ... 
        b = request. ...
        response.sum = a + b
        self.get_logger().info(f'Received: {a} + {b} = {response.sum}')
        
        # Return the response to the client
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    rclpy.spin(node)
    rclpy.shutdown()
