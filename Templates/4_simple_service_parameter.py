# Objective:
# To create a ROS 2 node that uses parameters in a service to add two integers 
# and an extra term defined by the parameter value and return the sum.

# 1. Import necessary libraries and custom message/service types
import rclpy
from rclpy.node import Node
from ....srv import ...

# 2. Define the AddTwoIntsServer class
class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        
        # 3. Declare the parameter 'extra_term' with a default value of 10
        self.declare_parameter('extra_term', 10)
        self.extra_term = self.get_parameter('extra_term').get_parameter_value().integer_value
        
        # 4. TODO: Create the service
        # The service name is 'add_two_ints'
        # The service type is your custom srv interface
        # The callback function is 'handle_request'
        self.service = self.create_service(... , 'add_two_ints', self.handle_request)

    # 5. TODO: Define the handle_request method to process the request and return the response
    def handle_request(self, request, response):
        # Add the parameter value to the sum
        # Update the extra term to the current value of the param:
        self.extra_term = self.get_parameter(... 
        response.sum = request.a + request.b + self.extra_term
        self.get_logger().info(f'Received: {request.a} + {request.b} + {self.extra_term} = {response.sum}')
        
        # Return the response to the client
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    rclpy.spin(node)
    rclpy.shutdown()
