# Objective:
# The goal of this exercise is to create a ROS2 node named 'STMControlNode' that controls the slave STM motor using Stop, Master-Slave Position Control and Close loop Impedance Control. 
# This last control approach, also closes the control loop inside of the node controling the slave STM motor adjusting the PWM based on the position and velocity errors between the master 
# and slave STM motor encoder position and velocity. So the control output looks like this: 
#       control_outut = kp * error_position + kd * error_velocity
# - Parameters are used to configure the group IDs for the master and slave nodes, services to change the control type dynamically, and a timer to control the STM at a specific frequency.
# - Services are used to change the control type and gain dynamically. 
#      -The control types are Stop (0), Position Control (1), Close loop Impedance Control (2).
# - Publishers are used to send control commands to the slave STM node.
# - Subscribers are used to receive the master and slave STM state data. 


import rclpy
from rclpy.node import Node

# 1. Import necessary libraries and custom message/service types
# TODO: Import the custom message types STMState and STMControl
# TODO: Import the custom service type STMSetControlType

# 2. Define the STMControlNode class
class STMControlNode(Node):
    def __init__(self):
        """
        Initialize the STMControlNode with parameters, publishers, subscribers, and services.
        """
        # Initialize the ROS 2 node with a unique name 'stm_control_node'
        super().__init__('stm_control_node')
        
        # 3. TODO:  Declare parameters 'master_group_id' and 'slave_group_id as set default values
        # 'master_group_id': ID for the master STM data topic, set the default value to 1
        # 'slave_group_id': ID for the slave control command topic, set the default value to 2
        self.declare_parameter('master_group_id', ...) 
        self.declare_parameter('slave_group_id', ...) 
        self.loop_frequency = 50  # Maintain value

        # 4. TODO: Retrieve parameter values from the node's configuration and assign them to instance 
        # variables
        self.master_group_id = ...  
        self.slave_group_id = ... 

        # 5. TODO: Define topic names for master and slave based on group IDs obtained from parameters
        self.master_state_topic = ...  # Master stm_state topic (/group_id/stm_state)
        self.slave_state_topic = ...   # Slave stm_state topic (/group_id/stm_state)
        self.slave_control_topic = ... # Slave stm_control topic (/group_id/stm_control)

        # 6. TODO: Create subscribers for STM state (master and slave)
        self.master_stm_state_subscriber = ...  # Subscriber for the master node state
        self.slave_stm_state_subscriber = ...   # Subscriber for the slave node state
        self.get_logger().info(f"Subscribed to {self.master_state_topic}")
        self.get_logger().info(f"Subscribed to {self.slave_state_topic}")

        # 7. TODO: Create publishers for STM control (slave)
        self.slave_stm_control_publisher = ...  # Publisher for the slave node control
        self.get_logger().info(f"Publishing to {self.stm_control_topic}")

        # 8. Define the service name ('set_control_type')
        self.control_type_service_name = 'set_control_type'

        # 9. TODO: Create a service of type 'STMSetControlType' to allow dynamic change of the control settings of the node:
        # Request:  Control types ->  0 = Stop  |  1 = Position Control
        # Request:  Control gains -> kp = Proportional gain, kd = Derivative gain
        # Response: Success status
        self.control_type_service = ...

        # 10. TODO: Create a timer for the publisher and to control the STM. 
        # !!! set the loop at 50Hz !!! (set the period to: 1 / self.loop_frequency) !!
        self.timer_control_loop = ...

        # 11. Initialize variables for storing master position
        self.master_position = 0.0
        self.slave_position = 0.0

        # 12. Initialize control type to 'Stop' (0), allowing dynamic changes via a service
        # Control types: 0 = Stop, 1 = Position Control
        self.control_type = 0

        # 13. Initialize control gains kp and kv to default value
        self.kp = 0.0
        self.kv = 0.0


    # 14. TODO: Implement callback functions
    def master_stm_state_callback(self, msg):
        """
        Callback activate when it is received data from the master STM state
        Updates master position.

        :param msg: STMState message containing motor position data.
        """
        # TODO: Update master_position with value from msg
        self.master_position = ...

    # 15. TODO: Implement callback functions
    def slave_stm_state_callback(self, msg):
        """
        Callback activate when it is received data from the slave STM state
        Updates slave position.

        :param msg: STMState message containing motor position data.
        """
        # TODO: Update master_position with value from msg
        self.slave_position = ...

    # 16. TODO: Implement the service callback
    def service_control_type_callback(self, request, response):
        """
        Service callback to set the control type.
        Updates the control_type based on the request and confirms success in the response.

        :param request: STMSetControlType.Request, containing the desired control type.
        :param response: STMSetControlType.Response, confirming the request was handled.
        :return: STMSetControlType.Response with success status.
        """
        # TODO: Update control_type based on the service request values
        # Set the control type to the requested value:
        # 0 = Stop, 1 = Position Control
        # If an invalid control type is requested, default to Stop (0)
        if (request.control_type == 0 or request.control_type == 1):
            self.control_type = request...
        else:
            self.control_type = 0
            self.get_logger().warning(f"Invalid control type: {request.control_type}. Defaulting to Stop (0)")

        # TODO: Update control kp gain based on the received requested message, 
        # ensure the value is non-negative
        if request.kp >= 0.0:
            self.kp = request...
        else:
            self.kp = 0.0
            self.get_logger().warning(f"Invalid control gain: kp={request.kp}. Defaulting to kp={self.kp}")

        # TODO: Update control kv gain based on the received requested message,
        # ensure the value is non-negative
        if request.kv >= 0.0:
            self.kv = request...
        else:
            self.kv = 0.0
            self.get_logger().warning(f"Invalid control gain: kv={request.kv}. Defaulting to kv={self.kv}")
            
        # Log the updated control type
        self.get_logger().info(f"Control type set to: {self.control_type}")
        self.get_logger().info(f"Control gain set to: kp={self.kp}")

        # Confirm the service request was handled successfully
        response.success = True
        return response

    # 17. TODO: Implement control logic in timer_control_loop callback
    def timer_callback(self):
        """
        timer_control_loop Callback to control the STM based on the current control type.
        Generates and publishes a control command handling two control modes: Stop and Position Control.
        """
        # TODO: Initialize a STMControl message that will be published to the slave node
        control_msg = STMControl()
        # Do things if control type is 0 (Stop), 1 (Position Control) or 2 (Close loop PWM Control)
        if self.control_type >=0 and self.control_type <=2:
            # TODO: Fill in the control message based on the control type and control constant received from the service request
            # and the slave setpoint position based on the master position received from the subscriber
            control_msg.control_type = ... 
            control_msg.kp = ...
            control_msg.position_setpoint = ...

            # TODO: Close loop for position control using PWM
            # TODO: Calculate the position error between the master and slave encoder positions
            error_pos = ...
            # TODO: Calculate the velocity error between the master and slave encoder velocities
            error_vel = ...
            # TODO: Calculate the control command using the gain constants
            control_command = ...
            # Saturate the control command to the valid range
            pwm_limit = 250.0  # Maximum PWM value !!
            control_command = max(-pwm_limit, min(pwm_limit, control_command))
            
            # TODO: Update the PWM control command in the STMControl message
            control_msg = ...

        else:
            # Handle unknown control types
            self.get_logger().warning(f"Unknown control type: {self.control_type}")
            # TODO: Default to stop mode
            control_msg.control_type = 0
            control_msg.kp = 0.0
            control_msg.position_setpoint = 0.0
        
        # TODO: Publish the constructed control command
        ...

        # Log the published command for debugging and monitoring
        self.get_logger().info(f"Published control command: {control_msg}")

def main(args=None):
    """
    Main function to initialize and run the STMControlNode.
    """
    # Initialize the rclpy library for ROS 2
    rclpy.init(args=args)
    
    # Instantiate the STMControlNode
    stm_control_node = STMControlNode()
    
    # Keep the node running, processing callbacks and publishing commands
    try:
        rclpy.spin(stm_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shut down the node
        stm_control_node.destroy_node()
        rclpy.shutdown()

# Entry point for running the script directly
if __name__ == '__main__':
    main()
