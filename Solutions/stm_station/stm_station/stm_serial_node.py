# Objective:
# The goal is to create a ROS 2 node that interacts with a serial device.
# It reads data from the serial device, publishes it as a custom STMState message,
# and listens for control commands via a custom STMControl message, sending those commands to the serial device.

# 1. Import necessary libraries
import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for creating ROS 2 nodes
from stm_interfaces.msg import STMState, STMControl  # Custom message types for publishing and subscribing
import serial  # Library to handle serial communication with external devices

# 2. Define the SerialPubSubNode class
class SerialPubSubNode(Node):
    def __init__(self):
        """
        Initialize the SerialPubSubNode with publishers, subscribers, and serial interface.
        """
        super().__init__('stm_serial_node', namespace='/group_1')  # Name the node 'stm_serial_node', predefine a namespace that can be changed when running or launching the node

        # 3. Configure serial communication parameters
        self.port = '/dev/ttyACM0'  # Serial port name (adjust as per your setup)
        self.baudrate = 115200  # Baud rate for serial communication
        self.loop_frequency = 1000  # Frequency (Hz) for reading data from the serial port

        # 4. Initialize the serial port
        self.serial_port = serial.Serial(self.port, self.baudrate, timeout=0.01)
        self.get_logger().info(f'Node initialized with serial port {self.port} at {self.baudrate} baud.')

        # 5. Set up a publisher to publish data read from the serial device
        self.stm_state_topic = 'stm_state'  # Topic name for publishing data
        self.stm_state_publisher = self.create_publisher(STMState, self.stm_state_topic, 10)  

        # 6. Create a timer to periodically read data from the serial port
        self.timer = self.create_timer(1 / self.loop_frequency, self.timer_read_pub_callback)

        # 7. Set up a subscriber to receive control commands and send them to the serial device
        self.stm_control_topic = 'stm_control'  # Topic name for receiving control commands
        self.stm_control_subscriber = self.create_subscription(
            STMControl,  # Message type for the subscriber
            self.stm_control_topic,  
            self.stm_control_callback,  
            10  # Queue size for incoming messages
        )

    # 8. Define the callback function for the timer
    def timer_read_pub_callback(self):
        """
        Periodically reads data from the serial port and publishes it as an STMState message.
        """
        if self.serial_port.in_waiting > 0:  # Check if data is available in the serial buffer
            try:
                # Read a line of data from the serial port
                serial_data_raw = self.serial_port.readline()
                # Decode the data to a string
                serial_data_decoded = serial_data_raw.decode('utf-8')
                # Remove leading and trailing whitespace
                serial_data = serial_data_decoded.strip()
                # Validate the data format (must start and end with braces)
                if serial_data.startswith('{') and serial_data.endswith('}'):
                    serial_data = serial_data[1:-1]  # Remove the braces
                else:
                    self.get_logger().warn('Data does not start and end with braces')
                    return
                
                # Split the data into components
                string_values = serial_data.split(',')

                # Validate the data length and convert to floats
                if len(string_values) == 8:
                    float_values = [float(value) for value in string_values]
                    
                    # Create and populate an STMState message
                    stm_state_msg = STMState()
                    stm_state_msg.header.stamp = self.get_clock().now().to_msg()    # Add the current ROS 2 time
                    stm_state_msg.header.frame_id = self.get_namespace() + "/stm_station"                    # Set the frame of reference (adjust as needed)
                    stm_state_msg.motor_encoder = float_values[0]
                    stm_state_msg.motor_velocity = float_values[1]
                    stm_state_msg.accel_x = float_values[2]
                    stm_state_msg.accel_y = float_values[3]
                    stm_state_msg.accel_z = float_values[4]
                    stm_state_msg.gyro_x = float_values[5]
                    stm_state_msg.gyro_y = float_values[6]
                    stm_state_msg.gyro_z = float_values[7]

                    # Publish the message
                    self.stm_state_publisher.publish(stm_state_msg)
                    self.get_logger().info(f'Published STMState: {stm_state_msg}')
                else:
                    self.get_logger().warn('Received data does not contain 8 fields')

            except ValueError as e:
                self.get_logger().warn(f'Error parsing serial data: {e}')

    # 9. Define the callback function for the subscriber
    def stm_control_callback(self, msg):
        """
        Callback function for receiving control commands and sending them to the serial device.
        """
        try:
            # Format the control data as a comma-separated string
            control_data = f"{{{msg.control_type}, {msg.setpoint}, {msg.kp}, {msg.pwm}}}\n"
            # Send the formatted data to the serial device
            self.serial_port.write(control_data.encode('utf-8'))
            self.get_logger().info(f'Sent control command: {control_data}')
        except Exception as e:
            self.get_logger().error(f'Error sending control command: {e}')

    # 10. Define the destroy_node method for cleanup
    def destroy_node(self):
        """
        Clean up resources when shutting down the node.
        """
        self.serial_port.close()  # Close the serial port
        super().destroy_node()  # Call the base class method

# 11. Main function to initialize and spin the node
def main(args=None):
    """
    Main function to initialize the ROS 2 node and start spinning.
    """
    rclpy.init(args=args)  # Initialize the ROS 2 Python library
    node = SerialPubSubNode()  # Create an instance of the SerialPubSubNode
    try:
        rclpy.spin(node)  # Keep the node running until interrupted
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully
    finally:
        node.destroy_node()  # Ensure resources are cleaned up
        rclpy.shutdown()  # Shut down the ROS 2 library

# 12. Entry point for the script
if __name__ == '__main__':
    main()  # Run the main function
