# Import the ROS2 Python client library
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

# Step 1: Define a class for the publisher node
# Replace 'Node_ID' with a unique name for your group’s node (e.g., 'node_1' if you’re Group 1)
class MinimalSubPub(Node):
    def __init__(self):
        # Initialize the node with the custom name 'node_NICKNAME'
        super().__init__('node_pubsub_other_interfaces')  # <-- Replace 'NICKNAME' with your actual group number
        
        # Step 2: Create a publisher for your topic
        # Replace 'topic_ID' with a unique topic name (e.g., 'topic_1' for Group 1)
        self.publisher_ = self.create_publisher(BatteryState, 'topic_other_interfaces_sub', 10)  # <-- Replace 'NICKNAME'
        
        # Step 3: Set a timer to publish messages at a chosen frequency
        # Modify 'timer_period' to control how often messages are published (e.g., 2.0 for every 2 seconds)
        timer_period = 2  # <-- You can change this to another value like 1.5 or 3.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
                       
        # Step 2: Create a subscription to your group's topic
        # Replace 'topic_ID' with the topic name you want to subscribe to (e.g., 'topic_1' for Group 1)
        # The callback function 'listener_callback' will be called whenever a message is received
        self.subscription = self.create_subscription(
            BatteryState,             # Message type used by the publisher
            'topic_other_interfaces_pub',  # <-- Replace 'NICKNAME' with the specific topic name for your group
            self.listener_callback,  
            10)  # Queue size for incoming messages

        # Step 4: Initialize a counter to keep track of message numbers
        self.i = 0  # Starts counting from 0

    # Step 5: Define the callback function that will be called by the timer
    def timer_callback(self):
        # # Create a new String message
        # msg = String()
        
        # # Customize the message data with a unique message
        # # Modify the message content to include your group number and a custom message
        # msg.data = f"Group NICKNAME says hello! This is message #{self.i}"  # <-- Replace 'NICKNAME'
        msg = BatteryState()
        msg.voltage = 24.0
        msg.temperature = 18.0
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log the published message to the console
        self.get_logger().info(f'Publishing: "{msg}"')
        
        # Increment the message counter
        self.i += 1
            
    # Step 3: Define the callback function that will be called when a message is received
    def listener_callback(self, msg):
        # Log the received message to the console
        self.get_logger().info(f'Received: "{msg}"')

# Step 6: Define the main function to run the node
def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the MinimalSubPub node
    minimal_subscriber_publisher = MinimalSubPub()

    try:
        # Run the node until interrupted
        rclpy.spin(minimal_subscriber_publisher)
    except KeyboardInterrupt:
        # Gracefully handle shutdown when Ctrl+C is pressed
        pass
    finally:
        # Destroy the node and shutdown the ROS2 Python client library
        minimal_subscriber_publisher.destroy_node()
        rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
