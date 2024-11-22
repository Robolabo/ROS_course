import rclpy  # Main ROS 2 library for Python
from rclpy.node import Node  # Node class to create ROS 2 nodes
from stm_interfaces.msg import STMState  # Custom message type for STM data
from tf2_ros import TransformBroadcaster  # ROS 2 TF broadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped  # Message types for TF transform and path poses
from nav_msgs.msg import Path  # Message type for Path
import math
import numpy as np

class STMVisualizationNode(Node):
    def __init__(self):
        """
        Initialize the STMVisualizationNode with subscriber, transform broadcaster, and path publisher.
        """
        super().__init__('stm_visualization_node')
        
        # Declare and retrieve parameter for STM node group ID
        # 'group_id': Unique identifier for the STM node (default: 1)
        self.declare_parameter('group_id', 1)
        self.group_id = self.get_parameter('group_id').get_parameter_value().integer_value

        # Define topic name based on group ID for subscribing to STM data
        input_topic = f'/group_{self.group_id}/stm_state'  # Input topic for STM data

        # Initialize position, velocity, and rotation state variables
        # These values will be updated from incoming STMState messages
        self.first_reading = 1
        self.position = [0.0, 0.0, 0.0]  # x, y, z position
        self.velocity = [0.0, 0.0, 0.0]  # x, y, z velocity
        self.rotation = 0.0  # Encoder rotation (in radians)
        # Get offset acceleration
        self.offset_acceleration = np.array([0.0, 0.0, 0.0])
        self.last_time = self.get_clock().now()  # Timestamp for integration

        # Create a subscriber for the STM data topic
        self.data_subscriber = self.create_subscription(
            STMState,
            input_topic,
            self.stm_state_callback,
            10
        )

        # Initialize the TF broadcaster for broadcasting position and orientation
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize the Path publisher for visualizing the path of the STM node
        self.path_publisher = self.create_publisher(Path, 'stm_path', 10)
        self.path_msg = Path()  # Path message initialization
        self.path_msg.header.frame_id = 'world'  # Frame ID for the path

        # Log setup details for confirmation
        self.get_logger().info(f"Subscribed to {input_topic} and broadcasting TF transforms and path.")

    def stm_state_callback(self, msg):
        """
        Callback for receiving STM data, updating position and orientation, and broadcasting TF transform.
        
        :param msg: STMState message containing IMU acceleration and encoder data.
        """
        if self.first_reading == 1:
            self.first_reading = 0 
            self.offset_acceleration = np.array([msg.accel_x,  msg.accel_y,  msg.accel_z])
        # Calculate elapsed time since the last message for integration
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9  # Convert to seconds
        self.last_time = current_time

        accel_world = np.array([msg.accel_x,  msg.accel_y,  msg.accel_z])

        # Remove the offset components from the world-frame acceleration
        corrected_accel = accel_world - self.offset_acceleration


        if (np.linalg.norm(corrected_accel) > 0.5):
            # Update velocity and position from corrected acceleration data using simple integration
            self.velocity[0] += corrected_accel[0] * dt  # Update velocity in x direction
            self.velocity[1] += corrected_accel[1] * dt  # Update velocity in y direction
            self.velocity[2] += corrected_accel[2] * dt  # Update velocity in z direction
            
            self.position[0] += self.velocity[0] * dt  # Update position in x direction
            self.position[1] += self.velocity[1] * dt  # Update position in y direction
            self.position[2] += self.velocity[2] * dt  # Update position in z direction

        # Update rotation from encoder data (assuming msg.motor_encoder provides a rotation angle in degrees)
        self.rotation = math.radians(msg.motor_encoder)  # Convert encoder angle to radians

        # Broadcast the transform using TF
        self.broadcast_transform()

        # Update and publish the path
        self.update_path()

    def broadcast_transform(self):
        """
        Creates and broadcasts a TransformStamped message using the current position and rotation.
        """
        # Create a TransformStamped message for the TF transform
        transform = TransformStamped()
        
        # Set the frame details for the transform
        transform.header.stamp = self.get_clock().now().to_msg()  # Current timestamp
        transform.header.frame_id = 'world'  # Parent frame (fixed frame)
        transform.child_frame_id = f'/group_{self.group_id}/stm_station'  # Child frame for STM serial node

        # Set the translation (position) values
        transform.transform.translation.x = self.position[0]
        transform.transform.translation.y = self.position[1]
        transform.transform.translation.z = self.position[2]

        # Set the rotation (orientation) as a quaternion (z-axis rotation)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(self.rotation / 2)
        transform.transform.rotation.w = math.cos(self.rotation / 2)

        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info(f"Broadcasted TF for position {self.position} and rotation {self.rotation} radians.")

    def update_path(self):
        """
        Updates and publishes the path based on the current position and orientation.
        """
        # Create a PoseStamped message for the current position to add to the path
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'  # Frame ID consistent with path frame
        pose.pose.position.x = self.position[0]
        pose.pose.position.y = self.position[1]
        pose.pose.position.z = self.position[2]
        
        # Set the orientation as a quaternion for path visualization
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(self.rotation / 2)
        pose.pose.orientation.w = math.cos(self.rotation / 2)

        # Append the pose to the path message
        self.path_msg.poses.append(pose)
        
        # Publish the path message to visualize the path in RViz
        self.path_publisher.publish(self.path_msg)
        self.get_logger().info(f"Updated path with position {self.position}.")

def main(args=None):
    """
    Main function to initialize and run the STMVisualizationNode.
    """
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create an instance of the STMVisualizationNode
    stm_visualization_node = STMVisualizationNode()
    
    # Spin the node to keep it running and processing callbacks
    try:
        rclpy.spin(stm_visualization_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shut down the node
        stm_visualization_node.destroy_node()
        rclpy.shutdown()

# Entry point for running the script directly
if __name__ == '__main__':
    main()
