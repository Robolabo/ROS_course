# Objective:
# Create a ROS 2 node that subscribes to the LiDAR `/scan` topic, splits the ranges vector
# into three regions (right, front, and left), stores the measured distances at the LiDAR
# acquisition speed, and publishes fixed commands at 10 Hz both to the simulation and to the
# STM station to implement a simple Bug0-style obstacle avoidance node.

# Step 1: Import required libraries
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from stm_interfaces.msg import STMControl


# Step 2: Create the Bug0LidarSTMNode class
class Bug0LidarSTMNode(Node):
    def __init__(self):
        """
        Initialize the Bug0LidarSTMNode with one LiDAR subscriber, two publishers, and one timer.
        """
        super().__init__('bug0_lidar_stm_node')

        # Step 2.1: Fixed values for the exercise
        self.obstacle_distance = 1.0
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        self.forward_pwm = 90.0
        self.turn_pwm = 70.0

        # Step 2.2 TODO: Create a subscriber for the LiDAR topic `/scan`
        self.scan_subscriber = ...

        # Step 2.3 TODO: Create a publisher for the simulated robot velocity command
        # In Jazzy, the diff drive controller expects TwistStamped on this topic.
        self.sim_velocity_publisher = ...

        # Step 2.4 TODO: Create a publisher for the STM control topic `/stm_control`
        self.stm_control_publisher = ...

        # Step 2.5: Initialize the variables that store the latest minimum distance of each sector
        self.right_distance = float('inf')
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.scan_received = False

        # Step 2.6 TODO: Create a timer to publish the commands at 10 Hz
        # Hint: 10 Hz means a period of 0.1 seconds
        self.control_timer = ...

        # Step 2.7: Log setup details
        self.get_logger().info(
            'Bug0 LiDAR exercise node created. Complete the TODOs to enable the node.'
        )

    # Step 3: Define a helper function to get the minimum valid distance in one LiDAR section
    def get_min_distance(self, ranges_section):
        """
        Returns the minimum valid range in the selected LiDAR section.
        """
        valid_ranges = [
            distance
            for distance in ranges_section
            if math.isfinite(distance) and distance > 0.05
        ]
        return min(valid_ranges) if valid_ranges else float('inf')

    # Step 4: Define the callback function for the LiDAR messages
    def scan_callback(self, msg):
        """
        Process the incoming LiDAR vector and store the latest sector distances.
        """
        # Step 4.1: Compute how many LiDAR samples belong to each section
        total_points = len(msg.ranges)
        section_size = total_points // 3

        # Step 4.2 TODO: Split the LiDAR vector into right, front, and left sections
        # Hint:
        #   - first third  -> right side
        #   - middle third -> front
        #   - last third   -> left side
        right_ranges = ...
        front_ranges = ...
        left_ranges = ...

        # Step 4.3 TODO: Compute the minimum distance in each section
        right_distance = ...
        front_distance = ...
        left_distance = ...

        # Step 4.4 TODO: Store the minimum distances in the node variables
        self.right_distance = ...
        self.front_distance = ...
        self.left_distance = ...
        self.scan_received = True

    # Step 5: Define the timer callback that publishes the commands at 10 Hz
    def control_timer_callback(self):
        """
        Publish commands at 10 Hz using the latest distances measured by the LiDAR.
        """
        # Step 5.1: Wait until at least one LiDAR message has been received
        if not self.scan_received:
            return

        # Step 5.2: Create the output messages
        sim_cmd = TwistStamped()
        stm_cmd = STMControl()

        # Step 5.3: Add a timestamp to both messages
        stamp = self.get_clock().now().to_msg()
        sim_cmd.header.stamp = stamp
        stm_cmd.header.stamp = stamp

        # Step 5.4: Configure the STM command fields that stay constant
        stm_cmd.control_type = 2
        stm_cmd.setpoint = 0.0
        stm_cmd.kp = 0.0

        # Step 5.5 TODO: Complete the Bug0 logic
        # If there is no obstacle in front, move forward using the fixed linear speed
        # and the forward PWM value.
        # Otherwise, turn towards the side with more free space using the fixed angular
        # speed and the turning PWM value.
        if self.front_distance > self.obstacle_distance:
            sim_cmd.twist.linear.x = ...
            sim_cmd.twist.angular.z = ...
            stm_cmd.pwm = ...
        else:
            sim_cmd.twist.linear.x = ...

            if self.left_distance >= self.right_distance:
                sim_cmd.twist.angular.z = ...
                stm_cmd.pwm = ...
            else:
                sim_cmd.twist.angular.z = ...
                stm_cmd.pwm = ...

        # Step 5.6 TODO: Publish both commands
        ...

        # Step 5.7 TODO: Print the three minimum distances for debugging
        self.get_logger().info(...)


# Step 6: Define the main function
def main(args=None):
    """
    Main function to initialize and run the Bug0LidarSTMNode.
    """
    # Step 6.1: Initialize ROS 2
    rclpy.init(args=args)

    # Step 6.2: Create the node
    bug0_lidar_stm_node = Bug0LidarSTMNode()

    # Step 6.3: Keep the node running
    try:
        rclpy.spin(bug0_lidar_stm_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Step 6.4: Clean up and shutdown
        bug0_lidar_stm_node.destroy_node()
        rclpy.shutdown()


# Entry point for running the script directly
if __name__ == '__main__':
    main()
