from launch import LaunchDescription  # Import LaunchDescription to define what will be launched
from launch_ros.actions import Node  # Import Node to define individual ROS nodes to launch

stm_name_space = 'group_1' # Define the name of the STM nodes

def generate_launch_description():
    """
    Function to generate the launch description for the ROS 2 launch system.
    It defines the nodes and their configurations to be launched.
    """

    # Define parameters for the control node
    parameters_control_node = {
        'master_group_id': 2,     # Default master group ID
        'slave_group_id': 1,      # Default slave group ID
    }

    # Return a LaunchDescription containing the nodes to launch
    return LaunchDescription([
        # Run the stm serial node
        Node(
            package='stm_station',         # ROS package containing the node's executable
            executable='stm_serial_node',  # Name of the executable to run
            name='stm_serial_node',        # Name to assign to the node (used in logs and debugging)
            namespace= stm_name_space,     # Namespace to group the node under
            output='screen'                # Specify where to send the node's output (console or log file)
        ),

        # Run the stm control node
        Node(
            package='stm_station',                  # ROS package containing the node's executable
            executable='stm_control_node',          # Name of the executable to run
            name='stm_control_node',                # Name to assign to the node (used in logs and debugging)
            namespace=stm_name_space,               # Namespace to group the node under
            parameters=[parameters_control_node],   # Inline parameters passed to the node
            output='screen'                         # Specify where to send the node's output (console or log file)
        )
    ])
