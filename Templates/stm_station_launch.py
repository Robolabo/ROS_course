from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory
 
 
def generate_launch_description():
    share_dir = get_package_share_directory('stm_station')
    serial_params = os.path.join(share_dir, 'config', 'serial_params.yaml')

    # Read the group ID from the YAML file and build the namespace
    params = yaml.safe_load(open(serial_params))
    namespace = f"group_{params['/**']['ros__parameters']['group_id']}"

    return LaunchDescription([
        Node(
            package='stm_station',
            executable='stm_serial_node',
            namespace=namespace, 
            name='stm_serial_node',
            parameters=[serial_params],
            output='screen',
        ),
        Node(
            package='stm_station',
            executable='stm_control_node_master_slave',
            namespace=namespace, 
            name='stm_control_node_master_slave',
            parameters=[serial_params],
            output='screen',
        )
    ])