from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    safety_params = os.path.join(
        get_package_share_directory('safety_controller'),
        'params.yaml'
    )
    return LaunchDescription([
        Node(
            package='safety_controller',
            executable='tester',
            name='safety_tester',
            parameters=[
                safety_params,
            ]
        ),
        Node(
            package='safety_controller',
            executable='controller',
            name='safety_controller',
            parameters=[
                safety_params,
            ]
        ),
        Node(
            package='safety_controller',
            executable='logger',
            name='logger_node',
            parameters=[
                safety_params,
            ]
        )
    ])
