import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('air_defense_sim'),
        'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='air_defense_sim',
            executable='target_sim_node',
            name='target_sim_node',
            parameters=[params],
            output='screen',
        ),
        # Phase 2+: radar_sensor_node, estimator_node, interceptor_node
    ])
