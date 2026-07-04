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
        # The radar sits at a fixed pose in the world: this publishes the
        # static transform world -> radar_link (x=30, y=0, z=0, no rotation).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='radar_tf',
            arguments=['--x', '30', '--frame-id', 'world',
                       '--child-frame-id', 'radar_link'],
        ),
        Node(
            package='air_defense_sim',
            executable='radar_sensor_node',
            name='radar_sensor_node',
            parameters=[params],
            output='screen',
        ),
        # Phase 3+: estimator_node, interceptor_node
    ])
