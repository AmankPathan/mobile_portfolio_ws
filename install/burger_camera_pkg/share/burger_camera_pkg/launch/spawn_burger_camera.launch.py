from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Path to your custom URDF
    urdf_file = os.path.join(
        os.path.dirname(__file__),
        '../urdf/turtlebot3_burger_with_camera.urdf'
    )

    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_file],
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_burger_camera',
            arguments=['-file', urdf_file, '-entity', 'burger_camera'],
            output='screen'
        ),
    ])

