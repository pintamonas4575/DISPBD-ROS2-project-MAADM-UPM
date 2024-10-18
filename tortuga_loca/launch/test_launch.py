
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='simulador_tortuga',
            output='screen',
        ),
        """Node(
            package='tortuga_loca',
            executable='prueba',
            name='prueba',
            output='screen', 
            parameters=[{
                'speed': 2.0,
                'log_level': "INFO"
            }]
        ),"""
    ])
