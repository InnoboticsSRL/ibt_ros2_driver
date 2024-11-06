from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    awtube_driver = Node(
        package='ibt_robot_driver',
        namespace='awtube',
        executable='ibt_robot_driver',
        parameters=[{'url': 'ws://localhost:9001/ws'},
                    {'use_fake': True}],
    )
    return LaunchDescription([awtube_driver])