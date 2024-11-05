from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    awtube_driver = Node(
        package='ibt_robot_driver',
        namespace='awtube',
        executable='ibt_robot_driver'
    )
    return LaunchDescription([awtube_driver])