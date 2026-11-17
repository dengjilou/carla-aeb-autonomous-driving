from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='control_runtime',executable='control_node',
             parameters=[{'use_sim_time':True}],output='screen'),
        Node(package='control_runtime',executable='bridge',
             parameters=[{'use_sim_time':True}],output='screen'),
    ])
