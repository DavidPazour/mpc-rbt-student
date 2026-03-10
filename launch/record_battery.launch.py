from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_pubsub',
            executable='talker',
            name='MojeNodePub',
            parameters=[{
                'voltage_min': 36.0,
                'voltage_max': 42.0,
            }]
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a'],
            output='screen'
        ),
    ])