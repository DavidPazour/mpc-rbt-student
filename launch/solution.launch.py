import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'configNew.rviz')

    return LaunchDescription([

        # Lokalizační node
        Node(
            package='mpc_rbt_student',
            executable='localization',
            name='localization_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # Planovaci node
        Node(
            package='mpc_rbt_student',
            executable='planning',
            name='planning_node',
            output='screen',
            parameters=[{'use_sim_time': True}], # Důležité pro synchronizaci s časem simulace (Gazebo)
        ),

        # motion control node
        Node(
            package='mpc_rbt_student',
            executable='motion_control',
            name='motion_control_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # RViz s předpřipraveným configem
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

    ])
