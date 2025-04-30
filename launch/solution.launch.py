from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory


def generate_launch_description():
    rviz_config_path = "/home/vitek/Projects/workspace/src/mpc-rbt-student/rviz/config.rviz"
    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(get_package_share_directory("mpc_rbt_simulator"),
        #                      "launch/simulation.launch.py"
        #                      )
        #     )
        # ),
        # ExecuteProcess(
        #     cmd=[
        #         'gnome-terminal', '--',
        #         'ros2', 'run', 'mpc_rbt_student', 'keyboard_control'
        #     ],
        #     output='screen'
        # ),
        Node(
            name="rviz2",
            package="rviz2",
            executable="rviz2",
            arguments=[ "-d" + rviz_config_path]
        ),
        Node(
            name="localization",
            package="mpc_rbt_student",
            executable="localization",
            output="log"
        ),
        Node(
            name="planning",
            package="mpc_rbt_student",
            executable="planning",
            output="log"
        ),
        ExecuteProcess(
            cmd=[
                'gnome-terminal', '--',
                'ros2', 'run', 'mpc_rbt_student', 'motion_control'
            ],
            output='screen'
        ),
    ])
