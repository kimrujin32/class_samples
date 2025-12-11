import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    table_num=2
    ld = LaunchDescription()


    kitchen_cmd = Node(
                package='example_project',
                executable='k4',
                output='screen',
                arguments=[
                    '-table_num', f'{table_num}'
                ],
            )
    ld.add_action(kitchen_cmd)


    for idx in range(1, table_num+1):
        customer_cmd = Node(
                package='example_project',
                executable='c4',
                namespace=f'table{idx}',
                output='screen',
            )
        ld.add_action(customer_cmd)
    

    return ld