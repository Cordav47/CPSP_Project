import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    risk_image_publisher = Node(
            package='drone_project',
            executable='risk_image_publisher',
            name='risk_image_publisher',
            parameters = [{'scenario': 0}]
        )
    
    standard_image_publisher = Node(
            package='drone_project',
            executable='standard_image_publisher',
            name='standard_image_publisher',
            parameters = [{'scenario': 0}]
        )
    
    image_processor = Node(
            package='drone_project',
            executable='image_processor',
            name='image_processor'
        )
    
    goal_sender = Node(
            package='drone_project',
            executable='goal_sender',
            name='goal_sender'
        )
    
    offboard_control = Node(
        package='drone_project',
        executable='offboard_control',
        name = 'offboard_control',
        #parameters = [{'radius': 10.0}, {'altitude': 5.0}, {'omega': 0.5}]
    )
    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(risk_image_publisher)
    ld.add_action(standard_image_publisher)
    ld.add_action(image_processor)
    ld.add_action(goal_sender)
    ld.add_action(offboard_control)

    return ld
