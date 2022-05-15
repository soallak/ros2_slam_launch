from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Stereo algorithm parameters
    stereo_algorithm_arg = DeclareLaunchArgument(
        name='stereo_algorithm', default_value='0',
        description='Stereo algorithm: Block Matching (0), Semi-Global Block Matching (1) or HW Accelerated Block Matching (2)'
    )
    approximate_sync_arg = DeclareLaunchArgument(
        name='approximate_sync', default_value='True', description="Synchronization policy for DisparityNode"
    )

    # TODO(soallak): find out how to remove the /
    left_namespace_arg = DeclareLaunchArgument(
        name="left_namespace", default_value="left/")
    right_namespace_arg = DeclareLaunchArgument(
        name="right_namespace", default_value="right/")
    image_arg = DeclareLaunchArgument(name="image", default_value="image_rect")

    container_name_arg = DeclareLaunchArgument(
        name='target_container', default_value='', description='Target container where to load components'
    )

    disparity_cmp = ComposableNode(
        package="stereo_image_proc",
        plugin="stereo_image_proc::DisparityNode",
        name="disparity",
        parameters=[
            {
                'stereo_algorithm': LaunchConfiguration('stereo_algorithm'),
                'approximate_sync': LaunchConfiguration('approximate_sync')
            }
        ],
        remappings=[
            ('left/image_rect',
             [LaunchConfiguration('left_namespace'), LaunchConfiguration('image')]),
            ('left/camera_info',
             [LaunchConfiguration('left_namespace'), 'camera_info']),
            ('right/image_rect',
             [LaunchConfiguration('right_namespace'), LaunchConfiguration('image')]),
            ('right/camera_info',
             [LaunchConfiguration('right_namespace'), 'camera_info']),
        ]

    )

    return LaunchDescription([

        stereo_algorithm_arg, approximate_sync_arg, left_namespace_arg, right_namespace_arg, image_arg, container_name_arg,
        # If a container name is not provided set the name of the container
        ComposableNodeContainer(
            condition=LaunchConfigurationEquals(container_name_arg.name, ''),
            package='rclcpp_components',
            executable='component_container',
            name='stereo_match_container',
            namespace='',
            composable_node_descriptions=[disparity_cmp],
            arguments=['--ros-args', '--log-level', 'INFO']
        ),

        # Otherwise load directly in provided container
        LoadComposableNodes(
            condition=LaunchConfigurationNotEquals(
                container_name_arg.name, ''),
            composable_node_descriptions=[disparity_cmp],
            target_container=LaunchConfiguration(container_name_arg.name),
        ),

        SetLaunchConfiguration(
            condition=LaunchConfigurationEquals(container_name_arg.name, ''),
            name=container_name_arg.name,
            value='stereo_match_container'
        )
    ])
