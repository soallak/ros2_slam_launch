from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals


def generate_launch_description():

    # TODO(soallak): find out how to remove the /
    left_namespace_arg = DeclareLaunchArgument(
        name="left_namespace", default_value="left")
    right_namespace_arg = DeclareLaunchArgument(
        name="right_namespace", default_value="right")
    image_arg = DeclareLaunchArgument(name="image", default_value="image_raw")

    container_name_arg = DeclareLaunchArgument(
        name='target_container', default_value='', description='Target container where to load components'
    )

    composable_nodes = [
        ComposableNode(
            package="image_proc",
            plugin="image_proc::RectifyNode",
            namespace=LaunchConfiguration(left_namespace_arg.name),
            name="left_rect",
            remappings=[("image", LaunchConfiguration(image_arg.name))]),
        ComposableNode(
            package="image_proc",
            plugin="image_proc::RectifyNode",
            namespace=LaunchConfiguration(right_namespace_arg.name),
            name="right_rect",
            remappings=[("image", LaunchConfiguration(image_arg.name))])
    ]
    return LaunchDescription([
        left_namespace_arg, right_namespace_arg, image_arg, container_name_arg,
        ComposableNodeContainer(
            condition=LaunchConfigurationEquals(container_name_arg.name, ''),
            package='rclcpp_components',
            executable='component_container',
            name='stereo_rectify_container',
            namespace='',
            composable_node_descriptions=composable_nodes,
            arguments=['--ros-args', '--log-level', 'INFO']
        ),

        # Otherwise load directly in provided container
        LoadComposableNodes(
            condition=LaunchConfigurationNotEquals(
                container_name_arg.name, ''),
            composable_node_descriptions=composable_nodes,
            target_container=LaunchConfiguration(container_name_arg.name),
        ),

        SetLaunchConfiguration(
            condition=LaunchConfigurationEquals(container_name_arg.name, ''),
            name=container_name_arg.name,
            value='stereo_rectify_container'
        )
    ])
