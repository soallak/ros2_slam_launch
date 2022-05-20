from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    stereo_algorithm_arg = DeclareLaunchArgument(
        "stereo_algorithm", description="Stereo Matcher Algorithm. 0:OpenCV LBM, 2:HWCV LBM", choices=['0', '2'], default_value='2'
    )
    # TODO: Use parameters
    disparity_cmp = ComposableNode(
        package="stereo_image_proc",
        plugin="stereo_image_proc::DisparityNode",
        name="disparity",
        parameters=[
            {"stereo_algorithm": LaunchConfiguration("stereo_algorithm")}]
    )

    container = ComposableNodeContainer(
        name="slam_edge_container",
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[disparity_cmp],
        namespace="", output="screen")

    return LaunchDescription([stereo_algorithm_arg, container])
