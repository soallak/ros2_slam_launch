from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    dataset_arg = DeclareLaunchArgument(
        "dataset_path", description="Path to the dateset to use")
    dataset_period_arg = DeclareLaunchArgument(
        "dataset_period", description="Publishing period in milliseconds")
    dataset_type_arg = DeclareLaunchArgument(
        "dataset_type", description="Type of the dataset", choices=["euroc"])

    stereo_algorithm_arg = DeclareLaunchArgument(
        "stereo_algorithm", description="Stereo Matcher Algorithm. 0:OpenCV LBM, 2:HWCV LBM", choices=['0', '2'], default_value='2'
    )

    # TODO: Adapt this depending on the dataset type
    dataset_publisher_cmp = ComposableNode(package="data_publisher", plugin="simulation::EurocPublisher", parameters=[{
        "dataset_path": LaunchConfiguration(dataset_arg.name),
        "period_ms": LaunchConfiguration(dataset_period_arg.name),
        "frame_id": "left_camera_frame"
    }])

    left_rect_cmp = ComposableNode(
        package="image_proc",
        plugin="image_proc::RectifyNode",
        namespace="/left",
        name="left_rect",
        remappings=[("image", "image_raw")])
    right_rect_cmp = ComposableNode(
        package="image_proc",
        plugin="image_proc::RectifyNode",
        namespace="/right",
        name="right_rect",
        remappings=[("image", "image_raw")])

    # TODO: Use parameters
    disparity_cmp = ComposableNode(
        package="stereo_image_proc",
        plugin="stereo_image_proc::DisparityNode",
        name="disparity",
        parameters=[
            {"stereo_algorithm": LaunchConfiguration("stereo_algorithm")}]
    )

    container = ComposableNodeContainer(
        name="slam_full_edge",
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            dataset_publisher_cmp, left_rect_cmp, right_rect_cmp, disparity_cmp],
        namespace="", output="screen")

    return LaunchDescription([dataset_arg,
                              dataset_period_arg,
                              dataset_type_arg,
                              stereo_algorithm_arg,
                              container])
