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

    start_rviz2_arg = DeclareLaunchArgument(
        "start_rviz2", choices=["true", "false"], default_value="false")  # TODO: can we have booleans instead of strings

    start_pangolin_viewer_arg = DeclareLaunchArgument(
        "start_pangolin_viewer", choices=["true", "false"], default_value="true")

    system_type_arg = DeclareLaunchArgument(
        "system_type", description="OpenVSLAM system type", choices=["stereo", "stereo-depth"], default_value="stereo"
    )

    stereo_algorithm_arg = DeclareLaunchArgument(
        "stereo_algorithm", description="Stereo Matcher Algorithm. 0:OpenCV LBM, 2:HWCV LBM", choices=['0', '2'], default_value='0'
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
        name="SLAM_pipeline",
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            dataset_publisher_cmp, left_rect_cmp, right_rect_cmp, disparity_cmp],
        namespace="", output="screen")

    rviz2_config = os.path.join(
        get_package_share_directory("slam_launch"), "slam.rviz")

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=LaunchConfigurationEquals("start_rviz2", "true"),
        arguments=['-d', rviz2_config]
    )

    openvslam_vocab = os.path.join(
        get_package_share_directory("slam_launch"), "orb_vocab.fbow")

    # TODO: This needs to depend on launch parameters
    openvslam_config = os.path.join(
        get_package_share_directory("slam_launch"), "euroc_stereo.yaml")

    openvslam = Node(
        package="openvslam_ros",
        executable="run_slam",
        name="slam",
        arguments=["-v", openvslam_vocab, "-c", openvslam_config,
                   "-t", LaunchConfiguration("system_type")],
        parameters=[{"map_frame": "map_frame",
                     "camera_frame": "left_camera_frame", "publish_tf": True,
                     "start_pangolin_viewer": LaunchConfiguration("start_pangolin_viewer")}]
    )

    return LaunchDescription([dataset_arg,
                              dataset_period_arg,
                              dataset_type_arg,
                              start_rviz2_arg,
                              start_pangolin_viewer_arg,
                              system_type_arg,
                              stereo_algorithm_arg,
                              rviz2,
                              openvslam,
                              container])
