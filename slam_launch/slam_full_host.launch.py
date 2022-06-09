from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    start_rviz2_arg = DeclareLaunchArgument(
        "start_rviz2", choices=["true", "false"], default_value="false")  # TODO: can we have booleans instead of strings

    start_pangolin_viewer_arg = DeclareLaunchArgument(
        "start_pangolin_viewer", choices=["true", "false"], default_value="true")

    system_type_arg = DeclareLaunchArgument(
        "system_type", description="OpenVSLAM system type", choices=["stereo", "stereo-depth"], default_value="stereo-depth"
    )

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

    return LaunchDescription([start_rviz2_arg,
                              start_pangolin_viewer_arg,
                              system_type_arg,
                              rviz2,
                              openvslam])
