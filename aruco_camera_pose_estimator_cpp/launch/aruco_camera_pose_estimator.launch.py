from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_context import LaunchContext
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


import os


def generate_launch_description():
    ld = LaunchDescription()

    aruco_calibration_node = Node(
        package="aruco_camera_pose_estimator_cpp",
        executable="aruco_camera_pose_estimator_cpp",
        # get parameters from yaml file
        parameters=[ParameterFile(get_package_share_directory('aruco_camera_pose_estimator') + "/config/config.yaml"),
                    ParameterFile(get_package_share_directory('aruco_camera_pose_estimator') + "/config/calibration_params.yaml")
                    ]
    )
    ####################################################################
    ld.add_action(aruco_calibration_node)
    return ld
