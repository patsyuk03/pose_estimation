import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("add_gripper", default_value="true"))
    ld.add_action(DeclareLaunchArgument("robot_ip", default_value="192.168.1.220"))
    ld.add_action(DeclareLaunchArgument("use_fake_hardware", default_value="false"))

    add_gripper = LaunchConfiguration("add_gripper")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch/rs_launch.py')
        ),
        launch_arguments={
            'align_depth.enable': 'true'
        }.items()
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pose_estimation'), 'launch/xarm.launch.py')
        ),
        launch_arguments={
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'add_gripper': add_gripper,
        }.items()
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('find_object_2d'), 'launch/find_object_3d.launch.py')
        ),
        launch_arguments={
            'rgb_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
        }.items()
    ))

    ld.add_action(Node(
        package='pose_estimation', 
        executable='transform_pose', 
        output='screen', 
        parameters=[{'target_frame_id': 'link_base'}],
    ))

    return ld