#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get package directories
    pkg_deepmind_bot_gazebo = get_package_share_directory('deepmind_bot_gazebo')
    description_package_name = "deepmind_bot_description"
    install_dir = get_package_prefix(description_package_name)
    gazebo_models_path = os.path.join(pkg_deepmind_bot_gazebo, 'models')

    # Configure Gazebo paths
    os.environ['GAZEBO_MODEL_PATH'] = os.pathsep.join([
        os.environ.get('GAZEBO_MODEL_PATH', ''),
        os.path.join(install_dir, 'share'),
        gazebo_models_path
    ]).strip(':')

    os.environ['GAZEBO_PLUGIN_PATH'] = os.pathsep.join([
        os.environ.get('GAZEBO_PLUGIN_PATH', ''),
        os.path.join(install_dir, 'lib')
    ]).strip(':')

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    # World file
    world_file_arg = LaunchConfiguration('world')

    # Launch gzserver WITHOUT problematic ROS plugins (they crash on macOS/RoboStack)
    gzserver = ExecuteProcess(
        cmd=['gzserver', world_file_arg, '--verbose'],
        output='screen'
    )

    # Launch gzclient
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_deepmind_bot_gazebo, 'worlds', 'perception1.world'),
            description='Path to the SDF world file'),
        gzserver,
        gzclient
    ])
