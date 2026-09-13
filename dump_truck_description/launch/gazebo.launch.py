import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from os.path import join

def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_dump_truck = get_package_share_directory('dump_truck_description')

    robot_description_file = os.path.join(pkg_dump_truck, 'urdf', 'dump_truck.xacro')
    ros_gz_bridge_config = os.path.join(pkg_dump_truck, 'config', 'ros_gz_bridge_gazebo.yaml')
    world_sdf = os.path.join(pkg_dump_truck, 'worlds', 'iron_mine.sdf')

    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # -r runs the simulation immediately upon launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={'gz_args': f"-r --render-engine ogre2 {world_sdf}"}.items()
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                "-topic", "/robot_description",
                "-name", "dump_truck",
                "-allow_renaming", "false",
                "-x", "70.0",
                "-y", "0.0",
                "-z", "-1.5",
                "-Y", "1.570795"
            ],
            output='screen'
        )]
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        ros_gz_bridge,
        robot_state_publisher,
    ])