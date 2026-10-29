import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    package_share = get_package_share_directory(
        'Sensors-and-Control-Integrated-Project'
    )
    world = os.path.join(package_share, 'worlds', 'snc_test_new_env.sdf')
    robot_description = os.path.join(
        package_share, 'urdf', 'turtlebot3_burger.urdf'
    )
    robot_model = os.path.join(package_share, 'sdf', 'turtlebot3_model.sdf')
    aruco_detector = os.path.join(
        package_share, 'scripts', 'aruco_detector.py'
    )

    robot_description_xml = xacro.process_file(robot_description).toxml()

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[
                os.environ.get('GAZEBO_MODEL_PATH', ''),
                os.pathsep, os.path.join(package_share, 'models'),
                os.pathsep, '/opt/ros/humble/share/turtlebot3_gazebo/models',
                os.pathsep, '/usr/share/gazebo-11/models',
                os.pathsep, '/opt/ros/humble/share',
            ],
        ),
        SetEnvironmentVariable(
            name='GAZEBO_RESOURCE_PATH',
            value=[
                os.environ.get('GAZEBO_RESOURCE_PATH', ''),
                os.pathsep, os.path.join(package_share, 'models'),
                os.pathsep, '/usr/share/gazebo-11',
                os.pathsep, '/usr/share/gazebo-11/media',
            ],
        ),
        SetEnvironmentVariable(
            name='GAZEBO_PLUGIN_PATH',
            value=[
                os.environ.get('GAZEBO_PLUGIN_PATH', ''),
                os.pathsep, '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins',
            ],
        ),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_DATABASE_URI',
            value='',
        ),
        ExecuteProcess(
            cmd=[
                'gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so', world,
            ],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_xml,
                'use_sim_time': True,
            }],
            output='screen',
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot3', '-file', robot_model,
                '-x', '-2.1', '-y', '0.0', '-z', '0.01',
            ],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['python3', aruco_detector],
            output='screen',
        ),
    ])