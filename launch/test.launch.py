import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    pkg_share = get_package_share_directory('Sensors-and-Control-Integrated-Project')
    world = os.path.join(pkg_share, 'worlds', 'snc_test.sdf')
    # world = os.path.join(get_package_share_directory('gazebo_ros'), 'worlds', 'gazebo_underwater.world')

    tb3_desc_dir = get_package_share_directory('turtlebot3_description')
    urdf_file = os.path.join(tb3_desc_dir, 'urdf', 'turtlebot3_burger.urdf')

    xacro_file = os.path.join(tb3_desc_dir, 'urdf', 'turtlebot3_burger.urdf')

    # 1. Process the xacro file to expand ${namespace} and macros into valid XML
    # doc = xacro.process_file(xacro_file, mappings={'namespace': ''})
    # robot_desc_xml = doc.toxml()

    return LaunchDescription([

        # launches gazebo
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        #         launch_arguments={'world': world}.items()
        # ),

        ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),

        # publish robot state (tf tranforms) so ros2 can look at it
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_file]
        ),

        # spawn turtlebot in world
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'turtlebot3', '-file', urdf_file, '-x', '0.0', '-y', '0.0', '-z', '0.01'],
            output='screen'
        )
    ])