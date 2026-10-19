import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    pkg_share = get_package_share_directory('Sensors-and-Control-Integrated-Project')
    world = os.path.join(pkg_share, 'worlds', 'snc_test.sdf')

    tb3_desc_dir = get_package_share_directory('turtlebot3_description')
    xacro_file = os.path.join(tb3_desc_dir, 'urdf', 'turtlebot3_burger.urdf')
    doc = xacro.process_file(xacro_file, mappings={'namespace': ''})
    robot_description_config = doc.toxml()

    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    sdf_file = os.path.join(tb3_gazebo_dir, 'models', 'turtlebot3_burger', 'model.sdf')

    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    tb3_parent_dir = os.path.dirname(tb3_desc_dir)

    return LaunchDescription([

        # Set resource path for Gazebo Sim / Ignition to locate meshes
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
                ':', tb3_parent_dir,
                ':', os.path.join(tb3_gazebo_dir, 'models')
            ]
        ),

        # 1. Start Gazebo Ignition / Sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r -v 4 {world}'}.items(),
        ),

        # 2. Publish robot state to /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
        ),

        # 3. Spawn TurtleBot3 in Gazebo Ignition
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'turtlebot3',
                '-file', sdf_file,
                '-x', '0.0', '-y', '0.0', '-z', '0.01'
            ],
            output='screen'
        ),

        # 4. Bridge ROS 2 and Gazebo Ignition topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
            ],
            output='screen'
        )
    ])