import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    pkg_share = get_package_share_directory('Sensors-and-Control-Integrated-Project')
    world = os.path.join(pkg_share, 'worlds', 'snc_test.sdf')

    # 1. URDF for Robot State Publisher (TF transforms)
    tb3_desc_dir = get_package_share_directory('turtlebot3_description')
    xacro_file = os.path.join(tb3_desc_dir, 'urdf', 'turtlebot3_burger.urdf')
    doc = xacro.process_file(xacro_file, mappings={'namespace': ''})
    robot_description_config = doc.toxml()

    # 2. SDF model for Gazebo (includes libgazebo_ros_diff_drive plugin)
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    sdf_file = os.path.join(tb3_gazebo_dir, 'models', 'turtlebot3_burger', 'model.sdf')

    py_script_path = os.path.join(pkg_share, 'scripts', 'Hello World.py')

    tb3_parent_dir = os.path.dirname(tb3_desc_dir)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    params_dir = os.path.join(pkg_share, 'parameters', 'params.yaml')

    return LaunchDescription([

        # Set GAZEBO_MODEL_PATH so Gazebo resolves meshes and SDF models
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[
                os.environ.get('GAZEBO_MODEL_PATH', ''), 
                ':', tb3_parent_dir,
                ':', os.path.join(tb3_gazebo_dir, 'models')
            ]
        ),

        # 1. Start Gazebo Server
        ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
            output='screen'
        ),

        # 2. Start Gazebo Client
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),

        # 3. Publish robot state / TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
        ),

        # 4. Spawn TurtleBot3 using model.sdf with diff drive plugin
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'turtlebot3', '-file', sdf_file, '-x', '0.0', '-y', '0.0', '-z', '0.01'],
            output='screen',
            parameters=[{
                'robot_description': robot_description_config,
                'use_sim_time': True
            }]
        ),

        # Run a Python script directly
        ExecuteProcess(
            cmd=['python3', py_script_path],
            output='screen'
        ),
    ])

