import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    pkg_name = 'Sensors-and-Control-Integrated-Project'
    pkg_share = get_package_share_directory(pkg_name)
    world = os.path.join(pkg_share, 'worlds', 'final_env.sdf')
    aruco_script_path = os.path.join(pkg_share, 'scripts', 'aruco_detector.py')
    ekf_path = os.path.join(pkg_share, 'scripts', 'ekf_node.py')
    lqr_path = os.path.join(pkg_share, 'scripts', 'lqr_controller.py')

    # 1. URDF for Robot State Publisher (TF transforms) 
    tb3_desc_dir = get_package_share_directory('turtlebot3_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'turtlebot3_burger.urdf') # switch xacro to switch between default turtlebot and custom one with camera
    doc = xacro.process_file(xacro_file, mappings={'namespace': ''})
    robot_description_config = doc.toxml()

    # 2. SDF model for Gazebo (includes libgazebo_ros_diff_drive plugin)
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    sdf_file = os.path.join(pkg_share, 'sdf', 'turtlebot3_model.sdf') # switch sdf for testing

    tb3_parent_dir = os.path.dirname(tb3_desc_dir)


    return LaunchDescription([

        # Set GAZEBO_MODEL_PATH so Gazebo resolves meshes and SDF models
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[
                os.environ.get('GAZEBO_MODEL_PATH', ''), 
                ':', tb3_parent_dir,
                ':', os.path.join(pkg_share, 'models'),
                ':', os.path.join(pkg_share, 'models', 'CAD'),
                ':', os.path.join(tb3_gazebo_dir, 'models')
            ]
        ),
        SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI', value=''),

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
            parameters=[{
                'robot_description': robot_description_config,
                'use_sim_time': True
                }]
            
        ),

        # 4. Spawn TurtleBot3 using model.sdf with diff drive plugin
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'turtlebot3', '-file', sdf_file, '-x', '-2', '-y', '3.75', '-z', '0.01'],
            output='screen',
            parameters=[{
                'robot_description': robot_description_config,
                'use_sim_time': True
            }]
        ),

        ExecuteProcess(
            cmd=['python3', aruco_script_path],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['python3', ekf_path],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['python3', lqr_path],
            output='screen'
        ),
    ])