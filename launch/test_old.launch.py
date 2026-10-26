import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('Sensors-and-Control-Integrated-Project')
    world = os.path.join(pkg_share, 'worlds', 'snc_test_new_env.sdf')

    # 1. URDF for Robot State Publisher (TF transforms)
    urdf_file = os.path.join(pkg_share, 'models', 'turtlebot3_burger.urdf')
    with open(urdf_file, 'r') as robot_file:
        robot_description_config = robot_file.read()

    # 2. SDF model for Gazebo Classic (includes ROS sensor plugins)
    sdf_file = os.path.join(
        pkg_share, 'models', 'turtlebot3_burger_camera', 'model.sdf'
    )
    aruco_script_path = os.path.join(pkg_share, 'scripts', 'aruco_detector.py')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    params_dir = os.path.join(pkg_share, 'parameters', 'params.yaml')

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            # 'params_file': params_dir,
        }.items()
    )

    return LaunchDescription([

        # Set GAZEBO_MODEL_PATH so Gazebo Classic resolves models.
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[
                os.environ.get('GAZEBO_MODEL_PATH', ''),
                ':', os.path.join(pkg_share, 'models')
            ]
        ),

        SetEnvironmentVariable(
            name='RMW_FASTRTPS_USE_SHM',
            value='0'
        ),

        ExecuteProcess(
            cmd=[
                'gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so', world
            ],
            output='screen'
        ),

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

        # 4. Spawn the project-owned robot model in Gazebo Classic.
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'turtlebot3', '-file', sdf_file,
                       '-x', '-2.1', '-y', '0.0', '-z', '0.01'],
            output='screen',
        ),

        # Run a Python script directly
        ExecuteProcess(
            cmd=['python3', aruco_script_path],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}]
        ),

        nav2_launch,

        slam_toolbox_launch,
    ])
