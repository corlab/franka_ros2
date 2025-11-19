from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_arg = DeclareLaunchArgument('robot', default_value='panda', description='choose your robot. Possible values: [panda, fr3]')
    arm_id_arg = DeclareLaunchArgument('arm_id', default_value=LaunchConfiguration('robot'), description='arm id (defaults to robot)')
    run_rqt_arg = DeclareLaunchArgument('run_rqt_reconfigure', default_value='false', description='Run rqt_reconfigure (optional)')

    # Include the bringup launch which starts ros2_control and robot_state_publisher
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py'
        ])),
        launch_arguments={
            'arm_id': LaunchConfiguration('arm_id'),
            # Override the default controllers yaml to use this package's config if present
            'controllers_yaml': PathJoinSubstitution([
                FindPackageShare('franka_example_controllers'), 'config', 'franka_example_controllers.yaml'
            ])
        }.items(),
    )

    # Spawn the controllers stopped (same behavior as ROS1 --stopped)
    spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='controller_spawner',
        output='screen',
        arguments=['--stopped', 'tf_controller', 'position_joint_trajectory_controller'],
    )

    # Optional rqt_reconfigure (ROS2 support may vary)
    rqt = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='rqt_reconfigure',
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_rqt_reconfigure')),
    )

    return LaunchDescription([
        robot_arg,
        arm_id_arg,
        run_rqt_arg,
        bringup,
        spawner,
        rqt,
    ])
