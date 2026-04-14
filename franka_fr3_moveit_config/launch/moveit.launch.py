#  Copyright (c) 2024 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

# This file is an adapted version of
# https://github.com/ros-planning/moveit_resources/blob/ca3f7930c630581b5504f3b22c40b4f82ee6369d/panda_moveit_config/launch/demo.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    Shutdown,
    OpaqueFunction,
    LogInfo
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

import yaml


robot_ip_parameter_name = 'robot_ip'
use_fake_hardware_parameter_name = 'use_fake_hardware'
fake_sensor_commands_parameter_name = 'fake_sensor_commands'
isaac_parameter_name = 'isaac'
namespace_parameter_name = 'namespace'


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def _apply_prefixes_in_list(data: list, prefix, stem, keys=True, only_start=True, exceptions=[]):
    for i in range(len(data)):
        if type(data[i]) is str and data[i] not in exceptions: # TODO: only_start handling
            data[i] = data[i].replace(stem, prefix + stem, 1)
        elif type(data[i]) is dict:
            data[i] = add_prefixes(data[i], prefix, stem, keys, only_start, exceptions)
        elif type(data[i]) is list:
            data[i] = _apply_prefixes_in_list(data[i], prefix, stem, keys, only_start, exceptions)
    return data

def add_prefixes(data: dict, prefix, stem, keys=True, only_start=True, exceptions=[]):
    """
    Add a prefix to keys and values of a yaml file. This can be used to apply prefix data
    to MoveIt configurations.

    Example:
    fr3_joint --> franko_fr3_joint

    Arguments:
    - data: YAML object where changes should be made
    - prefix: Prefix to add
    - stem: Stem where the prefix should be put before (Example: 'fr3_' --> 'PREFIXfr3_')
    - keys: If False, only apply prefixes to values and not keys.
    - only_start: Only add the prefix if the stem is at the beginning of the value/key (startswith)
    - exceptions: List of keys/values that should remain unchanged.
    """

    # loop through all keys
    keys_to_change = []
    for key in data:

        # check key
        if keys and (((only_start and key.startswith(stem)) or (not only_start and (stem in key)))
                     and key not in exceptions):
            keys_to_change.append(key)
        
        # then check value
        if type(data[key]) is dict:
            data[key] = add_prefixes(data[key], prefix, stem, keys, only_start, exceptions)
            continue

        elif type(data[key]) is list:
            data[key] = _apply_prefixes_in_list(data[key], prefix, stem, keys, only_start, exceptions)

        elif type(data[key]) is str and ((only_start and data[key].startswith(stem)) or (not only_start and (stem in data[key]))
                                         and data[key] not in exceptions):
            data[key] = data[key].replace(stem, prefix + stem, 1)
    
    # apply key changes
    # do this in a new loop because we can't change the dict keys as we're iterating through it
    for key in keys_to_change:
        new_key = key.replace(stem, prefix + stem, 1)
        data[new_key] = data[key]
        data.pop(key, None)

    return data


def launch_setup(context, *args, **kwargs):

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(
        fake_sensor_commands_parameter_name)
    namespace = LaunchConfiguration(namespace_parameter_name)
    isaac = LaunchConfiguration(isaac_parameter_name)

    # define modified parameters
    namespace_modified = str(namespace.perform(context)) + '_' if namespace.perform(context) else ''
    namespace_slash = str(namespace.perform(context)) + '/' if namespace.perform(context) else ''

    if isaac.perform(context).lower() == 'true':
        use_fake_hardware = 'true'

    # Command-line arguments

    db_arg = DeclareLaunchArgument(
        'db', default_value='False', description='Database flag'
    )

    # planning_context
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.urdf.xacro'
    )

    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=true',
         ' robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands, ' ros2_control:=true',
         ' isaac:=', isaac, ' namespace:=', namespace, ' arm_prefix:=', namespace])

    robot_description = {'robot_description': ParameterValue(
        robot_description_config, value_type=str)}

    # if namespace is franko, try to find franko's srdf first
    srdf_info = None
    if namespace.perform(context) == 'franko':
        try:
            franko_package = get_package_share_directory('franko_fr3_hand_moveit_config')
            franka_semantic_xacro_file = os.path.join(
                franko_package, 'config', 'fr3.srdf'
            )
            srdf_info = LogInfo(msg="Successfully found Franko SRDF!")
        except:
            # franko package not found, just use default franka file
            srdf_info = LogInfo(msg="Warning: Franko SRDF was not found!! This impacts the behavior of certain humation components and will lead to errors!")
            franka_semantic_xacro_file = os.path.join(
                get_package_share_directory('franka_description'),
                'robots', 'fr3', 'fr3.srdf.xacro'
            )
    
    else:
        franka_semantic_xacro_file = os.path.join(
            get_package_share_directory('franka_description'),
            'robots', 'fr3', 'fr3.srdf.xacro'
        )
        srdf_info = LogInfo(msg="Successfully found Franka SRDF.")

    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ',
         franka_semantic_xacro_file, ' hand:=true arm_prefix:=', namespace_modified]
    )

    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        robot_description_semantic_config, value_type=str)}

    kinematics_yaml = add_prefixes(load_yaml(
        'franka_fr3_moveit_config', 'config/kinematics.yaml'
    ), namespace_modified, 'fr3_')

    kinematics_config = {
        'robot_description_kinematics': kinematics_yaml
    }

    joint_limits_yaml = add_prefixes(load_yaml(
        'franka_fr3_moveit_config', 'config/fr3_joint_limits.yaml'
    ), namespace_modified, 'fr3_')

    joint_limits_config = {
        'robot_description_planning': joint_limits_yaml
    }

    cartesian_limits_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/cartesian_limits.yaml'
    )
    cartesian_limits_config = cartesian_limits_yaml if cartesian_limits_yaml else {}

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
                                ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath'
                                  ],
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = add_prefixes(load_yaml(
        'franka_fr3_moveit_config', 'config/fr3_controllers.yaml'
    ), namespace_modified, 'fr3_', exceptions=['fr3_arm_controller', 'fr3_gripper_controller'])
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    move_group_capabilities = {
        'capabilities': 'move_group/ExecuteTaskSolutionCapability'
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace_slash,
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            cartesian_limits_config,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            {'use_sim_time': isaac}
        ],
    )


    # RViz
    rviz_base = os.path.join(get_package_share_directory(
        'franka_fr3_moveit_config'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, namespace_modified + 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=namespace_slash,
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_config,
            joint_limits_config,
            cartesian_limits_config,
            {'use_sim_time': isaac}
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace_slash,
        output='both',
        parameters=[robot_description, {'use_sim_time': isaac}],
    )

    # Publish Isaac Sim compatibility transform
    isaac_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='isaac_transform_publisher',
        namespace=namespace,
        output='screen',
        arguments=['0', '0', '0', 
                   '0', '0', '0', '1',
                   namespace_modified + 'fr3_link0', 
                   'base']
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'config',
        namespace_modified + 'fr3_ros_controllers.yaml',
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace_slash,
        parameters=[robot_description, ros2_controllers_path, {'use_sim_time': isaac}],
        # remappings=[('joint_states', 'franka/joint_states')],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        on_exit=Shutdown(),
    )

    # Load controllers
    load_controllers = []
    for controller in ['fr3_arm_controller', 'fr3_gripper_controller', 'joint_state_broadcaster']:
        load_controllers.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'controller_manager', 'spawner', controller,
                    '--controller-manager-timeout', '60',
                    '--controller-manager',
                    PathJoinSubstitution([namespace, 'controller_manager'])
                ],
                output='screen'
            )
        )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[
            {'source_list': [namespace_slash + 'joint_states', 'fr3_gripper_controller/joint_states']},
            {'use_sim_time': isaac}],
    )

    franka_robot_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['franka_robot_state_broadcaster'],
        output='screen',
        condition=UnlessCondition(use_fake_hardware),
        parameters=[{'use_sim_time': isaac}],
    )

    only_real_robot_nodes = [joint_state_publisher, franka_robot_state_broadcaster]

    gripper_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
        launch_arguments={'robot_ip': robot_ip,
                          use_fake_hardware_parameter_name: use_fake_hardware,
                          'namespace': namespace}.items(),
    )

    # append nodes if needed
    if (isaac.perform(context).lower() == 'false'):
        print("Launching for real robot or Gazebo simulation")
        load_controllers += only_real_robot_nodes

    return [
         db_arg,
         rviz_node,
         robot_state_publisher,
         run_move_group_node,
         ros2_control_node,
         gripper_launch_file,
         isaac_transform_publisher,
         srdf_info
         ] + load_controllers


def generate_launch_description():
    robot_arg = DeclareLaunchArgument(
        robot_ip_parameter_name,
        default_value='none',  # added a default value here since using the robot with Isaac Sim does not need an IP.
        description='Hostname or IP address of the robot.')

    namespace_arg = DeclareLaunchArgument(
        namespace_parameter_name,
        default_value='',
        description='Namespace for the robot.'
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        use_fake_hardware_parameter_name,
        default_value='false',
        description='Use fake hardware'
    )
    isaac_arg = DeclareLaunchArgument(
        isaac_parameter_name,
        default_value='false',
        description='Use topic based ROS2 control for integration with Isaac Sim.'
    )
    fake_sensor_commands_arg = DeclareLaunchArgument(
        fake_sensor_commands_parameter_name,
        default_value='false',
        description="Fake sensor commands. Only valid when '{}' is true".format(
            use_fake_hardware_parameter_name))

    return LaunchDescription(
        [robot_arg,
         namespace_arg,
         use_fake_hardware_arg,
         isaac_arg,
         fake_sensor_commands_arg,
         OpaqueFunction(function=launch_setup)]
    )
