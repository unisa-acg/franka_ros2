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


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def robot_description_dependent_nodes_spawner(
        context: LaunchContext,
        robot_ip,
        arm_id,
        use_fake_hardware,
        fake_sensor_commands,
        load_gripper,
        async_interface,
        filter_commands,
        joint_position_rate_limit,
        joint_position_low_pass_filter,
        position_control_gain):

    robot_ip_str = context.perform_substitution(robot_ip)
    arm_id_str = context.perform_substitution(arm_id)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    fake_sensor_commands_str = context.perform_substitution(
        fake_sensor_commands)
    load_gripper_str = context.perform_substitution(load_gripper)
    async_interface_str = context.perform_substitution(async_interface)
    filter_commands_str = context.perform_substitution(filter_commands)
    joint_position_rate_limit_str = context.perform_substitution(joint_position_rate_limit)
    joint_position_low_pass_filter_str = context.perform_substitution(joint_position_low_pass_filter) 
    position_control_gain_str = context.perform_substitution(position_control_gain) 

    franka_xacro_filepath = os.path.join(get_package_share_directory(
        'franka_description'), 'robots', arm_id_str, arm_id_str+'.urdf.xacro')
    robot_description = xacro.process_file(franka_xacro_filepath,
                                           mappings={
                                               'ros2_control': 'true',
                                               'arm_id': arm_id_str,
                                               'robot_ip': robot_ip_str,
                                               'hand': load_gripper_str,
                                               'use_fake_hardware': use_fake_hardware_str,
                                               'fake_sensor_commands': fake_sensor_commands_str,
                                               'async_interface': async_interface_str,
                                               'filter_commands': filter_commands_str,
                                               'joint_position_rate_limit': joint_position_rate_limit_str,
                                               'joint_position_low_pass_filter': joint_position_low_pass_filter_str,
                                               'position_control_gain': position_control_gain_str,
                                           }).toprettyxml(indent='  ')

    controller_file_package = LaunchConfiguration('controller_file_package')
    controller_file_path = LaunchConfiguration('controller_file_path')

    # Default to original file if controller_file_path is empty
    controller_file_path_str = context.perform_substitution(controller_file_path)
    if controller_file_path_str:
        franka_controllers = PathJoinSubstitution(
            [FindPackageShare(controller_file_package), controller_file_path])
    else:
        franka_controllers = PathJoinSubstitution(
            [FindPackageShare('franka_bringup'), 'config', 'controllers.yaml'])

    nodes_to_spawn = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[franka_controllers,
                        {'robot_description': robot_description},
                        {'arm_id': arm_id},
                        ],
            remappings=[('joint_states', 'franka/joint_states')],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            on_exit=Shutdown(),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'source_list': ['franka/joint_states', f'{arm_id_str}_gripper/joint_states'],
                'rate': 30,
            }],
        ),
    ]
    
    if LaunchConfiguration('initial_joint_controller').perform(context):
        nodes_to_spawn.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[LaunchConfiguration('initial_joint_controller')],
                output='screen',
            )
        )
    
    return nodes_to_spawn


def generate_launch_description():
    arm_id_parameter_name = 'arm_id'
    robot_ip_parameter_name = 'robot_ip'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'
    async_interface_parameter_name = 'async_interface'
    filter_commands_parameter_name = 'filter_commands'
    joint_position_rate_limit_parameter_name = 'joint_position_rate_limit'
    joint_position_low_pass_filter_parameter_name = 'joint_position_low_pass_filter'
    position_control_gain_parameter_name = 'position_control_gain'

    arm_id = LaunchConfiguration(arm_id_parameter_name)
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    async_interface = LaunchConfiguration(async_interface_parameter_name)
    filter_commands = LaunchConfiguration(filter_commands_parameter_name)
    joint_position_rate_limit = LaunchConfiguration(joint_position_rate_limit_parameter_name)
    joint_position_low_pass_filter = LaunchConfiguration(joint_position_low_pass_filter_parameter_name)
    position_control_gain = LaunchConfiguration(position_control_gain_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(
        fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')

    robot_description_dependent_nodes_spawner_opaque_function = OpaqueFunction(
        function=robot_description_dependent_nodes_spawner,
        args=[
            robot_ip,
            arm_id,
            use_fake_hardware,
            fake_sensor_commands,
            load_gripper,
            async_interface,
            filter_commands,
            joint_position_rate_limit,
            joint_position_low_pass_filter,
            position_control_gain])

    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            description='Hostname or IP address of the robot.'),
        DeclareLaunchArgument(
            arm_id_parameter_name,
            description='ID of the type of arm used. Supported values: fer, fr3, fp3'),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='false',
            description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false',
            description='Use fake hardware'),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description='Fake sensor commands. Only valid when "{}" is true'.format(
                use_fake_hardware_parameter_name)),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'),
        DeclareLaunchArgument(
            async_interface_parameter_name,
            default_value='true',
            description='Use an async hardware interface?'),
        DeclareLaunchArgument(
            filter_commands_parameter_name,
            default_value='false',
            description='Filter position commands?'),
        DeclareLaunchArgument(
            joint_position_rate_limit_parameter_name,
            default_value='true',
            description='Use rate limiter on position commands?'),
        DeclareLaunchArgument(
            joint_position_low_pass_filter_parameter_name,
            default_value='true',
            description='Use a low pass filter on position commands?'),
        DeclareLaunchArgument(
            position_control_gain_parameter_name,
            default_value='0.001',
            description='Position control gain'),            
        DeclareLaunchArgument(
            'controller_file_package',
            default_value='franka_bringup',
            description='Package name where the controller config file is located.'
        ),
        DeclareLaunchArgument(
            'controller_file_path',
            default_value='config/controllers.yaml',
            description='Relative path to the controller config file inside the package.'
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="",
            description="Initially loaded robot controller. The controller has to be defined in the "
            "controllers file.",
        ),
        robot_description_dependent_nodes_spawner_opaque_function,
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['franka_robot_state_broadcaster'],
            parameters=[{'arm_id': arm_id}],
            output='screen',
            condition=UnlessCondition(use_fake_hardware),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
            launch_arguments={robot_ip_parameter_name: robot_ip,
                              use_fake_hardware_parameter_name: use_fake_hardware}.items(),
            condition=IfCondition(load_gripper)
        ),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file],
             condition=IfCondition(use_rviz)
             )

    ])

    return launch_description
