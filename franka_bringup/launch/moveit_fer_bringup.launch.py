from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)


def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration("robot_ip")

    franka_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("franka_bringup"), "launch", "franka.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "arm_id": "fer",
            "load_gripper": "false",
            "use_rviz": "false",
            "controller_file_package": "acg_resources_fer_moveit_config",
            "controller_file_path": "config/ros2_controllers_real.yaml",
            "initial_joint_controller": "fer_arm_trajectory_controller",
        }.items(),
    )

    # Start RViz without loading plugins associated to planning
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("acg_resources_fer_moveit_config"),
                    "config",
                    "moveit.rviz",
                ]
            ),
        ],
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("acg_resources_fer_moveit_config"),
                        "launch",
                        "move_group.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "hand": "false",
        }.items(),
    )

    return [franka_bringup_launch, rviz_node, move_group_launch]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", description="IP address by which the robot can be reached."
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )