#!/usr/bin/env python3

import os
from os import pathsep
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # PACKAGE DIRECTORIES
    panda_description = get_package_share_directory("panda_description")
    panda_moveit = get_package_share_directory("panda_moveit")
    pymoveit2 = get_package_share_directory("pymoveit2")

    # LAUNCH ARGUMENTS
    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="myworld.sdf",
        description="Name of the Gazebo world file",
    )

    use_rviz_arg = DeclareLaunchArgument(
        name="use_rviz",
        default_value="true",
        description="Launch RViz if true",
    )

    # CONFIGURATIONS
    world_name = LaunchConfiguration("world_name")
    use_rviz = LaunchConfiguration("use_rviz")

    world_path = PathJoinSubstitution([panda_description, "worlds", world_name])


    model_path = str(Path(panda_description).parent.resolve())
    model_path += pathsep + os.path.join(panda_description, "models")

    gazebo_resource_path = SetEnvironmentVariable(
        "IGN_GAZEBO_RESOURCE_PATH", model_path
    )

    # Determine is_ignition based on ROS distro
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    is_ignition = "True" if ros_distro == "humble" else "False"

    # Robot description (URDF via xacro)
    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(panda_description, "urdf", "panda.urdf.xacro"),
            " is_ignition:=", is_ignition,
        ]),
        value_type=str,
    )

    # GAZEBO
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_ign_gazebo"),
                "launch",
                "ign_gazebo.launch.py",
            )
        ]),
        launch_arguments={
            "ign_args": PythonExpression(["'", world_path, " -v 4 -r'"])
        }.items(),
    )

    # ROBOT STATE PUBLISHER
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True}
        ],
    )

    # SPAWN ROBOT IN GAZEBO
    ign_spawn_entity = Node(
        package="ros_ign_gazebo",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "panda",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
    )

    # GAZEBO-ROS BRIDGE (CLOCK)
    ign_ros2_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ign.msgs.Clock"],
    )

    # CONTROLLERS
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_action_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_action_controller", "--controller-manager", "/controller_manager"],
    )

    # MOVEIT 2
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="panda_moveit")
        .robot_description(
            file_path=os.path.join(panda_description, "urdf", "panda.urdf.xacro"),
            mappings={"is_ignition": is_ignition},
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"publish_robot_description_semantic": True},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RVIZ 2
    rviz_config = os.path.join(panda_moveit, "rviz", "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        condition=IfCondition(use_rviz),
    )

    # PICK AND PLACE NODE
    pick_and_place_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="panda_bringup",
                executable="pick_and_place.py",
                name="pick_and_place",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    # LAUNCH DESCRIPTION
    return LaunchDescription([
        # Arguments
        world_name_arg,
        use_rviz_arg,
        gazebo_resource_path,
        # Gazebo
        gazebo,
        robot_state_publisher_node,
        ign_spawn_entity,
        ign_ros2_bridge,
        # Controllers
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        gripper_action_controller_spawner,
        # MoveIt
        move_group_node,
        # RViz
        rviz_node,
        # Application
        pick_and_place_node,
    ])
