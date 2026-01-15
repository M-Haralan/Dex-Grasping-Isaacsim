from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)

def launch_setup(context, *args, **kwargs):
    # General arguments
    description_package = "linker_hand_description"
    description_file = "linker_hand_l20_left.urdf.xacro"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_linker_hand.rviz"]
    )

    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    # )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    
    # RViz exit handler
    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
        ),
    )

    nodes_to_start = [
        # joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        exit_event_handler
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])