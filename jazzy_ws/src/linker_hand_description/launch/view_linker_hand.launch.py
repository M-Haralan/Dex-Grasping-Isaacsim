from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

def launch_setup(context, *args, **kwargs):
    # General arguments
    control_package = "linker_hand_control"
    description_package = "linker_hand_description"
    description_file = "linker_hand_l20_left.ros2_control.xacro"
    
    # Get launch configurations
    use_gui = LaunchConfiguration("use_gui")
    can_interface = LaunchConfiguration("can_interface")
    can_bitrate = LaunchConfiguration("can_bitrate")

    # CAN interface setup commands
    can_down_cmd = ExecuteProcess(
        cmd=["sudo", "/usr/sbin/ip", "link", "set", can_interface, "down"],
        name="can_down",
        output="screen",
    )

    can_setup_cmd = ExecuteProcess(
        cmd=["sudo", "/usr/sbin/ip", "link", "set", can_interface, "type", "can", "bitrate", can_bitrate],
        name="can_setup",
        output="screen",
    )

    can_up_cmd = ExecuteProcess(
        cmd=["sudo", "/usr/sbin/ip", "link", "set", can_interface, "up"],
        name="can_up",
        output="screen",
    )

    # CAN interface cleanup command (for shutdown)
    can_cleanup_cmd = ExecuteProcess(
        cmd=["sudo", "/usr/sbin/ip", "link", "set", can_interface, "down"],
        name="can_cleanup",
        output="screen",
    )

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

    # Controller manager configuration
    controller_config = PathJoinSubstitution(
        [FindPackageShare(control_package), "config", "controllers.yaml"]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_linker_hand.rviz"]
    )

    # Controller manager node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ParameterFile(controller_config, allow_substs=True)],
        output="both",
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Joint trajectory controller spawner
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    # Joint state publisher GUI (optional - for manual control)
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(use_gui),
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Event handlers for CAN setup sequence 
    can_setup_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=can_down_cmd,
            on_exit=can_setup_cmd,
        )
    )

    can_up_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=can_setup_cmd,
            on_exit=can_up_cmd,
        )
    )

    # Start nodes after CAN is ready (we use timer delays instead of process exit)
    controller_manager_timer = RegisterEventHandler(
        OnProcessExit(
            target_action=can_up_cmd,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[controller_manager_node, robot_state_publisher_node],
                )
            ],
        )
    )

    # Start controllers after controller manager (with delay)
    controllers_timer = TimerAction(
        period=8.0,  # Wait 8 seconds for everything to be ready
        actions=[
            joint_state_broadcaster_spawner,
            TimerAction(
                period=3.0,  # Wait 3 more seconds for joint state broadcaster
                actions=[joint_trajectory_controller_spawner],
            )
        ],
    )

    # Start GUI and RViz after controllers (with delay)
    gui_and_rviz_timer = TimerAction(
        period=12.0,  # Wait 12 seconds total
        actions=[joint_state_publisher_gui_node, rviz_node],
    )

    # RViz exit handler - shutdown everything when RViz closes
    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
        ),
    )

    # Cleanup CAN interface on shutdown
    can_cleanup_handler = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=can_cleanup_cmd,
        ),
    )

    nodes_to_start = [
        # CAN setup sequence
        can_down_cmd,
        can_setup_handler,
        can_up_handler,
        
        # Main nodes (started after CAN is ready)
        controller_manager_timer,
        
        # Controllers and GUI (started with timers)
        controllers_timer,
        gui_and_rviz_timer,
        
        # Event handlers
        exit_event_handler,
        can_cleanup_handler,
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    # Add argument for enabling/disabling joint state publisher GUI
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gui",
            default_value="false",
            description="Start joint_state_publisher_gui",
        )
    )

    # Add CAN interface arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="CAN interface to use (can0 for the L20 hand)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "can_bitrate",
            default_value="1000000",
            description="CAN bitrate in bps (default: 1Mbps)",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])