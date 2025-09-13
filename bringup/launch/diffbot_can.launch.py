# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_device",
            default_value="can0",
            description="CAN device name (e.g., can0, vcan0).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_bitrate",
            default_value="500000",
            description="CAN bitrate in bps.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_wheel_can_id",
            default_value="0x101",
            description="CAN ID for left wheel velocity commands.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_wheel_can_id",
            default_value="0x102",
            description="CAN ID for right wheel velocity commands.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "broadcast_interval_ms",
            default_value="10",
            description="CAN message broadcast interval in milliseconds.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_can_config_file",
            default_value="false",
            description="Use external CAN configuration file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logging level (debug, info, warn, error, fatal).",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    can_device = LaunchConfiguration("can_device")
    can_bitrate = LaunchConfiguration("can_bitrate")
    left_wheel_can_id = LaunchConfiguration("left_wheel_can_id")
    right_wheel_can_id = LaunchConfiguration("right_wheel_can_id")
    broadcast_interval_ms = LaunchConfiguration("broadcast_interval_ms")
    use_can_config_file = LaunchConfiguration("use_can_config_file")
    log_level = LaunchConfiguration("log_level")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ros_diff_bot"), "urdf", "diffbot.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "can_device:=",
            can_device,
            " ",
            "can_bitrate:=",
            can_bitrate,
            " ",
            "left_wheel_can_id:=",
            left_wheel_can_id,
            " ",
            "right_wheel_can_id:=",
            right_wheel_can_id,
            " ",
            "broadcast_interval_ms:=",
            broadcast_interval_ms,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros_diff_bot"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )
    
    can_config = PathJoinSubstitution(
        [
            FindPackageShare("ros_diff_bot"),
            "config",
            "can_config.yaml",
        ]
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros_diff_bot"), "rviz", "diffbot.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers, can_config],
    output="both",
    arguments=["--ros-args", "--log-level", log_level],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
    parameters=[robot_description],
    arguments=["--ros-args", "--log-level", log_level],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
    arguments=["joint_state_broadcaster", "--ros-args", "--log-level", log_level],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diffbot_base_controller",
            "--param-file",
            robot_controllers,
            "--controller-ros-args",
            "-r /diffbot_base_controller/cmd_vel:=/cmd_vel",
            "--ros-args", "--log-level", log_level,
        ],
    )

    # CAN diagnostics node (optional)
    can_diagnostics_node = Node(
        package="ros_diff_bot",
        executable="can_diagnostics_node",
        name="can_diagnostics",
        parameters=[can_config],
        output="log",
        condition=IfCondition(LaunchConfiguration("enable_can_diagnostics", default="false")),
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
