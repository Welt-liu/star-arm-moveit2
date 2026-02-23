import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "cello_description", package_name="cello_moveit_config"
    ).to_moveit_configs()

    pkg_share = get_package_share_directory("cello_moveit_config")
    ros2_controllers_path = os.path.join(pkg_share, "config", "ros2_controllers.yaml")
    servo_params_path = os.path.join(pkg_share, "config", "moveit_servo.yaml")
    with open(servo_params_path, "r", encoding="utf-8") as f:
        servo_yaml = yaml.safe_load(f) or {}
    servo_params = {"moveit_servo": servo_yaml}

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "arm_controller", "-c", "/controller_manager"],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            servo_params,
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            ros2_control_node,
            controllers_spawner,
            servo_node,
        ]
    )
