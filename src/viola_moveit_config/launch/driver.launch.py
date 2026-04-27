from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    servo_port_name = LaunchConfiguration("servo_port_name")

    # Create and return the launch description
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "servo_port_name",
                default_value="/dev/star_arm_viola",
                description="Serial device path for the robot servos",
            ),
            Node(
                package="robo_driver",
                executable="driver",
                name="viola_driver",
                output="screen",
                parameters=[{"servo_port_name": servo_port_name}],
            ),
            Node(
                package="viola_controller",
                executable="controller",
                name="viola_controller",
                output="screen",
            )        
        ]
    )
