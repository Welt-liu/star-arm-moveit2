import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    moveit_config = MoveItConfigsBuilder(
        "cello_description", package_name="cello_moveit_config"
    ).to_moveit_configs()

    pkg_share = get_package_share_directory("cello_moveit_config")
    ros2_controllers_path = os.path.join(pkg_share, "config", "ros2_controllers.yaml")
    servo_params_path = os.path.join(pkg_share, "config", "moveit_servo.yaml")
    pose_tracking_params_path = os.path.join(
        pkg_share, "config", "pose_tracking_settings.yaml"
    )
    rviz_config_path = os.path.join(pkg_share, "config", "moveit.rviz")

    with open(servo_params_path, "r", encoding="utf-8") as f:
        servo_yaml = yaml.safe_load(f) or {}
    with open(pose_tracking_params_path, "r", encoding="utf-8") as f:
        pose_tracking_yaml = yaml.safe_load(f) or {}

    # PoseTracking publishes metric twists (m/s, rad/s), so Servo input must be speed_units.
    servo_yaml["command_in_type"] = "speed_units"

    # PoseTracking PID params must be in the same moveit_servo namespace.
    merged_servo_params = dict(servo_yaml)
    merged_servo_params.update(pose_tracking_yaml)
    servo_params = {"moveit_servo": merged_servo_params}

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

    pose_tracking_node = Node(
        package="cello_servo_pose_tracking",
        executable="xr_pose_tracking_node",
        name="xr_pose_tracking",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            servo_params,
            {
                "use_sim_time": use_sim_time,
                "position_tolerance": 0.005,
                "orientation_tolerance": 0.01,
                "target_pose_timeout": 0.75,
                "enable_startup_home": True,
                "control_x_translation": merged_servo_params.get("control_x_translation", True),
                "control_y_translation": merged_servo_params.get("control_y_translation", True),
                "control_z_translation": merged_servo_params.get("control_z_translation", True),
                "control_x_rotation": merged_servo_params.get("control_x_rotation", True),
                "control_y_rotation": merged_servo_params.get("control_y_rotation", True),
                "control_z_rotation": merged_servo_params.get("control_z_rotation", True),
                "home_joint_names": [
                    "joint1",
                    "joint2",
                    "joint3",
                    "joint4",
                    "joint5",
                    "joint6",
                ],
                "home_joint_positions": [0.1, -0.8, 0.8, -0.7, 0.5, 0.2],
                "home_trajectory_time_sec": 2.,
            },
        ],
    )

    wrist_to_target_node = Node(
        package="cello_servo_pose_tracking",
        executable="wrist_tf_to_target_pose_node",
        name="wrist_tf_to_target_pose",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "world_frame": "world",
                "wrist_frame": "right_wrist",
                "ee_frame": merged_servo_params.get("ee_frame_name", "link6"),
                "output_topic": "/xr_target_pose",
                "publish_rate_hz": 50.0,
                "reset_on_time_jump": True,
                "reset_on_forward_time_jump": False,
                "time_jump_threshold_sec": 1.0,
                # "orientation_mode": "world_translation_only",
                "translation_scale_x": -1.0,
                "translation_scale_y": -1.0,
                "translation_scale_z": 1.0,
                "orientation_mode": "full_offset",
                # "orientation_mode": "source_rp_yaw_offset",
                "publish_markers": True,
                "marker_topic": "/xr_target_markers",
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use ROS /clock time instead of system time",
            ),
            robot_state_publisher,
            ros2_control_node,
            controllers_spawner,
            pose_tracking_node,
            wrist_to_target_node,
            rviz_node,
        ]
    )
