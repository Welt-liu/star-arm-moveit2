from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    rviz_config = LaunchConfiguration('rviz_config')
    launch_calibration = LaunchConfiguration('launch_calibration')

    robo_state_publisher_launch = os.path.join(get_package_share_directory('viola_description'),'launch')
    robo_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robo_state_publisher_launch,'/robo_state_publisher.launch.py']),
        launch_arguments={'launch_calibration': launch_calibration}.items(),
    )


    robo_moveit_rviz_launch = os.path.join(get_package_share_directory('viola_moveit_config'),'launch')
    robo_moveit_rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robo_moveit_rviz_launch,'/move_group_rviz.launch.py']),
        launch_arguments={
            'rviz_config': rviz_config,
            'launch_calibration': launch_calibration,
        }.items(),
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                get_package_share_directory('viola_moveit_config'),
                'config',
                'moveit.rviz',
            ),
        ),
        DeclareLaunchArgument('launch_calibration', default_value='false'),
        robo_state_publisher_node,
        robo_moveit_rviz_node
    ])
