import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rescuebot_controller_pkg = get_package_share_directory('rescuebot_controller')
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",
                                      description="Use simulated time"
    )
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("rescuebot_controller"), "config", "joy_config.yaml")]
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(get_package_share_directory("rescuebot_controller"), "config", "joy_teleop.yaml")]
    )
    twist_mux_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("twist_mux"),
            "launch",
            "twist_mux_launch.py"
        ),
        launch_arguments={
            "cmd_vel_out": "/cmd_vel_unstamped",
            "config_topics": os.path.join(rescuebot_controller_pkg, "config", "twist_mux_topics.yaml"),
            "config_joy": os.path.join(rescuebot_controller_pkg, "config", "twist_mux_joy.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )
    pneumatic_teleop = Node(
        package="rescuebot_controller",
        executable="pnuematic_teleop.py",
        name="pneumatic_teleop",
    )

    drive_teleop = Node(
        package="rescuebot_controller",
        executable="drive_teleop.py",
        name="drive_teleop",
    )

    return LaunchDescription([
        joy_node,
        use_sim_time_arg,
        joy_teleop,
        # drive_teleop,
        twist_mux_launch,
    ])