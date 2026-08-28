import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="true"
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rescuebot_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rescuebot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True",
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rescuebot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )
    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rescuebot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True",
        }.items(),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rescuebot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
            get_package_share_directory("rescuebot_description"),
            "rviz",
            "nav2_default_view.rviz"
        )],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    april_tag_detection = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rescuebot_perception"),
            "launch",
            "apriltag.launch.py"
        ),
    )
    exploration = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("frontier_exploration_ros2"),
            "launch",
            "frontier_explorer.launch.py"
        ),
    )
    return LaunchDescription([
        use_slam_arg,
        gazebo,
        controller,
        joystick,
        slam,
        rviz,
        navigation,
        exploration,
        april_tag_detection,
    ])