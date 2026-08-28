import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="true"
    )

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mecanum_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    laser_driver = Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            parameters=[os.path.join(
                get_package_share_directory("rescuebot_bringup"),
                "config",
                "rplidar_a1.yaml"
            )],
            output="screen"
    )
    
    controller = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("rescuebot_controller"),
                "launch",
                "controller.launch.py"
            ),
            launch_arguments={
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
            "use_sim_time": "False"
        }.items()
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rescuebot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False",
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
    camera_pub = Node(
            package="rescuebot_perception",
            executable="camera_publisher",
            name="camera_publisher",
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
        hardware_interface,
        laser_driver,
        controller,
        joystick,
        slam,
        navigation,
        camera_pub,
        exploration,
        april_tag_detection,
    ])
