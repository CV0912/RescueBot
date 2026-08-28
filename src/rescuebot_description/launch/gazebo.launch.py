import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rescuebot_description = get_package_share_directory("rescuebot_description")

    model_arg = DeclareLaunchArgument(
        name="model", default_value=os.path.join(
                rescuebot_description, "urdf", "rescuebot.xacro"
            ),
        description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="test_world1.sdf")

    world_path = PathJoinSubstitution([
            rescuebot_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + ' -v 4 -r'"])
        ]
    )

    model_path = str(Path(rescuebot_description).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("rescuebot_description"), 'models')

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
        )

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
                }.items()
             )

    # gz_spawn_entity = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     output="screen",
    #     arguments=["-topic", "robot_description",
    #                "-name", "rescuebot",
    #                "-x", "-0.2758",
    #                "-y", "-5.6774",
    #                "-z", "0.5006",
    #                 ],
    # ) 2026 theme
    gz_spawn_entity = Node(
    package="ros_gz_sim",
    executable="create",
    output="screen",
    arguments=[
        "-topic", "robot_description",
        "-name", "rescuebot",
        # "-x", "-3.3029",
        # "-y", "-4.7616",
        # "-z", "0.698",
        # "-R", "0.0",
        # "-P", "0.0",
        # "-Y", "0.0",
    ],
)
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        # remappings=[
        #     ('/imu', '/imu/out'),
        # ]
    )
    return LaunchDescription([
        model_arg,
        world_name_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])