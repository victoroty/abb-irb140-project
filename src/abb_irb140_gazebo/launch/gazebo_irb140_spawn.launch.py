from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("abb_irb140_support"),
                            "urdf",
                            "irb140.xacro"
                        ]
                    ),
                    " use_gazebo:=true"
                ]
            ),
            value_type=str
        )
    }
    gazebo_config_path = SetEnvironmentVariable(
        name="GZ_CONFIG_PATH",
        value=(
            "/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:"
            "/opt/ros/jazzy/opt/sdformat_vendor/share/gz"
        )
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=(
            "/root/ros2_projects/abb_project/install:"
            "/root/ros2_projects/abb_project/install/abb_irb140_support/share:"
            "/root/ros2_projects/abb_project/src"
        )
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py"
                ]
            )
        ),
        launch_arguments={
            "gz_args": (
                "/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/"
                "gz-sim8/worlds/empty.sdf"
            )
        }.items()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {
                "use_sim_time": True
            }
        ]
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            "empty",
            "-topic",
            "/robot_description",
            "-name",
            "abb_irb140",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0"
        ]
    )

    delayed_spawn_robot = TimerAction(
        period=3.0,
        actions=[
            spawn_robot
        ]
    )

    return LaunchDescription(
        [
            gazebo_config_path,
            gazebo_resource_path,
            gazebo,
            robot_state_publisher,
            delayed_spawn_robot
        ]
    )
