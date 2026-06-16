from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_config_path = SetEnvironmentVariable(
        name="GZ_CONFIG_PATH",
        value=(
            "/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:"
            "/opt/ros/jazzy/opt/sdformat_vendor/share/gz"
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
                "-r "
                "/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/"
                "gz-sim8/worlds/empty.sdf"
            )
        }.items()
    )

    return LaunchDescription(
        [
            gazebo_config_path,
            gazebo
        ]
    )
