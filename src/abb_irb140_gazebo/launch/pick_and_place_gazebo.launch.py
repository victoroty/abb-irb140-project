from launch import LaunchDescription

from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "abb_irb140",
            package_name="abb_irb140_moveit_config",
        )
        .robot_description(
            mappings={
                "use_gazebo": "true",
            }
        )
        .to_moveit_configs()
    )

    pick_and_place_app = Node(
        package="abb_irb140_apps",
        executable="pick_and_place_demo",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
            },
        ],
    )

    return LaunchDescription(
        [
            pick_and_place_app,
        ]
    )
