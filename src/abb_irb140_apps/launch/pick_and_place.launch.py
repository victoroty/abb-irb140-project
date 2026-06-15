from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "abb_irb140",
            package_name="abb_irb140_moveit_config"
        ).to_moveit_configs()
    )

    demo = generate_demo_launch(moveit_config)

    app = Node(
        package="abb_irb140_apps",
        executable="pick_and_place_demo",
        output="screen",
        parameters=[
            moveit_config.to_dict()
        ]
    )

    ld = LaunchDescription()
    for entity in demo.entities:
        ld.add_action(entity)

    ld.add_action(app)

    return ld
