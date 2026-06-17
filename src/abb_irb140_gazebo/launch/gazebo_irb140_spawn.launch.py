from launch import LaunchDescription
from launch.actions import ExecuteProcess
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
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_args": PathJoinSubstitution(
                [
                    FindPackageShare("abb_irb140_gazebo"),
                    "worlds",
                    "irb140_workcell.sdf",
                ]
            )
        }.items(),
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

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )


    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            "irb140_workcell",
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

    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "20",
            "--inactive",
        ],
    )

    spawn_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "20",
            "--inactive",
        ],
    )

    spawn_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "20",
            "--inactive",
        ],
    )

    unpause_gazebo = ExecuteProcess(
        output="screen",
        cmd=[
            "bash",
            "-lc",
            (
                "export GZ_CONFIG_PATH="
                "/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:"
                "/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:"
                "/opt/ros/jazzy/opt/sdformat_vendor/share/gz:"
                "${GZ_CONFIG_PATH}; "
                "gz service -s /world/irb140_workcell/control "
                "--reqtype gz.msgs.WorldControl "
                "--reptype gz.msgs.Boolean "
                "--timeout 2000 "
                "--req 'pause: false'"
            ),
        ],
    )


    activate_controllers = ExecuteProcess(
        output="screen",
        cmd=[
            "ros2",
            "control",
            "switch_controllers",
            "--activate",
            "joint_state_broadcaster",
            "joint_trajectory_controller",
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
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
            clock_bridge,
            robot_state_publisher,

            TimerAction(period=3.0, actions=[spawn_robot]),

            # Temporary startup sequencing:
            # load controllers while Gazebo is paused,
            # then unpause and activate.
            # Later, replace fixed timers with readiness checks.
            TimerAction(period=7.0, actions=[spawn_joint_state_broadcaster]),

            TimerAction(period=9.0, actions=[spawn_joint_trajectory_controller]),
            TimerAction(period=11.0, actions=[spawn_gripper_controller]),

            TimerAction(period=14.0, actions=[unpause_gazebo]),
            TimerAction(period=16.0, actions=[activate_controllers]),
        ]
    )
