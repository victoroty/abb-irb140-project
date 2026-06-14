#include <memory>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("move_home");

    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(node);

    std::thread spinner(
        [&executor]()
        {
            executor.spin();
        }
    );

    moveit::planning_interface::MoveGroupInterface move_group(
        node,
        "manipulator"
    );

    rclcpp::sleep_for(
        std::chrono::seconds(2)
    );

    RCLCPP_INFO(
        node->get_logger(),
        "Planning frame: %s",
        move_group.getPlanningFrame().c_str()
    );


    RCLCPP_INFO(
        node->get_logger(),
        "End effector link: %s",
        move_group.getEndEffectorLink().c_str()
    );

    auto current_pose =
        move_group.getCurrentPose();

    RCLCPP_INFO(
        node->get_logger(),
        "Current pose:"
    );

    RCLCPP_INFO(
        node->get_logger(),
        "x = %.3f, y = %.3f, z = %.3f",
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z
    );

    RCLCPP_INFO(
        node->get_logger(),
        "qx = %.3f, qy = %.3f, qz = %.3f, qw = %.3f",
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w
    );

    geometry_msgs::msg::Pose target_pose;

    target_pose.orientation.w = 1.0;

    target_pose.position.x = 0.30;
    target_pose.position.y = 0.20;
    target_pose.position.z = 0.80;

    RCLCPP_INFO(
        node->get_logger(),
        "Setting Cartesian target"
    );

    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success =
        static_cast<bool>(move_group.plan(plan));

    if (success)
    {
        RCLCPP_INFO(
            node->get_logger(),
            "Planning succeeded"
        );

        RCLCPP_INFO(
            node->get_logger(),
            "Executing trajectory..."
        );

        move_group.execute(plan);

        RCLCPP_INFO(
            node->get_logger(),
            "Execution finished"
        );
    }
    else
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Planning failed"
        );
    }

    executor.cancel();

    spinner.join();

    rclcpp::shutdown();

    return 0;
}
