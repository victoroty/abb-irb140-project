#include "abb_irb140_apps/robot_controller.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    RobotController robot;

    RCLCPP_INFO(
        rclcpp::get_logger("test"),
        "RobotController created successfully"
    );

    robot.addTable();

    robot.goHome();

    robot.goReady();

    auto pose =
        robot.getCurrentPose();

    RCLCPP_INFO(
        rclcpp::get_logger("test"),
        "Current pose: x=%f y=%f z=%f",
        pose.position.x,
        pose.position.y,
        pose.position.z
    );

    robot.moveToPose(
        0.45,
        0.15,
        0.50
    );

    robot.moveLinear(
        0.45,
        0.15,
        0.30
    );

    robot.moveLinear(
        0.45,
        0.15,
        0.50
    );

    robot.goHome();

    rclcpp::shutdown();

    return 0;
}
