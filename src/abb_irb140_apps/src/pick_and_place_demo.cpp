#include "abb_irb140_apps/robot_controller.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    RobotController robot;

    RCLCPP_INFO(
        rclcpp::get_logger("demo"),
        "Starting pick and place demo"
    );

    robot.goHome();

    robot.moveToPose(
        0.45,
        0.20,
        0.50
    );

    robot.moveLinear(
        0.45,
        0.20,
        0.20
    );

    robot.moveLinear(
        0.45,
        0.20,
        0.50
    );

    robot.moveToPose(
        0.45,
        -0.20,
        0.50
    );

    robot.moveLinear(
        0.45,
        -0.20,
        0.20
    );

    robot.moveLinear(
        0.45,
        -0.20,
        0.50
    );

    robot.goHome();

    rclcpp::shutdown();

    return 0;
}
