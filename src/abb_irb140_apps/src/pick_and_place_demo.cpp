#include "abb_irb140_apps/robot_controller.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    RobotController robot;

    auto fail =
        [](const char * step)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("demo"),
                "%s failed",
                step
            );

            rclcpp::shutdown();

            return 1;
        };

    RCLCPP_INFO(
        rclcpp::get_logger("demo"),
        "Starting side-grasp pick and place demo"
    );

    const double box_x = 0.465;
    const double pick_y = 0.20;
    const double place_y = -0.20;
    const double box_z = 0.120;

    if (!robot.goHome())
    {
        return fail("goHome");
    }

    if (!robot.addTable())
    {
        return fail("addTable");
    }

    if (!robot.addBox(
        box_x,
        pick_y,
        box_z
    ))
    {
        return fail("addBox");
    }

    if (!robot.openGripper())
    {
        return fail("openGripper");
    }

    if (!robot.pick(
        box_x,
        pick_y,
        box_z
    ))
    {
        return fail("pick");
    }

    if (!robot.place(
        box_x,
        place_y,
        box_z
    ))
    {
        return fail("place");
    }

    if (!robot.goHome())
    {
        return fail("goHome final");
    }

    rclcpp::shutdown();

    return 0;
}
