#ifndef ABB_IRB140_APPS_ROBOT_CONTROLLER_HPP
#define ABB_IRB140_APPS_ROBOT_CONTROLLER_HPP

#include <memory>
#include <thread>

#include <vector>
#include <geometry_msgs/msg/pose.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

class RobotController
{

public:

    RobotController();

    ~RobotController();

    bool goHome();

    bool goReady();

    bool openGripper();

    bool closeGripper();

    bool addTable();

    bool addBox(
        double x,
        double y,
        double z
    );

    bool attachBox();

    bool detachBox(
        double x,
        double y,
        double z
    );

    bool pick(
        double x,
        double y,
        double z
    );

    bool place(
        double x,
        double y,
        double z
    );

    bool moveToPose(
    double x,
    double y,
    double z
    );

    bool moveLinear(
        double x,
        double y,
        double z
    );

    bool moveToJointTarget(
       const std::vector<double>& joints
    );

    geometry_msgs::msg::Pose
    getCurrentPose();

private:

    bool executeCurrentTarget();

    rclcpp::Node::SharedPtr node_;

    rclcpp::executors::SingleThreadedExecutor executor_;

    std::thread executor_thread_;

    std::shared_ptr<
        moveit::planning_interface::MoveGroupInterface
    > move_group_;

    std::shared_ptr<
        moveit::planning_interface::MoveGroupInterface
    > gripper_group_;
};

#endif
