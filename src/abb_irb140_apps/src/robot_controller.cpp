#include "abb_irb140_apps/robot_controller.hpp"
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

RobotController::RobotController()
{
    node_ =
        rclcpp::Node::make_shared(
            "robot_controller"
        );

    executor_.add_node(node_);

    executor_thread_ =
        std::thread(
            [this]()
            {
                executor_.spin();
            }
        );

    move_group_ =
        std::make_shared<
            moveit::planning_interface::MoveGroupInterface
        >(
            node_,
            "manipulator"
        );
}

RobotController::~RobotController()
{
    executor_.cancel();

    if (executor_thread_.joinable())
    {
        executor_thread_.join();
    }
}

bool RobotController::executeCurrentTarget()
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success =
        static_cast<bool>(
            move_group_->plan(plan)
        );

    if (!success)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Planning failed"
        );

        return false;
    }

    auto result =
        move_group_->execute(plan);

    return result ==
        moveit::core::MoveItErrorCode::SUCCESS;
}

bool RobotController::goHome()
{
    move_group_->setNamedTarget(
        "home"
    );

    return executeCurrentTarget();
}

bool RobotController::goReady()
{
    move_group_->setNamedTarget(
        "ready"
    );

    return executeCurrentTarget();
}

bool RobotController::pick(
    double x,
    double y,
    double z
)
{
    const double approach_height = 0.10;

    if (!moveToPose(
        x,
        y,
        z + approach_height
    ))
    {
        return false;
    }

    if (!moveLinear(
        x,
        y,
        z
    ))
    {
        return false;
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "Closing gripper (simulated)"
    );

    if (!moveLinear(
        x,
        y,
        z + approach_height
    ))
    {
        return false;
    }

    return true;
}

bool RobotController::place(
    double x,
    double y,
    double z
)
{
    const double approach_height = 0.10;

    if (!moveToPose(
        x,
        y,
        z + approach_height
    ))
    {
        return false;
    }

    if (!moveLinear(
        x,
        y,
        z
    ))
    {
        return false;
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "Opening gripper (simulated)"
    );

    if (!moveLinear(
        x,
        y,
        z + approach_height
    ))
    {
        return false;
    }

    return true;
}

bool RobotController::moveToPose(
    double x,
    double y,
    double z
)
{
    geometry_msgs::msg::Pose target_pose;

    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;

    move_group_->setPoseTarget(
        target_pose
    );

    return executeCurrentTarget();
}

bool RobotController::moveToJointTarget(
    const std::vector<double>& joints
)
{
    move_group_->setJointValueTarget(
        joints
    );

    return executeCurrentTarget();
}

geometry_msgs::msg::Pose
RobotController::getCurrentPose()
{
    return move_group_->
        getCurrentPose().pose;
}
bool RobotController::moveLinear(
    double x,
    double y,
    double z
)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;

    auto target =
        getCurrentPose();

    target.position.x = x;
    target.position.y = y;
    target.position.z = z;

    waypoints.push_back(target);

    moveit_msgs::msg::RobotTrajectory trajectory;

    double fraction =
        move_group_->computeCartesianPath(
            waypoints,
            0.01,
            trajectory
        );

    if (fraction < 0.95)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Cartesian path failed"
        );

        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    plan.trajectory = trajectory;

    auto result =
        move_group_->execute(plan);

    return result ==
        moveit::core::MoveItErrorCode::SUCCESS;
}
bool RobotController::addTable()
{
    moveit::planning_interface::PlanningSceneInterface
        planning_scene_interface;

    moveit_msgs::msg::CollisionObject table;

    table.header.frame_id = "base_link";

    table.id = "table";

    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type =
        primitive.BOX;

    primitive.dimensions.resize(3);

    primitive.dimensions[0] = 1.0;
    primitive.dimensions[1] = 1.0;
    primitive.dimensions[2] = 0.10;

    geometry_msgs::msg::Pose table_pose;

    table_pose.orientation.w = 1.0;

    table_pose.position.x = 0.60;
    table_pose.position.y = 0.00;
    table_pose.position.z = 0.05;

    table.primitives.push_back(
        primitive
    );

    table.primitive_poses.push_back(
        table_pose
    );

    table.operation =
        table.ADD;

    planning_scene_interface
        .applyCollisionObject(table);

    RCLCPP_INFO(
        node_->get_logger(),
        "Table added to planning scene"
    );

    return true;
}
