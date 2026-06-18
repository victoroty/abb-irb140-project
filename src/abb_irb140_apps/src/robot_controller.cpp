#include "abb_irb140_apps/robot_controller.hpp"

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>

#include <stdexcept>

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

    if (!move_group_->setEndEffectorLink(
            "gripper_grasp_frame"
        ))
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Failed to set end effector link to gripper_grasp_frame"
        );

        throw std::runtime_error(
            "Failed to set end effector link to gripper_grasp_frame"
        );
    }

    gripper_group_ =
        std::make_shared<
            moveit::planning_interface::MoveGroupInterface
        >(
            node_,
            "gripper"
        );

    RCLCPP_INFO(
        node_->get_logger(),
        "Planning frame: %s",
        move_group_->getPlanningFrame().c_str()
    );

    RCLCPP_INFO(
        node_->get_logger(),
        "End effector link: %s",
        move_group_->getEndEffectorLink().c_str()
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

bool RobotController::openGripper()
{
    gripper_group_->setJointValueTarget(
        "gripper_finger_joint1",
        0.04
    );

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success =
        static_cast<bool>(
            gripper_group_->plan(plan)
        );

    if (!success)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Failed to plan gripper opening"
        );

        return false;
    }

    auto result =
        gripper_group_->execute(plan);

    return result ==
        moveit::core::MoveItErrorCode::SUCCESS;
}

bool RobotController::closeGripper()
{
/*
  The box is 0.03 m wide. A full close target of 0.0 can collide
  through a physical/simulated box and make the gripper controller abort.
  0.017 keeps a small clearance around a 0.03 m object.
*/
    gripper_group_->setJointValueTarget(
        "gripper_finger_joint1",
        0.017
    );

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success =
        static_cast<bool>(
            gripper_group_->plan(plan)
        );

    if (!success)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Failed to plan gripper closing"
        );

        return false;
    }

    auto result =
        gripper_group_->execute(plan);

    return result ==
        moveit::core::MoveItErrorCode::SUCCESS;
}

bool RobotController::pick(
    double x,
    double y,
    double z
)
{
    const double approach_distance = 0.12;
    const double lift_height = 0.10;

    const double pre_grasp_x =
        x - approach_distance;

    if (!moveToPose(
        pre_grasp_x,
        y,
        z
    ))
    {
        return false;
    }

    auto p = getCurrentPose();

    RCLCPP_INFO(
        node_->get_logger(),
        "Current grasp frame pose: x=%.3f y=%.3f z=%.3f",
        p.position.x,
        p.position.y,
        p.position.z
    );

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
    "Before attachBox"
);

if (!attachBox())
{
    RCLCPP_ERROR(
        node_->get_logger(),
        "attachBox failed"
    );

    return false;
}

RCLCPP_INFO(
    node_->get_logger(),
    "After attachBox"
);

RCLCPP_INFO(
    node_->get_logger(),
    "Before closeGripper"
);

if (!closeGripper())
{
    RCLCPP_ERROR(
        node_->get_logger(),
        "closeGripper failed"
    );

    return false;
}

RCLCPP_INFO(
    node_->get_logger(),
    "After closeGripper"
);

    if (!moveLinear(
        pre_grasp_x,
        y,
        z
    ))
    {
        return false;
    }

    if (!moveToPose(
        pre_grasp_x,
        y,
        z + lift_height
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
    const double approach_distance = 0.12;
    const double lift_height = 0.10;

    const double pre_place_x =
        x - approach_distance;

    if (!moveToPose(
        pre_place_x,
        y,
        z + lift_height
    ))
    {
        return false;
    }

    if (!moveToPose(
        pre_place_x,
        y,
        z
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

    if (!openGripper())
    {
        return false;
    }

    if (!detachBox(
        x,
        y,
        z
    ))
    {
        return false;
    }

    if (!moveLinear(
        pre_place_x,
        y,
        z
    ))
    {
        return false;
    }

    if (!moveToPose(
        pre_place_x,
        y,
        z + lift_height
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

    // Side-grasp orientation.
    // This is the first test orientation for a horizontal side grasp.
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.7071068;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.7071068;

    RCLCPP_INFO(
        node_->get_logger(),
        "moveToPose target x=%.3f y=%.3f z=%.3f",
        x,
        y,
        z
    );

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

    auto current = getCurrentPose();

    RCLCPP_INFO(
        node_->get_logger(),
        "moveLinear from x=%.3f y=%.3f z=%.3f to x=%.3f y=%.3f z=%.3f",
        current.position.x,
        current.position.y,
        current.position.z,
        x, y, z
    );

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
            trajectory,
            true
        );

    RCLCPP_INFO(
        node_->get_logger(),
        "Cartesian fraction = %.3f",
        fraction
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

    primitive.dimensions[0] = 0.40;
    primitive.dimensions[1] = 0.70;
    primitive.dimensions[2] = 0.020;

    geometry_msgs::msg::Pose table_pose;

    table_pose.orientation.w = 1.0;

    table_pose.position.x = 0.65;
    table_pose.position.y = 0.00;
    table_pose.position.z = 0.290;

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

bool RobotController::addBox(
    double x,
    double y,
    double z
)
{
    moveit::planning_interface::PlanningSceneInterface
        planning_scene_interface;

    moveit_msgs::msg::CollisionObject box;

    box.header.frame_id = "base_link";

    box.id = "box";

    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type =
        primitive.BOX;

    primitive.dimensions.resize(3);

    primitive.dimensions[0] = 0.03;
    primitive.dimensions[1] = 0.03;
    primitive.dimensions[2] = 0.08;

    geometry_msgs::msg::Pose pose;

    pose.orientation.w = 1.0;

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    box.primitives.push_back(
        primitive
    );

    box.primitive_poses.push_back(
        pose
    );

    box.operation =
        box.ADD;

    planning_scene_interface
        .applyCollisionObject(box);

    rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(
        node_->get_logger(),
        "Box added"
    );

    return true;
}

bool RobotController::attachBox()
{
    moveit::planning_interface::PlanningSceneInterface
        planning_scene_interface;

    moveit_msgs::msg::AttachedCollisionObject attached;

    attached.link_name =
        "gripper_grasp_frame";

    attached.object.header.frame_id =
        "gripper_grasp_frame";

    attached.object.id =
        "box";

    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type =
        primitive.BOX;

    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.03;
    primitive.dimensions[1] = 0.03;
    primitive.dimensions[2] = 0.08;

    geometry_msgs::msg::Pose pose;

    /*
      The free workpiece is defined in base_link with identity orientation.
      The attached workpiece is defined relative to gripper_grasp_frame.

      Because the gripper uses a side-grasp orientation, an identity
      orientation here would align the workpiece with the gripper frame and
      make the taller billet appear rotated when attached.

      This inverse side-grasp rotation keeps the attached workpiece visually
      aligned with the upright world workpiece during the current demo.
    */
    pose.orientation.x = 0.0;
    pose.orientation.y = -0.7071068;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.7071068;

    // Workpiece center relative to gripper_grasp_frame.
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;

    attached.object.primitives.push_back(
        primitive
    );

    attached.object.primitive_poses.push_back(
        pose
    );

    attached.object.operation =
        attached.object.ADD;

    attached.touch_links = {
        "gripper_grasp_frame",
        "gripper_panda_hand",
        "gripper_leftfinger",
        "gripper_rightfinger"
    };

    planning_scene_interface
        .applyAttachedCollisionObject(
            attached
        );

    rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(
        node_->get_logger(),
        "Box attached"
    );

    return true;
}

bool RobotController::detachBox(
    double x,
    double y,
    double z
)
{
    moveit::planning_interface::PlanningSceneInterface
        planning_scene_interface;

    moveit_msgs::msg::AttachedCollisionObject attached;

    attached.link_name =
        "gripper_grasp_frame";

    attached.object.id =
        "box";

    attached.object.operation =
        attached.object.REMOVE;

    planning_scene_interface
        .applyAttachedCollisionObject(
            attached
        );

    rclcpp::sleep_for(
        std::chrono::milliseconds(500)
    );

    moveit_msgs::msg::CollisionObject box;

    box.header.frame_id =
        "base_link";

    box.id =
        "box";

    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type =
        primitive.BOX;

    primitive.dimensions.resize(3);

    primitive.dimensions[0] = 0.03;
    primitive.dimensions[1] = 0.03;
    primitive.dimensions[2] = 0.08;

    geometry_msgs::msg::Pose pose;

    pose.orientation.w =
        1.0;

    pose.position.x =
        x;

    pose.position.y =
        y;

    pose.position.z =
        z;

    box.primitives.push_back(
        primitive
    );

    box.primitive_poses.push_back(
        pose
    );

    box.operation =
        box.ADD;

    planning_scene_interface
        .applyCollisionObject(
            box
        );

    rclcpp::sleep_for(
        std::chrono::milliseconds(500)
    );

    RCLCPP_INFO(
        node_->get_logger(),
        "Box detached and placed in world"
    );

    return true;
}
