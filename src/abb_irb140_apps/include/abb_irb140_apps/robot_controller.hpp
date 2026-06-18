#ifndef ABB_IRB140_APPS_ROBOT_CONTROLLER_HPP
#define ABB_IRB140_APPS_ROBOT_CONTROLLER_HPP

#include <atomic>
#include <memory>
#include <thread>
#include <string>

#include <vector>
#include <geometry_msgs/msg/pose.hpp>

#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class RobotController
{

public:

    RobotController();

    ~RobotController();

    bool goHome();

    bool goReady();

    bool openGripper();

    bool closeGripper();

    bool clearWorkcellObjects();

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

    void gazeboBoxSyncLoop();

    bool setGazeboBoxPose(
        const geometry_msgs::msg::Pose& pose
    );

    bool waitForCollisionObject(
        const std::string& object_id,
        double timeout_seconds
    );

    bool waitUntilCollisionObjectRemoved(
        const std::string& object_id,
        double timeout_seconds
    );

    bool waitForAttachedObject(
        const std::string& object_id,
        double timeout_seconds
    );

    bool waitUntilAttachedObjectRemoved(
        const std::string& object_id,
        double timeout_seconds
    );

    rclcpp::Node::SharedPtr node_;

    rclcpp::executors::SingleThreadedExecutor executor_;

    std::thread executor_thread_;

    std::shared_ptr<
        moveit::planning_interface::MoveGroupInterface
    > move_group_;

    std::shared_ptr<
        moveit::planning_interface::MoveGroupInterface
    > gripper_group_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    gz::transport::Node gazebo_transport_node_;

    std::atomic_bool gazebo_sync_running_ {false};

    std::atomic_bool gazebo_box_attached_ {false};

    std::thread gazebo_sync_thread_;
};

#endif
