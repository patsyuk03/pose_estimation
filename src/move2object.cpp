#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


std::vector<double> initial_pose = {-0.097065, -0.842345, -0.005829, 0.710569, -0.014803, 1.588374, 0.000941};
geometry_msgs::msg::PoseStamped object_pose;

void callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	object_pose = *msg;
}


int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>("pose_estimation_node");

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("pose_estimation_node");

    // We spin up a SingleThreadedExecutor so we can get current joint values later
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Subscribe to tf_pose
    auto subscription = node->create_subscription<geometry_msgs::msg::PoseStamped>("tf_pose", 100, callback);

    // Create the MoveIt Move Group Interface for xarm and gripper
    moveit::planning_interface::MoveGroupInterface move_group_xarm(node, "xarm7");
    moveit::planning_interface::MoveGroupInterface move_group_gripper(node, "xarm_gripper");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction;

    geometry_msgs::msg::PoseStamped xarm_pose;

    // Move to initial position
    move_group_xarm.setJointValueTarget(initial_pose);
    bool success = (move_group_xarm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success) {
        move_group_xarm.execute(my_plan);
    } else {
        RCLCPP_ERROR(logger, "Planing failed!");
    }

    // Gripper open
    move_group_gripper.setJointValueTarget("drive_joint", 0.0);
    success = (move_group_gripper.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success) {
        move_group_gripper.execute(my_plan);
    } else {
        RCLCPP_ERROR(logger, "Planing failed!");
    }

    // Move to object
    xarm_pose = move_group_xarm.getCurrentPose();
    xarm_pose.pose=object_pose.pose;

	tf2::Quaternion q_orig, q_rot, q_new;
    tf2::convert(xarm_pose.pose.orientation , q_orig);
    double r=3.14159, p=0, y=0;  // Rotate the previous pose by 180* about X
    q_rot.setRPY(r, p, y);
    q_new = q_rot*q_orig;  // Calculate the new orientation
    q_new.normalize();
    tf2::convert(q_new, xarm_pose.pose.orientation);

	xarm_pose.pose.position.x = object_pose.pose.position.x;
	xarm_pose.pose.position.y = object_pose.pose.position.y;
    xarm_pose.pose.position.z = object_pose.pose.position.z+0.2;
	

    waypoints = {};
    waypoints.push_back(xarm_pose.pose);
    fraction = move_group_xarm.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
    RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);
    if(fraction == 1){
        move_group_xarm.execute(trajectory);
    }

    // Move to initial position
    move_group_xarm.setJointValueTarget(initial_pose);
    success = (move_group_xarm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success) {
        move_group_xarm.execute(my_plan);
    } else {
        RCLCPP_ERROR(logger, "Planing failed!");
    }

    // Shutdown
    rclcpp::shutdown();
    spinner.join();
    return 0;
}