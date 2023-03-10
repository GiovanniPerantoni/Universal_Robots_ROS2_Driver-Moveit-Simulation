#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // // Specify the planning group name
    // const std::string PLANNING_GROUP = "manipulator";

    // // Create an Options object with the planning group name
    // moveit::planning_interface::MoveGroupInterface::Options options(PLANNING_GROUP);

    // // Create a MoveGroupInterface instance with the Options object
    // moveit::planning_interface::MoveGroupInterface move_group(options);

    // // Set the planner to use Hybrid Planning
    // move_group.setPlannerId("HybridPlanner");

    // // Set the target pose for the end-effector
    // geometry_msgs::msg::PoseStamped target_pose;
    // // Set the target pose here...

    // // Set the target pose as the goal
    // move_group.setPoseTarget(target_pose);

    // // Plan the motion
    // moveit::planning_interface::MoveGroupInterface::Plan plan;
    // move_group.plan(plan);

    // // Execute the motion
    // move_group.execute(plan);

    rclcpp::shutdown();
    return 0;
}