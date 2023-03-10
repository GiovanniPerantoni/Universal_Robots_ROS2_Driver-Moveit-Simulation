#include <rclcpp/rclcpp.hpp>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include<iostream>
using namespace std;
using namespace std::chrono_literals;
#define Z_BASE_LINK 1.79
#define Z_DESK 0.867
static const rclcpp::Logger LOGGER = rclcpp::get_logger("keyboard_control.interface.cpp");

// BEGIN_TUTORIAL

// Setup
// ^^^^^
// First we declare pointers to the node and publisher that will publish commands to Servo
rclcpp::Node::SharedPtr node_;
float val = 0;

void move_collisions(rclcpp::Node::SharedPtr node_);

// Next we will set up the node, planning_scene_monitor, and collision object
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  // This is false for now until we fix the QoS settings in moveit to enable intra process comms
  node_options.use_intra_process_comms(false);
  node_ = std::make_shared<rclcpp::Node>("servo_node", node_options);

  // Pause for RViz to come up. This is necessary in an integrated demo with a single launch file
  rclcpp::sleep_for(std::chrono::seconds(5));

  // Create the planning_scene_monitor. We need to pass this to Servo's constructor, and we should set it up first
  // before initializing any collision objects
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, "robot_description", tf_buffer, "planning_scene_monitor");

  // Here we make sure the planning_scene_monitor is updating in real time from the joint states topic
  if (planning_scene_monitor->getPlanningScene())
  {
    planning_scene_monitor->startStateMonitor("/joint_states");
    planning_scene_monitor->setPlanningScenePublishingFrequency(25);
    planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                         "/moveit_servo/publish_planning_scene");
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->providePlanningSceneService();
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning scene not configured");
    return EXIT_FAILURE;
  }

  auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();

  while (true) {
    move_collisions(node_);
    // executor->add_node(node_);
    // executor->spin();
    executor->spin_node_once(node_);
    cout << "\n\n\nMove\n\n" << endl;
    rclcpp::sleep_for(std::chrono::seconds(2));
  }

  executor->spin();
  
  rclcpp::shutdown();
  return 0;
}

void move_collisions(rclcpp::Node::SharedPtr node_) {
  val += 0.05;
  cout << val << endl;

  // Next we will create a collision object in the way of the arm. As the arm is servoed towards it, it will slow down
  // and stop before colliding
  moveit_msgs::msg::CollisionObject l_arm;
  moveit_msgs::msg::CollisionObject r_arm;
  moveit_msgs::msg::CollisionObject h_body;
  moveit_msgs::msg::CollisionObject l_fore;
  moveit_msgs::msg::CollisionObject r_fore;

  // define reference frame
  l_arm.header.frame_id = "base_link";
  r_arm.header.frame_id = "base_link";
  h_body.header.frame_id = "base_link";
  l_fore.header.frame_id = "base_link";
  r_fore.header.frame_id = "base_link";

  l_arm.id = "l_arm";
  r_arm.id = "r_arm";
  h_body.id = "h_body";
  l_fore.id = "l_fore";
  r_fore.id = "r_fore";

  shape_msgs::msg::SolidPrimitive primitive1, primitive2, primitive3, primitive4, primitive5;
  // defining left arm's collision
  primitive1.type = primitive1.CYLINDER;
  primitive1.dimensions.resize(2);
  primitive1.dimensions[0] = 1;
  primitive1.dimensions[1] = 0.075;
  // defining right arm's collision
  primitive2.type = primitive2.CYLINDER;
  primitive2.dimensions.resize(2);
  primitive2.dimensions[0] = 1;
  primitive2.dimensions[1] = 0.075;
  // defining body's collision
  primitive3.type = primitive2.CYLINDER;
  primitive3.dimensions.resize(2);
  primitive3.dimensions[0] = 2;
  primitive3.dimensions[1] = 0.25;
  // defining left forearm's collision
  primitive4.type = primitive2.CYLINDER;
  primitive4.dimensions.resize(2);
  primitive4.dimensions[0] = 1;
  primitive4.dimensions[1] = 0.075;
  // defining right forearm's collision
  primitive5.type = primitive2.CYLINDER;
  primitive5.dimensions.resize(2);
  primitive5.dimensions[0] = 1;
  primitive5.dimensions[1] = 0.075;

  geometry_msgs::msg::Pose l_arm_pose, r_arm_pose, h_body_pose, l_fore_pose, r_fore_pose;
  // defining left arm's pose
  l_arm_pose.orientation.w = -0.707;
  l_arm_pose.orientation.y = 0.707; //val; //
  // l_arm_pose.orientation.x = val;
  l_arm_pose.position.x = 0-0.4;
  l_arm_pose.position.y = 0.75;
  l_arm_pose.position.z = Z_DESK - 0.5;
  // defining right arm's pose
  r_arm_pose.orientation.w = -0.707;
  r_arm_pose.orientation.y = 0.707;
  r_arm_pose.position.x = 1-0.4;
  r_arm_pose.position.y = 0.75;
  r_arm_pose.position.z = Z_DESK - 0.5;
  // defining body pose
  h_body_pose.orientation.w = 1;
  h_body_pose.position.x = 0.5-0.4;
  h_body_pose.position.y = 0.75;
  h_body_pose.position.z = Z_DESK - 0.25;
  // defining left forearm's pose
  l_fore_pose.orientation.w = -0.707;
  l_fore_pose.orientation.x = 0.707;
  l_fore_pose.position.x = 0-0.5;
  l_fore_pose.position.y = 0.75;
  l_fore_pose.position.z = Z_DESK - 0.5;
  // defining right forearm's pose
  r_fore_pose.orientation.w = -0.707;
  r_fore_pose.orientation.x = 0.707;
  r_fore_pose.position.x = 1-0.3;
  r_fore_pose.position.y = 0.75;
  r_fore_pose.position.z = Z_DESK - 0.5;

  l_arm.primitives.push_back(primitive1);
  l_arm.primitive_poses.push_back(l_arm_pose);
  l_arm.operation = l_arm.ADD;
  
  r_arm.primitives.push_back(primitive2);
  r_arm.primitive_poses.push_back(r_arm_pose);
  r_arm.operation = r_arm.ADD;

  h_body.primitives.push_back(primitive3);
  h_body.primitive_poses.push_back(h_body_pose);
  h_body.operation = h_body.ADD;

  l_fore.primitives.push_back(primitive4);
  l_fore.primitive_poses.push_back(l_fore_pose);
  l_fore.operation = l_fore.ADD;

  r_fore.primitives.push_back(primitive5);
  r_fore.primitive_poses.push_back(r_fore_pose);
  r_fore.operation = r_fore.ADD;

  // Create the message to publish the collision object
  moveit_msgs::msg::PlanningSceneWorld psw;
  psw.collision_objects.push_back(l_arm);
  psw.collision_objects.push_back(r_arm);
  psw.collision_objects.push_back(h_body);
  psw.collision_objects.push_back(l_fore);
  psw.collision_objects.push_back(r_fore);
  moveit_msgs::msg::PlanningScene ps;
  ps.is_diff = true;
  ps.world = psw; 
  // Publish the collision object to the planning scene
  auto scene_pub = node_->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
  scene_pub->publish(ps);

  // We use a multithreaded executor here because Servo has concurrent processes for moving the robot and avoiding collisions
  // executor->add_node(node_);
  // executor->spin();
}