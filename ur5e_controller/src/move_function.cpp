#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include "ur5e_interfaces/srv/ur5e_service.hpp"

#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning_api_tutorial");


void move(const std::shared_ptr<ur5e_interfaces::srv::Ur5eService::Request> request,
         std::shared_ptr<ur5e_interfaces::srv::Ur5eService::Response>      response)
{
   // moveit code with response->complete (bool) and request->x ,y ,z how to access and write data

      double x =request->x;
      double y =request->y;
      double z =request->z;

    rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();



  static const std::string PLANNING_GROUP = "ur_manipulator";

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  move_group.allowReplanning(true);
  

  // Raw pointers are frequently used to refer to the planning group for improved performance.
 const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


move_group.setNamedTarget("start");
move_group.move();
RCLCPP_INFO(LOGGER, "moved to start");



         using namespace Eigen;
      //Roll pitch and yaw in Radians
      double roll = 1.571, pitch = 1.571, yaw = 	1.571;    
        
      Quaterniond q;
      q = AngleAxisd(roll, Vector3d::UnitX())* AngleAxisd(pitch, Vector3d::UnitY())* AngleAxisd(yaw, Vector3d::UnitZ());
      
      
    





std::vector<geometry_msgs::msg::Pose> waypoints;

geometry_msgs::msg::Pose target_pose1;

  //target_pose1.orientation.w = q.coeffs()[3];
  //target_pose1.orientation.x = q.coeffs()[0];
  //target_pose1.orientation.y = q.coeffs()[1];
  //target_pose1.orientation.z = q.coeffs()[1];

  target_pose1.position.x = x;
  waypoints.push_back(target_pose1);
  target_pose1.position.y = y;
  waypoints.push_back(target_pose1);
  target_pose1.position.z = z;
  waypoints.push_back(target_pose1);

moveit_msgs::msg::RobotTrajectory trajectory;
const double jump_threshold = 0.2;
const double eef_step = 0.1;
double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  //readd later //////bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group.execute(trajectory); 


executor.cancel();



}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("move_function_server");

  rclcpp::Service<ur5e_interfaces::srv::Ur5eService>::SharedPtr service =
    node->create_service<ur5e_interfaces::srv::Ur5eService>("move_function", &move);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service started");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
