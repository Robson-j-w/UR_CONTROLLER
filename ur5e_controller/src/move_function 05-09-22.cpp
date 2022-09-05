#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include "ur5e_interfaces/srv/ur5e_service.hpp"

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
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(move_group_node);
      std::thread([&executor]() { executor.spin(); }).detach();
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////USING MOVE GROUP INTERFACE   ----- COULD TRY MOVEIT CPP API FOR COMPLEX AND BETTER PERFORMANCE//////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



      
      

      static const std::string PLANNING_GROUP = "ur_manipulator";
      moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

      // Getting Basic Information
      // ^^^^^^^^^^^^^^^^^^^^^^^^^
      //
      // We can print the name of the reference frame for this robot.
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning frame: %s", move_group.getPlanningFrame().c_str());

      // We can also print the name of the end-effector link for this group.
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "End effector link: %s", move_group.getEndEffectorLink().c_str());

      // We can get a list of all the groups in the robot:
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Available Planning Groups:");
      std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

      // Planning to a Pose goal
      // ^^^^^^^^^^^^^^^^^^^^^^^
      // We can plan a motion for this group to a desired pose for the
      // end-effector.
      
         using namespace Eigen;
      //Roll pitch and yaw in Radians
      double roll = 1.571, pitch = 1.571, yaw = 	1.571;    
        
      Quaterniond q;
      q = AngleAxisd(roll, Vector3d::UnitX())* AngleAxisd(pitch, Vector3d::UnitY())* AngleAxisd(yaw, Vector3d::UnitZ());
      
      
    
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "initial move");

      move_group.setPoseReferenceFrame("base_link");
      move_group.allowReplanning(true);


      geometry_msgs::msg::Pose target_pose1;

      


        geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose("tool0").pose;

  move_group.setNamedTarget("up");
  move_group.move();
  

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose);

  start_pose.position.z = z ;

//  start_pose.orientation.w = q.coeffs()[3];
//  start_pose.orientation.x = q.coeffs()[0];
//  start_pose.orientation.y = q.coeffs()[1];
//  start_pose.orientation.z = q.coeffs()[1];

  waypoints.push_back(start_pose);

  start_pose.position.y = y;
  waypoints.push_back(start_pose);

  start_pose.position.x = x;
  waypoints.push_back(start_pose);

  

  


  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshhold = 0.2;
  const double step = 0.0001;
  double fraction = 0.0;
  int max_tries = 100;
  int attempts = 0;

  while(fraction < 1.0 && attempts < max_tries){
    fraction = move_group.computeCartesianPath(waypoints, step, jump_threshhold, trajectory);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"fraction attempt %d",attempts);
    attempts++;
    
  }

  if(fraction == 1){
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Path planning");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    plan.trajectory_ = trajectory;


    move_group.execute(plan);

    response->complete = true;
  }
  else {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Path planning failed with %d tries",max_tries);
    
  }

      
//////// to add --> iterative mover so straight lines also curret state knowledge for straight line paths too. also big in favoured over outwards BI the SI then SO then BO


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
