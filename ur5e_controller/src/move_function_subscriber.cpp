#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

using std::placeholders::_1;



class PositionControl : public rclcpp::Node
{
  public:
    // Constructor
    PositionControl()
    : Node("position_control")
    {
      // Create the subscription.
      subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "coords", 100, std::bind(&PositionControl::topic_callback, this, _1));

      

 
    }
  
  
    // Receives the message that is published over the topic
    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
      

      // Write the message that was received on the console window
      RCLCPP_INFO(this->get_logger(), "in");
      x = msg->data[0];
      y = msg->data[1];
      z = msg->data[2];
     RCLCPP_INFO(this->get_logger(), "x:%f , y:%f , z:%f", x,y,z);
     test();
     RCLCPP_INFO(this->get_logger(), "returned");
     move_robot();
     RCLCPP_INFO(this->get_logger(), "returned2");

      

    }
    void test()
    {
      RCLCPP_INFO(this->get_logger(), "test");
    }

    void move_robot()
    {
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
      RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());

      // We can also print the name of the end-effector link for this group.
      RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

      // We can get a list of all the groups in the robot:
      RCLCPP_INFO(this->get_logger(), "Available Planning Groups:");
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
      
      
    
      RCLCPP_INFO(this->get_logger(), "initial move");

      move_group.setPoseReferenceFrame("base_link");
      move_group.allowReplanning(true);


      geometry_msgs::msg::Pose target_pose1;

      


        geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose("tool0").pose;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose);

  start_pose.position.z = z;

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
  const double jump_threshhold = 0.0;
  const double step = 0.01;
  double fraction = 0.0;
  int max_tries = 30;
  int attempts = 0;

  while(fraction < 1.0 && attempts < max_tries){
    fraction = move_group.computeCartesianPath(waypoints, step, jump_threshhold, trajectory);
    attempts++;
  }

  if(fraction == 1){
    

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    move_group.execute(plan);
  }
  else {
    RCLCPP_INFO(this->get_logger(),"Path planning failed with %d tries",max_tries);
  }

      
//////// to add --> iterative mover so straight lines also curret state knowledge for straight line paths too. also big in favoured over outwards BI the SI then SO then BO


      executor.cancel();


    

    }

  private:
    // Declare the subscription attribute
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    float x;
    float y;
    float z;

};
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_tutorial");

int main(int argc, char * argv[])
{
  // Launch ROS 2
  
  rclcpp::init(argc, argv);
  RCLCPP_INFO(LOGGER, "Initialize node");
  // Prepare to receive messages that arrive on the topic
  rclcpp::spin(std::make_shared<PositionControl>());
  RCLCPP_INFO(LOGGER, "out");

   
  // Shutdown routine for ROS2
  rclcpp::shutdown();
  return 0;
}