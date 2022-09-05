#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#define BOOST_BIND_NO_PLACEHOLDERS


// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <geometry_msgs/msg/point_stamped.h>

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
      "coords", 10, std::bind(&PositionControl::topic_callback, this, _1));

      

 
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

      // This enables loading undeclared parameters
      // best practice would be to declare parameters in the corresponding classes
      // and provide descriptions about expected use
      node_options.automatically_declare_parameters_from_overrides(true);
      rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("control", "", node_options);

      // We spin up a SingleThreadedExecutor for the current state monitor to get information
      // about the robot's state.
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node);
      std::thread([&executor]() { executor.spin(); }).detach();

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////USING MOVE GROUP INTERFACE   ----- COULD TRY MOVEIT CPP API FOR COMPLEX AND BETTER PERFORMANCE//////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



      
      

      static const std::string PLANNING_GROUP = "ur_manipulator";
      

      auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
      moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

      auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
      auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
      auto robot_start_state = planning_components->getStartState();
      auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);
      


       // Plan #1
  // ^^^^^^^
  //
  // We can set the start state of the plan to the current state of the robot
  planning_components->setStartStateToCurrentState();

  // The first way to set the goal of the plan is by using geometry_msgs::PoseStamped ROS message type as follow
  geometry_msgs::msg::PoseStamped target_pose1;
  target_pose1.header.frame_id = "base_link";
  target_pose1.pose.orientation.w = 1.0;
  target_pose1.pose.position.x = 0.28;
  target_pose1.pose.position.y = -0.2;
  target_pose1.pose.position.z = 0.5;
  planning_components->setGoal(target_pose1, "tool0");

  // Now, we call the PlanningComponents to compute the plan and visualize it.
  // Note that we are just planning
  auto plan_solution1 = planning_components->plan();

  // Check if PlanningComponents succeeded in finding the plan
  if (plan_solution1)
  {
    planning_components->execute(); // Execute the plan 
  }
      
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