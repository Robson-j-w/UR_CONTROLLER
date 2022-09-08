#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>


using std::placeholders::_1;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_cartesian_sub");



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
  
     move_robot();
     RCLCPP_INFO(this->get_logger(), "returned2");

      

    }

    void move_robot()
    {
     
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_function2", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // BEGIN_TUTORIAL








// Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "ur_manipulator";

  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // We will use the
  // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  move_group.setPlanningTime(10);







// Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "test Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

 // RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));



move_group.setNamedTarget("start");
move_group.move();

/////

 move_group.clearPoseTargets();
 move_group.setStartStateToCurrentState();

//////

// Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a Cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations
  std::vector<geometry_msgs::msg::Pose> waypoints;
 

  geometry_msgs::msg::Pose target_pose3 = move_group.getCurrentPose().pose;
  RCLCPP_INFO(LOGGER, "Current Pose: %f", move_group.getCurrentPose().pose.position.x);
  RCLCPP_INFO(LOGGER, "Current Pose: %f", move_group.getCurrentPose().pose.position.y);
  RCLCPP_INFO(LOGGER, "Current Pose: %f", move_group.getCurrentPose().pose.position.z);
  
  RCLCPP_INFO(LOGGER, "moving to : %f %f %f",x,y,z);


  
  target_pose3.position.z = z;
  waypoints.push_back(target_pose3);  
  

  target_pose3.position.y = y;
  waypoints.push_back(target_pose3);  

  target_pose3.position.x = x;
  waypoints.push_back(target_pose3);  

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);


if(fraction==1)
{

  RCLCPP_INFO(LOGGER, "Path Fully planned");
}
  // Cartesian motions should often be slow, e.g. when approaching objects. The speed of Cartesian
  // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
  // the trajectory manually, as described `here <https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4>`_.
  // Pull requests are welcome.
  //
  // You can execute a trajectory like this.
  auto outcome = move_group.execute(trajectory); 
   RCLCPP_INFO(LOGGER, "Executing path");
  if(outcome)
  {
    move_group.stop();
    RCLCPP_INFO(LOGGER, "Completed stopping");

  }

  //std::cout <<outcome;


 






//////






      executor.cancel();


    

    }

  private:
    // Declare the subscription attribute
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    float x;
    float y;
    float z;

};


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