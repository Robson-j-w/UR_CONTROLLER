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
  std::shared_ptr<rclcpp::Node> motion_planning_api_tutorial_node =
      rclcpp::Node::make_shared("move_function", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(motion_planning_api_tutorial_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // BEGIN_TUTORIAL
  // Start
  // ^^^^^
  // Setting up to start using a planner is pretty easy. Planners are
  // setup as plugins in MoveIt and you can use the ROS pluginlib
  // interface to load any planner that you want to use. Before we can
  // load the planner, we need two objects, a RobotModel and a
  // PlanningScene. We will start by instantiating a
  // :moveit_codedir:`RobotModelLoader<moveit_ros/planning/robot_model_loader/include/moveit/robot_model_loader/robot_model_loader.h>`
  // object, which will look up the robot description on the ROS
  // parameter server and construct a
  // :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>`
  // for us to use.
  const std::string PLANNING_GROUP = "ur_manipulator";
  robot_model_loader::RobotModelLoader robot_model_loader(motion_planning_api_tutorial_node, "robot_description");
  const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
  /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

  // Using the
  // :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>`,
  // we can construct a
  // :moveit_codedir:`PlanningScene<moveit_core/planning_scene/include/moveit/planning_scene/planning_scene.h>`
  // that maintains the state of the world (including the robot).
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  // Configure a valid robot state
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

  // We will now construct a loader to load a planner, by name.
  // Note that we are using the ROS pluginlib library here.
  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name = "ompl_interface/OMPLPlanner";

  // We will get the name of planning plugin we want to load
  // from the ROS parameter server, and then load the planner
  // making sure to catch all exceptions.
  if (!motion_planning_api_tutorial_node->get_parameter("planning_plugin", planner_plugin_name))
    RCLCPP_FATAL(LOGGER, "Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, motion_planning_api_tutorial_node,
                                      motion_planning_api_tutorial_node->get_namespace()))
      RCLCPP_FATAL(LOGGER, "Could not initialize planner instance");
    RCLCPP_INFO(LOGGER, "Using planning interface '%s'", planner_instance->getDescription().c_str());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (const auto& cls : classes)
      ss << cls << " ";
    RCLCPP_ERROR(LOGGER, "Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_plugin_name.c_str(),
                 ex.what(), ss.str().c_str());
  }

  moveit::planning_interface::MoveGroupInterface move_group(motion_planning_api_tutorial_node, PLANNING_GROUP);

      
//////// to add --> iterative mover so straight lines also curret state knowledge for straight line paths too. also big in favoured over outwards BI the SI then SO then BO


      // Pose Goal
  // ^^^^^^^^^
  // We will now create a motion plan request for the arm of the Panda
  // specifying the desired pose of the end-effector as input.
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "panda_link0";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.4;
  pose.pose.position.z = 0.75;
  pose.pose.orientation.w = 1.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  // We will create the request as a constraint using a helper function available
  // from the
  // :moveit_codedir:`kinematic_constraints<moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.h>`
  // package.
  moveit_msgs::msg::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("tool0", pose, tolerance_pose, tolerance_angle);

  req.group_name = PLANNING_GROUP;
  req.goal_constraints.push_back(pose_goal);

  // We now construct a planning context that encapsulate the scene,
  // the request and the response. We call the planner using this
  // planning context
  planning_interface::PlanningContextPtr context =
      planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
    
  }


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
