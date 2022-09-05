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

      

    }
    void test()
    {
      RCLCPP_INFO(this->get_logger(), "test");
    }

    void move_robot()
    {



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