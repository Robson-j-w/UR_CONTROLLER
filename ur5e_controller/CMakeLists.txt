cmake_minimum_required(VERSION 3.5)
project(ur5e_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(moveit_common REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ur_robot_driver REQUIRED)
find_package(ur_bringup REQUIRED)
find_package(ur_controllers REQUIRED)
find_package(ur_description REQUIRED)
find_package(ur_moveit_config REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ur5e_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ur5e_interfaces REQUIRED)

#### custom service


####################
#Put the stuff here 
###################
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  rclpy
  rclcpp_action
  tf2_geometry_msgs
  tf2_ros
  moveit_core
  rviz_visual_tools
  moveit_visual_tools
  moveit_ros_planning_interface
  interactive_markers
  tf2_geometry_msgs
  moveit_ros_planning
  pluginlib
  Eigen3
  Boost
  control_msgs
  moveit_servo
  std_msgs
  ur5e_interfaces
)
####################
####################
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)



  add_executable(test_move src/cpp/test_move.cpp)
  target_include_directories(test_move
    PUBLIC include)
  ament_target_dependencies(test_move
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

  add_executable(move_function src/cpp/move_function.cpp)
   target_include_directories(move_function
    PUBLIC include)
  ament_target_dependencies(move_function
      ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

  add_executable(pub src/cpp/pub.cpp)
      target_include_directories(pub
       PUBLIC include)
     ament_target_dependencies(pub
         ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

  add_executable(move_function_mcpp src/cpp/move_function_mcpp.cpp)
         target_include_directories(move_function_mcpp
          PUBLIC include)
        ament_target_dependencies(move_function_mcpp
            ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

  add_executable(move_function_tutorial src/cpp/move_group_interface_tutorial.cpp)
         target_include_directories(move_function_tutorial
          PUBLIC include)
        ament_target_dependencies(move_function_tutorial
            ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

  add_executable(move_function2 src/new/move.cpp)
            target_include_directories(move_function2
             PUBLIC include)
           ament_target_dependencies(move_function2
               ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)


  add_executable(move_cartesian src/new/move_cartesian.cpp)
               target_include_directories(move_cartesian
                PUBLIC include)
              ament_target_dependencies(move_cartesian
                  ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
  add_executable(move_cartesian_service src/new/move_cartesian_service.cpp)
                  target_include_directories(move_cartesian_service
                   PUBLIC include)
                 ament_target_dependencies(move_cartesian_service
                     ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
  # add_executable(move_cartesian_sub src/new/move_cartesian_sub.cpp)
  #                    target_include_directories(move_cartesian_sub
  #                     PUBLIC include)
  #                   ament_target_dependencies(move_cartesian_sub
  #                       ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
  # add_executable(pub_t src/cpp/pub_topic.cpp)
  #                   target_include_directories(pub_t
  #                   PUBLIC include)
  #                 ament_target_dependencies(pub_t
  #                     ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)



  install(TARGETS
  test_move
  move_cartesian
  # move_cartesian_sub
  move_cartesian_service
  move_function
  move_function2
  pub
  # pub_t
  move_function_mcpp
  move_function_tutorial
  DESTINATION lib/${PROJECT_NAME})

####################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


######
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_include_directories(include)



#####
ament_package()
