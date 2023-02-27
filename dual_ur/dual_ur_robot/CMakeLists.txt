cmake_minimum_required(VERSION 3.0.2)
project(dual_ur_robot)

find_package(catkin REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
  tf_conversions
  moveit_msgs
  moveit_core
  moveit_visual_tools
  moveit_ros_planning_interface
  rospy
)


catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  tf
  tf_conversions
  moveit_msgs 
  moveit_core
  moveit_visual_tools
  moveit_ros_planning_interface

  roscpp
)

catkin_python_setup()


install(FILES etc/rightarm_controllers.yaml etc/leftarm_controllers.yaml etc/view_robot.rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/etc
)
install(FILES
  launch/dual_robot_startup.launch
  launch/load_dual_description.launch
  launch/view_dual_robot.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(FILES
  resources/ursim/programs_rightarm/default.installation
  resources/ursim/programs_rightarm/default.variables
  resources/ursim/programs_rightarm/external_control.urp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources/ursim/programs_rightarm
)
install(FILES
  resources/ursim/programs_leftarm/default.installation
  resources/ursim/programs_leftarm/default.variables
  resources/ursim/programs_leftarm/external_control.urp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources/ursim/programs_leftarm
)
install(FILES
  resources/ursim/urcaps/externalcontrol-1.0.5.jar
  resources/ursim/urcaps/ros_as_a_service-0.1-beta4.jar
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources/ursim/urcaps
)
install(FILES scripts/docker_rightarm_leftarm.sh
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
install(FILES urdf/dual_robot.xacro
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)


include_directories(${catkin_INCLUDE_DIRS})
add_executable(collision_object_publisher src/collision_object/collision_object_publisher.cpp)
target_link_libraries(collision_object_publisher ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

