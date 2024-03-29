cmake_minimum_required(VERSION 2.8.3)
project(yumi_controllers)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  kdl_wrapper
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
)
find_package(orocos_kdl REQUIRED)

catkin_package(
  DEPENDS orocos_kdl system_lib
  CATKIN_DEPENDS  kdl_parser kdl_wrapper moveit_core moveit_ros_planning_interface roscpp
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${orocos_kdl_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})
find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})

add_executable(kdl_controller src/kdl_controller.cpp)
add_executable(circlerun_pad_controller src/circlerun_pad_controller.cpp)
add_executable(yumi_pushing_left_arm src/yumi_pushing_left_arm.cpp)
add_executable(yumi_pushing_right_arm src/yumi_pushing_right_arm.cpp)
add_executable(yumi_right_arm src/yumi_right_arm.cpp)
add_executable(goto_init_jpose src/goto_init_jpose.cpp)
target_link_libraries(kdl_controller  ${catkin_LIBRARIES} )
target_link_libraries(circlerun_pad_controller  ${catkin_LIBRARIES} )
target_link_libraries(yumi_pushing_left_arm  ${catkin_LIBRARIES} )
target_link_libraries(yumi_pushing_right_arm  ${catkin_LIBRARIES} )
target_link_libraries(yumi_right_arm  ${catkin_LIBRARIES} )
target_link_libraries(goto_init_jpose  ${catkin_LIBRARIES} )
