cmake_minimum_required(VERSION 2.8.3)
project(ed_pmov)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  moveit_core
  moveit_ros_planning_interface
  actionlib
  trajectory_msgs
  control_msgs
  sensor_msgs
  cv_bridge
)

find_package(OpenCV 3 REQUIRED)
add_compile_options(-std=c++14 -pthread)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES ed_control
 CATKIN_DEPENDS roscpp
  cv_bridge
  actionlib
  trajectory_msgs
  control_msgs
  sensor_msgs
#  DEPENDS system_lib
   DEPENDS moveit_core
 moveit_ros_planning_interface
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})

 add_executable(pmov src/pmov.cpp
	include src/armfcn.h
	src/rrtfunctions.h
	src/armbase.h
	src/uavarmfcns.h
	src/movfcns.h)


target_link_libraries(pmov ${catkin_LIBRARIES} ${OpenCS_LIBS} ${OpenCV_LIBRARIES})
add_dependencies(pmov ${catkin_EXPORTED_TARGETS})


add_executable(ed_pmov src/thread_src/main.cpp
	src/thread_src/uav_arm_tools.cpp
	src/thread_src/ed_pmov.cpp
	src/thread_src/trajectory_client.cpp
	src/thread_src/rrt_functions.cpp
	include src/thread_src/uav_arm_tools.hpp
	src/thread_src/ed_pmov.hpp
	src/thread_src/trajectory_client.hpp
	src/thread_src/includes.hpp
	src/thread_src/rrt_functions.hpp
	)

target_link_libraries(ed_pmov ${catkin_LIBRARIES} ${OpenCS_LIBS} ${OpenCV_LIBRARIES})
add_dependencies(ed_pmov ${catkin_EXPORTED_TARGETS})

add_executable(thrtest src/test/main.cpp)


target_link_libraries(thrtest ${catkin_LIBRARIES} ${OpenCS_LIBS} )
add_dependencies(thrtest ${catkin_EXPORTED_TARGETS})


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

