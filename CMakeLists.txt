cmake_minimum_required(VERSION 2.8.3)
project(ed_pmov)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  actionlib
  trajectory_msgs
  control_msgs
  sensor_msgs
apriltag_ros
)

find_package(OpenCV 3 REQUIRED)
add_compile_options(-std=c++14 -pthread)


catkin_package(
 INCLUDE_DIRS #include
#  LIBRARIES ed_control
 CATKIN_DEPENDS roscpp
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

# add_executable(pmov src/pmov.cpp
	#include 
#	src/armfcn.h
#	src/rrtfunctions.h
#	src/armbase.h
#	src/uavarmfcns.h
#	src/movfcns.h)


#target_link_libraries(pmov ${catkin_LIBRARIES} ${OpenCS_LIBS} ${OpenCV_LIBRARIES})
#add_dependencies(pmov ${catkin_EXPORTED_TARGETS})


add_executable(ed_pmov src/thread_src/main.cpp
	src/thread_src/uav_arm_tools.cpp
	src/thread_src/ed_pmov.cpp
	src/thread_src/trajectory_client.cpp
	src/thread_src/rrt_functions.cpp
	src/thread_src/prediction.cpp
	src/thread_src/map_gen.cpp
	src/thread_src/uav_ugv_commands.cpp
	#include
	)
add_dependencies(ed_pmov ${catkin_EXPORTED_TARGETS})


target_link_libraries(ed_pmov ${catkin_LIBRARIES} ${OpenCS_LIBS} ${OpenCV_LIBRARIES})



add_executable(ed_ik_solver src/thread_src/ik_solver/ed_pmov.cpp)

target_link_libraries(ed_ik_solver ${catkin_LIBRARIES} ${OpenCS_LIBS} ${OpenCV_LIBRARIES})

add_dependencies(ed_ik_solver ${catkin_EXPORTED_TARGETS})



add_executable(only_uav_commands src/thread_src/main_only_uav_orders.cpp
	src/thread_src/uav_arm_tools.cpp
	src/thread_src/ed_pmov.cpp
	src/thread_src/trajectory_client.cpp
	src/thread_src/rrt_functions.cpp
	src/thread_src/prediction.cpp
	src/thread_src/map_gen.cpp
	src/thread_src/uav_ugv_commands.cpp
	#include
	)
target_link_libraries(only_uav_commands ${catkin_LIBRARIES} ${OpenCS_LIBS} ${OpenCV_LIBRARIES})
add_dependencies(only_uav_commands ${catkin_EXPORTED_TARGETS})




install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

