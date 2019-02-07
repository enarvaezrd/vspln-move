#ifndef INCLUDE_LIBS
#define INCLUDE_LIBS

//ed_pmov
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>

//traj_client

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>

//uav_arm_tools
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <thread>
#include <mutex>
#include <condition_variable>

#define OPENCV_DRAW
#define PI 3.141592654
#define VERBOSE_
class Printer{
public:
    Printer(){}
    void operator()(std::string str, double a = -1111, double b = -1111 , double c = -1111,double d = -1111,double e = -1111,double f = -1111)
    {
        #ifdef VERBOSE 
            return;
        #endif 
        std::vector<double> input;
        input.push_back(a);
        input.push_back(b);
        input.push_back(c);
        input.push_back(d);
        input.push_back(e);
        input.push_back(f);
        int strSize=0;
        std::cout<<"> "<<str<<": ";
        strSize += 4;
        strSize += str.size();
        for(auto i : input )
        {
            //std::cout<<i;
            if (i != -1111)
            {
                std::ostringstream str_s;
                str_s << i;
                std::string str_t = str_s.str();
                std::cout<<i<<", ";
                strSize += 2;
                strSize += str_t.size();
            }

        }
        std::cout<<"\n";
        for (int i = 0; i < strSize; i++)
        {
            std::cout<<'\b';
        }
        return;
    }

};


#endif
