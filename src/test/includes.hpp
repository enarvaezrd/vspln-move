#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <queue>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
//traj_client

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>

//uav_arm_tools
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

//Prediction
#include <sensor_msgs/LaserScan.h>

//UAV control message
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <thread>
#include <future>
#include <mutex>
#include <condition_variable>

#define PI 3.141592654

class Printer
{
public:
    Printer() {}
    void operator()(std::string str, double a = -1111, double b = -1111, double c = -1111, double d = -1111, double e = -1111, double f = -1111, double g = -1111)
    {
#ifndef PRINT
        return;
#endif
        std::vector<double> input;
        input.push_back(a);
        input.push_back(b);
        input.push_back(c);
        input.push_back(d);
        input.push_back(e);
        input.push_back(f);
        input.push_back(g);
        int strSize = 0;
        std::cout << "> " << str << ": ";
        strSize += 4;
        strSize += str.size();
        for (auto i : input)
        {
            //std::cout<<i;
            if (i != -1111)
            {
                std::ostringstream str_s;
                str_s << i;
                std::string str_t = str_s.str();
                std::cout << i << ", ";
                strSize += 2;
                strSize += str_t.size();
            }
        }
        std::cout << '\b';
        std::cout << ' ';
        std::cout << "\n";
        for (int i = 0; i < strSize; i++)
        {
            std::cout << '\b';
        }
        return;
    }
};

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;
class FollowTrajectoryClient
{
public:
  FollowTrajectoryClient();
  virtual ~FollowTrajectoryClient();

  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal);
  control_msgs::FollowJointTrajectoryGoal makeArmUpTrajectory(std::vector<double> joints_obj);
  actionlib::SimpleClientGoalState getState();
  
private:
  ros::NodeHandle nh_trajectory_client;
  ros::Publisher pub_trajectory_arm_command;
  ros::NodeHandle nh_;
  TrajClient traj_client_;
  ros::Subscriber joint_state_sub_;
  std::vector<std::string> joint_names_;
  bool got_joint_state_;
  std::vector<double> current_joint_state_;
  ros::AsyncSpinner spinner_;

  void boundValue(double &val, double maxv, double minv);
  void jointStateCB(const sensor_msgs::JointState::ConstPtr &msg);
  double wait_time_calc(std::vector<double> joints);
  std::chrono::time_point<std::chrono::high_resolution_clock> start;
  Printer Print;
};