#ifndef TRAJECTORY_CLIENT
#define TRAJECTORY_CLIENT
#include"includes.hpp"



#define NUM_JOINTS 6

namespace edArm_trajectory
{
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
      ros::NodeHandle nh_;
      TrajClient traj_client_;
      ros::Subscriber joint_state_sub_;
      std::vector<std::string> joint_names_;
      bool got_joint_state_;
      std::vector<double> current_joint_state_;
      ros::AsyncSpinner spinner_;

      void boundValue(double& val, double maxv,double minv);
      void jointStateCB(const sensor_msgs::JointState::ConstPtr &msg);
      double wait_time_calc(std::vector<double> joints);



    };
}
#endif
