#ifndef ED_PMOV
#define ED_PMOV


#include "trajectory_client.hpp"



class Ed_Pmov{


public:
    edArm_trajectory::FollowTrajectoryClient arm;
    moveit::planning_interface::MoveGroupInterface group;


    Ed_Pmov() : group("pro_arm")
    {
        group.setPlannerId("RRTConnectkConfigDefault");//PRMstarkConfigDefault---RRTConnectkConfigDefault--RRTkConfigDefault--PRMkConfigDefault--RRTstarkConfigDefault
        group.setGoalTolerance(0.005);//0.004
        group.setGoalOrientationTolerance(0.008);//0.008
        group.setPlanningTime(0.1);

        RobotModelLoader robot_model_loader("robot1/robot_description");   //--

        kinematic_model = robot_model_loader.getModel();   //--
        robot_state::RobotStatePtr kinematic_statea(new robot_state::RobotState(kinematic_model));

        kinematic_state = kinematic_statea;//A(new robot_state::RobotState(kinematic_model));

        joint_model_group = kinematic_model->getJointModelGroup("pro_arm");
        joints_pub = nh_.advertise< control_msgs::FollowJointTrajectoryGoal>("/joints_data", 1);

    }

    geometry_msgs::Pose getCurrentPose();
    bool ReqMovement_byJointsValues(std::vector<double> );
    bool SendMovement_byJointsValues(std::vector<double> );
    bool ReqMovement_byPose(geometry_msgs::Pose , int );
    bool Check_Collision( std::vector<double> , int );
    void PrintCurrentPose(std::string);
    void PrintPose(std::string , geometry_msgs::Pose );
    double getDelayTime(){return delay_time;}
    void Sleep(long double);
    void tic();
    long double toc(); //regresa el tiempo transcurrido
    bool SendInitialPose();



private:

    typedef robot_model_loader::RobotModelLoader RobotModelLoader;//si se pasa a publico o antes de public: produce un error
    typedef moveit::planning_interface::MoveGroupInterface::Plan GroupPlan;
    typedef moveit::planning_interface::MoveItErrorCode ErrorCode;
    typedef moveit_msgs::MoveItErrorCodes MoveitCodes;

    robot_state::RobotStatePtr kinematic_state ;
    Printer Print;
    robot_model::RobotModelPtr kinematic_model;
    const robot_state::JointModelGroup* joint_model_group;
    double delay_time;
    geometry_msgs::Pose currentPose;
    ros::Publisher joints_pub;
    ros::NodeHandle nh_;

    GroupPlan my_plan;

    double tic_clock_time;


};



#endif
