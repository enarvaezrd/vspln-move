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
        kinematic_state = kinematic_statea;
        joint_model_group = kinematic_model->getJointModelGroup("pro_arm");
       // joints_pub = nh_.advertise< control_msgs::FollowJointTrajectoryGoal>("/joints_data", 1);    //uncomment if necessary     
    }

    geometry_msgs::Pose getCurrentPose();

    void CheckandFixPoseRequest(geometry_msgs::Pose &pose_req);
    bool ReqMovement_byJointsValues(std::vector<double> );
    bool SendMovement_byJointsValues(std::vector<double> );
    bool ReqMovement_byPose(geometry_msgs::Pose );
    bool ReqMovement_byPose_FIx_Orientation(geometry_msgs::Pose pose_req);
   bool Check_Collision_TypeA(std::vector<double> Position);
   bool Check_Collision_TypeB(std::vector<double> Position);
    void PrintCurrentPose(std::string);
    void PrintPose(std::string , geometry_msgs::Pose );
    std::chrono::microseconds getDelayTime(){return delay_time;}
    void Sleep(std::chrono::microseconds);
    void tic();
    std::chrono::microseconds  toc(); //regresa el tiempo transcurrido
    std::chrono::microseconds toc(std::chrono::time_point<std::chrono::high_resolution_clock> );
    bool SendInitialPose();
    void PrintModelInfo(){
        Print("Model frame");Print( kinematic_model->getModelFrame().c_str());
        Print("Joint Group Name");Print(joint_model_group->getName().c_str());
        Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
        Eigen::MatrixXd jacobian;
        kinematic_state->getJacobian(joint_model_group,
                             kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                             reference_point_position, jacobian);
        ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
    }



private:

    typedef robot_model_loader::RobotModelLoader RobotModelLoader;//si se pasa a publico o antes de public: produce un error
    typedef moveit::planning_interface::MoveGroupInterface::Plan GroupPlan;
    typedef moveit::planning_interface::MoveItErrorCode ErrorCode;
    typedef moveit_msgs::MoveItErrorCodes MoveitCodes;

    robot_state::RobotStatePtr kinematic_state ;
    Printer Print;
    robot_model::RobotModelPtr kinematic_model;
    const robot_state::JointModelGroup* joint_model_group;
    std::chrono::microseconds delay_time;
    geometry_msgs::Pose currentPose;
    ros::Publisher joints_pub;
    ros::NodeHandle nh_;

    GroupPlan my_plan;

    std::chrono::time_point<std::chrono::high_resolution_clock> tic_clock_time;


};

#endif
