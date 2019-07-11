#ifndef UAV_UGV_COMMANDS
#define UAV_UGV_COMMANDS

#include "includes.hpp"

class RobotCommands
{
public:
    geometry_msgs::Twist uav_msg;
    ros::NodeHandle nh_uav_ugv_comm;
    ros::Publisher uav_msg_pub;
    ros::Subscriber sub_ugv_Odom; //Marker pose
    float uav_xpos;
    float uav_ypos;
    float uav_altitude;
    float max_uav_correction;

    RobotCommands() :
    uav_xpos(0.1), 
    uav_ypos(0.27),
    uav_altitude(1.5),
    max_uav_correction(0.5)
    {
        uav_msg_pub = nh_uav_ugv_comm.advertise<geometry_msgs::Twist>("/robot2/visual_local_guidance/uav_msg", 1); //commands for the UAV
        sub_ugv_Odom    = nh_uav_ugv_comm.subscribe("/robot1/robotnik_base_control/odom", 1, &RobotCommands::UGV_Odom_Handler, this);
    }

    void UGV_Odom_Handler(const nav_msgs::Odometry &ugv_odom);
    void Calculate_and_Send_Commands(geometry_msgs::Pose uav_local_pose);
    void Send_Empty_Commands();
    Angles ConvPosetoAngles(geometry_msgs::Pose pose);
    Printer Print;
    RobotState_ ugv_state;
};

#endif
