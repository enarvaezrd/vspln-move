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

    RobotCommands(std::string odometry_str_, double uav_x, double uav_y, bool real_robots_) : uav_xpos(uav_x),
                                                                                              uav_ypos(uav_y),   //0.22-0.4
                                                                                              uav_altitude(1.1), //0.94; 1.1 for non contact
                                                                                              max_uav_correction(1.0),
                                                                                              odometry_str(odometry_str_),
                                                                                              real_robots_flag(real_robots_)
    {
        uav_msg_pub = nh_uav_ugv_comm.advertise<geometry_msgs::Twist>("/robot2/visual_local_guidance/uav_msg", 1); //commands for the UAV
        sub_ugv_Odom = nh_uav_ugv_comm.subscribe(odometry_str.c_str(), 1, &RobotCommands::UGV_Odom_Handler, this); ///robot1/robotnik_base_control/odom
        old_uav_command.linear.x = uav_xpos;
        old_uav_command.linear.x = uav_xpos;
    }

    void UGV_Odom_Handler(const nav_msgs::Odometry &ugv_odom);
    void Calculate_and_Send_Commands(geometry_msgs::Pose uav_local_pose, double height_correction, double x_corection, double y_corection);
    void Send_Empty_Commands();
    Angles ConvPosetoAngles(geometry_msgs::Pose pose);
    Printer Print;
    RobotState_ ugv_state;
    std::string odometry_str;
    geometry_msgs::Twist old_uav_command;
    double uav_speed;
    bool real_robots_flag;
};

#endif
