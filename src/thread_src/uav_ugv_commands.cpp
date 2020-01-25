#ifndef UAV_UGV_COMMANDS_CODE
#define UAV_UGV_COMMANDS_CODE

#include "uav_ugv_commands.hpp"

Angles RobotCommands::ConvPosetoAngles(geometry_msgs::Pose pose) //Convertir quaternion de pose en angulos
{
    Angles angles;
    Quat q;
    q.x = pose.orientation.x;
    q.y = pose.orientation.y;
    q.z = pose.orientation.z;
    q.w = pose.orientation.w;
    double ysqr = q.y * q.y;
    // roll (x-axis rotation)
    double t00 = +2.0 * (q.w * q.x + q.y * q.z);
    double t11 = +1.0 - 2.0 * (q.x * q.x + ysqr);
    double roll = std::atan2(t00, t11);

    // pitch (y-axis rotation)
    double t22 = +2.0 * (q.w * q.y - q.z * q.x);
    t22 = t22 > 1.0 ? 1.0 : t22;
    t22 = t22 < -1.0 ? -1.0 : t22;
    double pitch = std::asin(t22);

    // yaw (z-axis rotation)
    double t33 = +2.0 * (q.w * q.z + q.x * q.y);
    double t44 = +1.0 - 2.0 * (ysqr + q.z * q.z);
    double yaw = std::atan2(t33, t44);
    angles.roll = roll;
    angles.pitch = pitch;
    angles.yaw = yaw;
    return angles;
}

void RobotCommands::Calculate_and_Send_Commands(geometry_msgs::Pose uav_local_pose, double height_correction, double y_corection)
{
    Angles UAV_yaw_Angle = ConvPosetoAngles(uav_local_pose);    //tal vez es necesario sacar el angulo del uav desde marker pose
    float x_error_local = uav_local_pose.position.x - uav_xpos; //Error calculation in local coordinates
    float y_error_local = uav_local_pose.position.y - uav_ypos;       //(uav_ypos + (y_corection - uav_ypos) / 4.0);
    //std::cout<<"UAV LOCAL ERRORS: "<< x_error_local<<", y: "<< y_error_local<<std::endl;
    UAV_yaw_Angle.yaw *= -1.0; //Inverse rotation


    UAV_yaw_Angle.yaw = fmod(UAV_yaw_Angle.yaw, PI);
    float x_error_uav = x_error_local * sin(UAV_yaw_Angle.yaw) + y_error_local * cos(UAV_yaw_Angle.yaw); //Rotation to UAV coordinates
    float y_error_uav = x_error_local * cos(UAV_yaw_Angle.yaw) - y_error_local * sin(UAV_yaw_Angle.yaw);

    if (x_error_uav > max_uav_correction)
        x_error_uav = max_uav_correction;
    if (x_error_uav < -max_uav_correction)
        x_error_uav = -max_uav_correction;
    if (y_error_uav > max_uav_correction)
        y_error_uav = max_uav_correction;
    if (y_error_uav < -max_uav_correction)
        y_error_uav = -max_uav_correction;

    geometry_msgs::Twist uav_command;

    if (real_robots_flag)
    {
        uav_command.linear.x = y_error_uav;
        uav_command.linear.y = x_error_uav;
    }
    else
    {
        uav_command.linear.x = -x_error_uav;
        uav_command.linear.y = y_error_uav;
    }
    // std::cout << "UAV ERRORS: x: " << uav_command.linear.x << ", y: " << uav_command.linear.y << std::endl;
    uav_command.linear.z = uav_altitude + height_correction; //just sending the altitude info, needs to be recalculated in a_control side

    uav_command.angular.x = 0.0;
    uav_command.angular.y = 0.0;
    uav_command.angular.z = 0.0;
    uav_msg_pub.publish(uav_command);

    uav_speed = 0.1 * (abs(uav_command.linear.x - old_uav_command.linear.x) + abs(uav_command.linear.x - old_uav_command.linear.x));
    old_uav_command = uav_command;
    return;
}

void RobotCommands::Send_Empty_Commands()
{
    geometry_msgs::Twist uav_command;
    uav_command.linear.x = 0.0;
    uav_command.linear.y = 0.0;
    uav_command.linear.z = uav_altitude; //just sending the altitude info, needs to be recalculated in a_control side
    uav_command.angular.x = 0.0;
    uav_command.angular.y = 0.0;
    uav_command.angular.z = 0.0;
    uav_msg_pub.publish(uav_command);
    return;
}
void RobotCommands::UGV_Odom_Handler(const nav_msgs::Odometry &ugv_odom)
{
    ugv_state.position.x = ugv_odom.pose.pose.position.x;
    ugv_state.position.y = ugv_odom.pose.pose.position.y;
    ugv_state.position.z = ugv_odom.pose.pose.position.z;
    ugv_state.orientation.x = ugv_odom.pose.pose.orientation.x;
    ugv_state.orientation.y = ugv_odom.pose.pose.orientation.y;
    ugv_state.orientation.z = ugv_odom.pose.pose.orientation.z;
    ugv_state.orientation.w = ugv_odom.pose.pose.orientation.w;

    ugv_state.velocity_linear.dx = ugv_odom.twist.twist.linear.x;
    ugv_state.velocity_linear.dy = ugv_odom.twist.twist.linear.y;
    ugv_state.velocity_linear.dz = ugv_odom.twist.twist.linear.z;
    ugv_state.velocity_angular.dx = ugv_odom.twist.twist.angular.x;
    ugv_state.velocity_angular.dy = ugv_odom.twist.twist.angular.y;
    ugv_state.velocity_angular.dz = ugv_odom.twist.twist.angular.z;
    return;
}

#endif
