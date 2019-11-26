#ifndef MAIN_APP
#define MAIN_APP
#define REAL_ROBOTS_

#include <sensor_msgs/Joy.h>
#include "rrt_functions.hpp"

using namespace std;

bool new_message_received = false;
sensor_msgs::Joy joystick_msg;
//---------------JOYSTICK------------------------
void Joy_Handler(const sensor_msgs::Joy &joystick)
{
    joystick_msg = joystick;
#ifndef WIRELESS_CONTROLLER
    double T2 = joystick_msg.axes[2];

    joystick_msg.axes[5] = T2;
#endif
    new_message_received = true;
}
int main(int argc, char **argv)
{
    //==============================CONFIGURATION PARAMETERS===========================

    int image_size = 600;

    int d_prv = 5;      // profundidad de datos previos disponibles para prediccion
    int d_pr_m = 3;     // datos previos a usar para calculo de mean values
    int prof_expl = 10; // Profundidad de exploracion  Esz=prof_f
    int Map_size = image_size;
    float scale = 1.0;
    double rrt_extension = 0.33; //extension of each rrt step for regression 0.38 0.28
    int num_nodes_per_region = prof_expl + 15;
    double rad_int = 0.26;
    double rad_ext = 0.425;
    double armMinAltitude = 0.47;
    bool load_joints_state_sub;
    std::string controller_topic = "/joy";
    double Docking_Alt_Lim_ = 0.83;
    double DockingFactor = 0.062;
#ifdef REAL_ROBOTS
    load_joints_state_sub = true;
    string odom_str = "/robot1/odom"; ///robot1/robotnik_base_control/odom  ;   /robot1/odom
    string laser_topic = "/scan";     ///robot1/front_laser/scan SIMM ;  /scan REAL
#else
    string odom_str = "/robot1/odom";                ///robot1/robotnik_base_control/odom  ;   /robot1/odom
    string laser_topic = "/robot1/front_laser/scan"; ///robot1/front_laser/scan SIMM ;  /scan REAL
    load_joints_state_sub = false;
#endif

    double UAV_position_x = -0.18;
    double UAV_position_y = 0.22;
    //==================================================================================

    ros::init(argc, argv, "arm_program");
    ros::NodeHandle nodle_handle;

    // Jopystick subscriber
    ros::Subscriber sub_joystick = nodle_handle.subscribe("/joy", 1, Joy_Handler);

    ros::Rate loop_rate(30);
    Printer Print;
    std::vector<double> joint_valuesT(6);
    joint_valuesT[0] = -PI / 2 + 0.1;
    joint_valuesT[1] = 2.0; //PI/2;
    joint_valuesT[2] = 2.0; //PI/2;
    joint_valuesT[3] = 0.0; // PI/2;
    joint_valuesT[4] = 0.0;
    joint_valuesT[5] = 0.0; // PI/2;
    rrt_planif::RRT RRT_model(image_size, d_prv, d_pr_m, prof_expl, scale, num_nodes_per_region, load_joints_state_sub);
    sleep(2.0);

    RRT_model.ArmModel.SendMovement_byJointsValues(joint_valuesT);

    RRT_model.ArmModel.PrintModelInfo();

    RRT_model.ArmModel.PrintCurrentPose("====>STARTING POSE ::::");
    float alturap = armMinAltitude; //0.21

    geometry_msgs::Pose target_pose = RRT_model.ArmModel.getCurrentPose();

    target_pose.orientation.w = 0.0; //0.1
    target_pose.orientation.x = 0.0; //0.10
    target_pose.orientation.y = 1.0;
    target_pose.orientation.z = 0.0;

    target_pose.position.x = UAV_position_x; //0.1
    target_pose.position.y = UAV_position_y;
    target_pose.position.z = alturap;

    bool reqState = RRT_model.ArmModel.ReqMovement_byPose(target_pose);
    sleep(2.0);

    geometry_msgs::Pose target_posea = RRT_model.ArmModel.getCurrentPose();

    int TrackingState = 0;
    int ThreadStep = 0;

    auto ArmGoal = RRT_model.ArmModel.Req_Joints_byPose_FIx_Orientation(target_posea); //return to origin
    double Gainx = 0.01;
    double Gainy = 0.01;
    bool update_gain = true;
    while (ros::ok())
    {
        if (!new_message_received)
        {
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        if (joystick_msg.axes[5] == 0.0 || joystick_msg.axes[5] == 1.0)
        {
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        if (update_gain)
        {
            if (joystick_msg.axes[6] == -1.0)
                Gainy += 0.01;
            if (joystick_msg.axes[6] == 1.0)
                Gainy -= 0.01;
            if (joystick_msg.axes[7] == -1.0)
                Gainx -= 0.01;
            if (joystick_msg.axes[7] == 1.0)
                Gainx -= 0.01;
            update_gain = false;
        }
        if (joystick_msg.axes[6] == 0.0 && joystick_msg.axes[7] == 0.0)
            update_gain = true;

        double max = 0.2;
        if (Gainy < -max)
            Gainy = -max;
        if (Gainy > max)
            Gainy = max;
        if (Gainx < -max)
            Gainx = -max;
        if (Gainx > max)
            Gainx = max;
        target_posea.position.x = Gainx;
        target_posea.position.y = Gainy;

        ArmGoal = RRT_model.ArmModel.Req_Joints_byPose_FIx_Orientation(target_posea);
        if (ArmGoal.trajectory.points.size() > 0.0)
        {
            ArmGoal = RRT_model.SteerJoints(ArmGoal);
            Print("JOINTS REQUEST", ArmGoal.trajectory.points[0].positions[0], ArmGoal.trajectory.points[0].positions[1]);
            RRT_model.ArmModel.Request_Movement_byJointsTrajectory(ArmGoal);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("node finished correctly\n");
}
#endif
