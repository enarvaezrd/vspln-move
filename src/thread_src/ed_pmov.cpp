#ifndef ED_PMOV_CODE
#define ED_PMOV_CODE

#include "ed_pmov.hpp"



bool Ed_Pmov::ReqMovement_byJointsValues(std::vector<double> joints_values)
{
    bool state = false;


    group.setJointValueTarget(joints_values);
    ErrorCode stateC = group.plan(my_plan);
    if (stateC == MoveitCodes::SUCCESS)
    {
        control_msgs::FollowJointTrajectoryGoal goaled;
        goaled = arm.makeArmUpTrajectory(joints_values);
        arm.startTrajectory(goaled);//Inicio de trayectoria en GAZEBO
        state=true;
    }
    else
    {
         Print("NO solution for joint request!");;state=false;
    }
    return state;
}
bool Ed_Pmov::SendMovement_byJointsValues(std::vector<double> joints_values)
{
    control_msgs::FollowJointTrajectoryGoal goaled;
    goaled = arm.makeArmUpTrajectory(joints_values);
    arm.startTrajectory(goaled);//Inicio de trayectoria en GAZEBO
}
bool Ed_Pmov::SendInitialPose()
{
    std::vector<double> joint_values(6);
    joint_values[0] = 0.0;
    joint_values[1] = -PI/2;
    joint_values[2] = -PI/2;
    joint_values[3] = 0.0;// PI/2;
    joint_values[4] = 0.0;
    joint_values[5] = PI/2;
    Print("sending initial position ...");
    return SendMovement_byJointsValues(joint_values);
}




bool Ed_Pmov::ReqMovement_byPose(geometry_msgs::Pose pose_req,int type)
//type 1 with normal execution, type 2 for last joint preference
{
    ros::Duration tiempo_traj(0.0);
    bool fk = kinematic_state->setFromIK(joint_model_group, pose_req, 1, 0.1);

    if (fk)
    {
        std::vector<double> jv(6);
        control_msgs::FollowJointTrajectoryGoal goale;
        kinematic_state->copyJointGroupPositions(joint_model_group, jv);
        if (type==2)
        {
            std::vector<double> jvT(6);
            geometry_msgs::Pose TPoseTemp = pose_req;
            TPoseTemp.orientation.w = 0.0;
            TPoseTemp.orientation.x = 0.0;
            TPoseTemp.orientation.y = 1.0;
            TPoseTemp.orientation.z = 0.0;
            bool found_ikO = kinematic_state->setFromIK(joint_model_group, TPoseTemp, 1, 0.1);
            if (found_ikO)
            {
                kinematic_state->copyJointGroupPositions(joint_model_group, jvT);  //de jv saco posicion de joints 0 a 5
                jv[0] = jvT[0];
                jv[1] = jvT[1];
                jv[2] = jvT[2];
                jv[3] = jvT[3];
                jv[4] = jvT[4];
                //std::cout<<"last joint todo: "<<jv[5]<<std::endl;
                //std::cout<<"last joint simple: "<<jvT[5]<<std::endl;
            }
        }
        //std::cout<<"joints : "<<jv[0]<<" "<<jv[1]<<" "<<jv[2]<<" "<<jv[3]<<" "<<jv[4]<<" "<<jv[5]<<std::endl;
        goale = arm.makeArmUpTrajectory(jv);

        //int numpoints6 = goale.trajectory.points.size()-1;//escoger el punto final ya que empieza desde 0
        tiempo_traj = goale.trajectory.points[1].time_from_start; //tiempo del punto final
        delay_time = tiempo_traj.toSec();
        //usleep(1000000*(delay_time));
        arm.startTrajectory(goale);//Inicio de trayectoria en GAZEBO
    }
    else
    {

        PrintPose("NO solution for pose request!",pose_req );
        //SendInitialPose();
       // Print("Initial Position sent!");
    }
    return fk;
}


bool Ed_Pmov::Check_Collision( std::vector<double> Position, int type)
//type - 1 para solo posicion y 2 para posicion y orientacion juntas
{
    geometry_msgs::Pose CheckPose;
    CheckPose.position.x=Position[0];
    CheckPose.position.y=Position[1];
    CheckPose.position.z=Position[2];

    if (type==2) //cambiar a 3
    {
        CheckPose.orientation.w=Position[3];
        CheckPose.orientation.x=Position[4];
        CheckPose.orientation.y=Position[5];
        CheckPose.orientation.z=Position[6];
    }
    else
    {
        CheckPose.orientation.w=  0.0;
        CheckPose.orientation.x=  0.0;
        CheckPose.orientation.y= 1.0;
        CheckPose.orientation.z=  0.0;
    }

    bool found_ik;
    found_ik= kinematic_state->setFromIK(joint_model_group, CheckPose, 1, 0.01);

    return found_ik;
}


geometry_msgs::Pose Ed_Pmov::getCurrentPose()
{
    currentPose=group.getCurrentPose().pose;
    return currentPose;
}


void Ed_Pmov::PrintCurrentPose(std::string workspace)
{
    currentPose=group.getCurrentPose().pose;
    Print(workspace + ", Crrnt Position x,y,z: ",currentPose.position.x,currentPose.position.y,currentPose.position.z);
    Print(workspace + ", Crrnt Orientation ow,ox,oy,oz: ",currentPose.orientation.w,currentPose.orientation.x,currentPose.orientation.y,currentPose.orientation.z);
    return;
}
void Ed_Pmov::PrintPose(std::string workspace,  geometry_msgs::Pose req_pose)
{
    Print(workspace + ", Position x,y,z", req_pose.position.x,req_pose.position.y,req_pose.position.z);
    Print(workspace + ", Orientation ow, ox, oy, oz", req_pose.orientation.w, req_pose.orientation.x,req_pose.orientation.y,req_pose.orientation.z);
    //std::cout<<workspace<<", Position x: "<<req_pose.position.x<<" y: "<<req_pose.position.y<<" z: "<<req_pose.position.z<< std::endl;
    //std::cout<<workspace<<", Orientation ow: "<<req_pose.orientation.w<<" ox: "<<req_pose.orientation.x<<" oy: "<<req_pose.orientation.y<<" oz: "<<req_pose.orientation.z<< std::endl;
    return;
}

void Ed_Pmov::Sleep(long double elapsed_time)
{
    long double time_for_sleep = delay_time - elapsed_time;
    if (time_for_sleep <= 0.0) time_for_sleep=0.0;
    usleep(1000000*(time_for_sleep + 0.0001));
}
void Ed_Pmov::tic()
{
    tic_clock_time=0;
    tic_clock_time = clock();
}
long double Ed_Pmov::toc()
{
    double elapsed_time_clocks = clock() - tic_clock_time;
    long double elapsed_time = elapsed_time_clocks*1.0/CLOCKS_PER_SEC;//tiempo transcurrido en el codigo
    return elapsed_time;
}

#endif
