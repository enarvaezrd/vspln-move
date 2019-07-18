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
        arm.startTrajectory(goaled); //Inicio de trayectoria en GAZEBO
        state = true;
    }
    else
    {
        Print("NO solution for joint request!");
        ;
        state = false;
    }
    return state;
}
bool Ed_Pmov::SendMovement_byJointsValues(std::vector<double> joints_values)
{
    control_msgs::FollowJointTrajectoryGoal goaled;
    goaled = arm.makeArmUpTrajectory(joints_values);
    arm.startTrajectory(goaled); //Inicio de trayectoria en GAZEBO
}
bool Ed_Pmov::SendInitialPose()
{
    std::vector<double> joint_values(6);
    joint_values[0] = 0.0;
    joint_values[1] = -PI / 2;
    joint_values[2] = -PI / 2;
    joint_values[3] = 0.0; // PI/2;
    joint_values[4] = 0.0;
    joint_values[5] = PI / 2;
    Print("sending initial position ...");
    return SendMovement_byJointsValues(joint_values);
}

void Ed_Pmov::CheckandFixPoseRequest(geometry_msgs::Pose &pose_req)
{
    int round_factor = 100000;
    pose_req.position.x = roundf(pose_req.position.x * round_factor) / round_factor;
    pose_req.position.y = roundf(pose_req.position.y * round_factor) / round_factor;
    pose_req.position.z = roundf(pose_req.position.z * round_factor) / round_factor;
    pose_req.orientation.x = roundf(pose_req.orientation.x * round_factor) / round_factor;
    pose_req.orientation.y = roundf(pose_req.orientation.y * round_factor) / round_factor;
    pose_req.orientation.z = roundf(pose_req.orientation.z * round_factor) / round_factor;
    pose_req.orientation.w = roundf(pose_req.orientation.w * round_factor) / round_factor;
    return;
}

bool Ed_Pmov::ReqMovement_byPose(geometry_msgs::Pose pose_req)
//type 1 with normal execution, type 2 for last joint preference
{
    ros::Duration tiempo_traj(0.0);
    //CheckandFixPoseRequest(pose_req);
    bool fk = kinematic_states_[0]->setFromIK(joint_model_groups_[0], pose_req, 2, 0.006);

    if (fk)
    {
        std::vector<double> jv(6);
        control_msgs::FollowJointTrajectoryGoal goale;
        kinematic_states_[0]->copyJointGroupPositions(joint_model_groups_[0], jv);
        goale = arm.makeArmUpTrajectory(jv);
        // tiempo_traj = goale.trajectory.points[1].time_from_start; //tiempo del punto final
        //delay_time = std::chrono::microseconds(int(tiempo_traj.toSec() * 1000000));
        //usleep(1000000*(delay_time));
        arm.startTrajectory(goale); //Inicio de trayectoria en GAZEBO
    }
    else
    {

        PrintPose("NO solution for pose request!", pose_req);
        //SendInitialPose();
        // Print("Initial Position sent!");
    }
    return fk;
}
bool Ed_Pmov::ReqMovement_byPose_FIx_Orientation(geometry_msgs::Pose pose_req)
//type 1 with normal execution, type 2 for last joint preference
{
    ros::Duration tiempo_traj(0.0);
    //CheckandFixPoseRequest(pose_req);
    bool fk = kinematic_states_[0]->setFromIK(joint_model_groups_[0], pose_req, 2, 0.025);

    if (fk)
    {
        std::vector<double> joints_result(6);
        control_msgs::FollowJointTrajectoryGoal goale;
        kinematic_states_[0]->copyJointGroupPositions(joint_model_groups_[0], joints_result);

        std::vector<double> joints_result_pos(6);
        geometry_msgs::Pose TPoseTemp = pose_req;
        TPoseTemp.orientation.w = 0.0;
        TPoseTemp.orientation.x = 0.0;
        TPoseTemp.orientation.y = 1.0;
        TPoseTemp.orientation.z = 0.0;
        bool found_ikO = kinematic_states_[0]->setFromIK(joint_model_groups_[0], TPoseTemp, 1, 0.05);
        if (found_ikO)
        {
            kinematic_states_[0]->copyJointGroupPositions(joint_model_groups_[0], joints_result_pos); //de jv saco posicion de joints 0 a 5
            joints_result[0] = joints_result_pos[0];
            joints_result[1] = joints_result_pos[1];
            joints_result[2] = joints_result_pos[2];
            joints_result[3] = joints_result_pos[3];
            joints_result[4] = joints_result_pos[4];
            //std::cout<<"last joint todo: "<<jv[5]<<std::endl;
            //std::cout<<"last joint simple: "<<jvT[5]<<std::endl;
        }
        //std::cout<<"joints : "<<jv[0]<<" "<<jv[1]<<" "<<jv[2]<<" "<<jv[3]<<" "<<jv[4]<<" "<<jv[5]<<std::endl;
        goale = arm.makeArmUpTrajectory(joints_result);

        //int numpoints6 = goale.trajectory.points.size()-1;//escoger el punto final ya que empieza desde 0
        // tiempo_traj = goale.trajectory.points[1].time_from_start; //tiempo del punto final
        //delay_time = std::chrono::microseconds(int(tiempo_traj.toSec() * 1000000));
        //usleep(1000000*(delay_time));
        arm.startTrajectory(goale); //Inicio de trayectoria en GAZEBO
    }
    else
    {

        PrintPose("NO solution for pose request!", pose_req);
        //SendInitialPose();
        // Print("Initial Position sent!");
    }
    return fk;
}
bool Ed_Pmov::Check_Collision_TypeB(std::vector<double> Position)
//type - 1 para solo posicion y 2 para posicion y orientacion juntas
{
    geometry_msgs::Pose CheckPose;
    CheckPose.position.x = Position[0];
    CheckPose.position.y = Position[1];
    CheckPose.position.z = Position[2] - 0.05;

    CheckPose.orientation.w = Position[3];
    CheckPose.orientation.x = Position[4];
    CheckPose.orientation.y = Position[5];
    CheckPose.orientation.z = Position[6];
    //tic();
    //  process_mtx.lock();
    bool found_ik = kinematic_states_[0]->setFromIK(joint_model_groups_[0], CheckPose, 1, 0.0008);
    //process_mtx.unlock();
    //Print("check pose",found_ik,toc().count());
    return found_ik;
}
inline bool Ed_Pmov::Check_Collision_Indx(std::vector<double> Position, int index)
//type - 1 para solo posicion y 2 para posicion y orientacion juntas
{
    geometry_msgs::Pose CheckPose;
    CheckPose.position.x = Position[0];
    CheckPose.position.y = Position[1];
    CheckPose.position.z = Position[2] -0.05;

    CheckPose.orientation.w = 0.0;
    CheckPose.orientation.x = 0.0;
    CheckPose.orientation.y = 1;
    CheckPose.orientation.z = 0.0;
    bool found_ik = kinematic_states_[index]->setFromIK(joint_model_groups_[index], CheckPose, 1, 0.006);
    return found_ik;
}

void Ed_Pmov::FeedCollisionCheck_Queue(VectorDbl eeff_point, VectorDbl eeff_point_traslation, int region)
{

    PositionResults position;
    position.Position = eeff_point;
    position.Position_Only_T = eeff_point_traslation;
    position.region = region;
    //Print("feeding values",eeff_point.size(),eeff_point_traslation.size(),region);
    eeff_positions_queue.push(position);
    return;
}
void Ed_Pmov::ComputeThread_CollisionCheck(int index)
{
    computing_thread_.push_back(std::thread([&]() {
        int index_th = index_ks;
        Print("THREAD CREATION", index_th);
        PositionResults eeff_position;
        PositionResults position_results_t;
        while (true)
        {
            bool input_state = eeff_positions_queue.pop(eeff_position);
            if (input_state)
            {
                //Print("Calculating", index_th);
                bool state = Check_Collision_Indx(eeff_position.Position, index_th);
                // Print("result to be created",index_th);
                position_results_t.Position = eeff_position.Position;
                position_results_t.Position_Only_T = eeff_position.Position_Only_T;
                position_results_t.region = eeff_position.region;
                std::pair<bool, PositionResults> result_N = std::make_pair(state, position_results_t);
                eeff_positions_results.push(result_N);
                // Print("result stored",index_th);
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }));
    std::this_thread::sleep_for(std::chrono::milliseconds(75));
    return;
}

std::pair<bool, PositionResults> Ed_Pmov::RetrieveResults()
{
    bool result_retrieved = false;
    std::pair<bool, PositionResults> result;

    result.first = false;
    while (!result_retrieved)
    {
        bool state = eeff_positions_results.pop(result);
        if (state)
        {
            result_retrieved = true;
            return result;
        }
        else
        {
            result_retrieved = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    return result;
}


geometry_msgs::Pose Ed_Pmov::getCurrentPose()
{
    currentPose = group.getCurrentPose().pose;
    auto currentPoseT = currentPose;
    return currentPoseT;
}

void Ed_Pmov::PrintCurrentPose(std::string workspace)
{
    currentPose = group.getCurrentPose().pose;
    Print(workspace + ", Crrnt Position x,y,z: ", currentPose.position.x, currentPose.position.y, currentPose.position.z);
    Print(workspace + ", Crrnt Orientation ow,ox,oy,oz: ", currentPose.orientation.w, currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z);
    return;
}
void Ed_Pmov::PrintPose(std::string workspace, geometry_msgs::Pose req_pose)
{
    Print(workspace + ", Position x,y,z", req_pose.position.x, req_pose.position.y, req_pose.position.z);
    Print(workspace + ", Orientation ow, ox, oy, oz", req_pose.orientation.w, req_pose.orientation.x, req_pose.orientation.y, req_pose.orientation.z);
    //std::cout<<workspace<<", Position x: "<<req_pose.position.x<<" y: "<<req_pose.position.y<<" z: "<<req_pose.position.z<< std::endl;
    //std::cout<<workspace<<", Orientation ow: "<<req_pose.orientation.w<<" ox: "<<req_pose.orientation.x<<" oy: "<<req_pose.orientation.y<<" oz: "<<req_pose.orientation.z<< std::endl;
    return;
}

void Ed_Pmov::Sleep(std::chrono::microseconds elapsed_time)
{
    auto time_for_sleep = delay_time - elapsed_time;
    int time_for_sleepD = time_for_sleep.count();
    if (time_for_sleepD <= 0.0)
        time_for_sleepD = 0.0;
    usleep(1 * (time_for_sleepD + 100));
}
void Ed_Pmov::tic()
{
    tic_clock_time = std::chrono::high_resolution_clock::now();
}
std::chrono::microseconds Ed_Pmov::toc()
{
    auto toc_clock = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(toc_clock - tic_clock_time);
    return elapsed_time;
}
std::chrono::microseconds Ed_Pmov::toc(std::chrono::time_point<std::chrono::high_resolution_clock> tic_clock)
{
    auto toc_clock = std::chrono::high_resolution_clock::now();
    auto elapsed_c = std::chrono::duration_cast<std::chrono::microseconds>(toc_clock - tic_clock);
    return elapsed_c;
}

#endif
