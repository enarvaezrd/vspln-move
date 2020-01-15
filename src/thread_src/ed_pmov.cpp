#ifndef ED_PMOV_CODE
#define ED_PMOV_CODE

#include "ed_pmov.hpp"

void Arm_Joint_State::Mx_Joints_State_Handler(const sensor_msgs::JointState &joints_stateP)
{
    arm_measurements_mtx.lock();
    Mx_Joints_State = joints_stateP;
    arm_measurements_mtx.unlock();
}
void Arm_Joint_State::Pro_Joints_State_Handler(const sensor_msgs::JointState &joints_state)
{
    arm_measurements_mtx.lock();
    Pro_Joints_State = joints_state;
    arm_measurements_mtx.unlock();
}

std::vector<double> Arm_Joint_State::GetCurrentArmJoints()
{
    arm_measurements_mtx.lock();
    int joint_count = 0;
    int pro_joints = 0;
    int mx_joints = 0;
    for (auto pro_joint : Pro_Joints_State.position)
    {
        pro_joints++;
        //  cout << "--- pro angles received: " << pro_joint << endl;
        arm_joints_state[joint_count] = pro_joint;
        joint_count++;
    }
    for (auto mx_joint : Mx_Joints_State.position)
    {
        mx_joints++;
        // cout << "---  mx angles received: " << mx_joint << endl;
        arm_joints_state[joint_count] = mx_joint;
        joint_count++;
    }

    arm_measurements_mtx.unlock();

    if (joint_count != 6)
        std::cerr << "Error ed_pmov.cpp: joints are not complete, Number of Joints: " << joint_count <<", mx: "<<mx_joints<<", pro: "<<pro_joints<< "\n";

    return arm_joints_state;
}

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
        state = false;
    }
    return state;
}

bool Ed_Pmov::ReqMovement_byPose_Moveit(geometry_msgs::Pose pose_req)
{
    bool state = false;
    group.setPoseTarget(pose_req);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        control_msgs::FollowJointTrajectoryGoal goaled;
        goaled.trajectory = my_plan.trajectory_.joint_trajectory;

        arm.startTrajectory(goaled); //Inicio de trayectoria en GAZEBO
        state = true;
    }
    else
    {
        Print("NO solution for pose request with moveit!");
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
        //jv[1] *= -1;
        //jv[5] *= -1;
        //jv[4] *= -1;
        goale = arm.makeArmUpTrajectory(jv);
        // tiempo_traj = goale.trajectory.points[1].time_from_start; //tiempo del punto final
        //delay_time = std::chrono::microseconds(int(tiempo_traj.toSec() * 1000000));
        //usleep(1000000*(delay_time));

        // Print("Joints", jv[0], jv[1], jv[2], jv[3], jv[4], jv[5]);
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

    bool fk = kinematic_states_[0]->setFromIK(joint_model_groups_[0], pose_req, 2, 0.01);

    if (fk)
    {
        std::vector<double> joints_result(6);
        control_msgs::FollowJointTrajectoryGoal goale;
        kinematic_states_[0]->copyJointGroupPositions(joint_model_groups_[0], joints_result);

        std::vector<double> joints_result_pos(6);
        geometry_msgs::Pose TPoseTemp = pose_req;
        TPoseTemp.orientation.w = 0.0;
        TPoseTemp.orientation.x = 1.0;
        TPoseTemp.orientation.y = 0.0;

        TPoseTemp.orientation.z = 0.0;
        bool found_ikO = kinematic_states_[0]->setFromIK(joint_model_groups_[0], TPoseTemp, 2, 0.01);
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
        //joints_result[1] *= -1;
        //std::cout<<"joints : "<<jv[0]<<" "<<jv[1]<<" "<<jv[2]<<" "<<jv[3]<<" "<<jv[4]<<" "<<jv[5]<<std::endl;
        goale = arm.makeArmUpTrajectory(joints_result);

        //int numpoints6 = goale.trajectory.points.size()-1;//escoger el punto final ya que empieza desde 0
        tiempo_traj = goale.trajectory.points[1].time_from_start; //tiempo del punto final
        delay_time = std::chrono::microseconds(int(tiempo_traj.toSec() * 1000000));
        int delay_time_usecs = delay_time.count();
        if (delay_time_usecs > 50000)
        {
            // Print("Delay time", delay_time_usecs);
            delay_time_usecs = 50000;
        }
        //usleep(delay_time_usecs);
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

control_msgs::FollowJointTrajectoryGoal Ed_Pmov::Req_Joints_byPose_FIx_Orientation(geometry_msgs::Pose pose_req)
{
    ros::Duration tiempo_traj(0.0);
    //CheckandFixPoseRequest(pose_req);

    control_msgs::FollowJointTrajectoryGoal goale;
    bool fk = kinematic_states_[0]->setFromIK(joint_model_groups_[0], pose_req, 2, 0.01);

    if (fk)
    {
        std::vector<double> joints_result(6);
        kinematic_states_[0]->copyJointGroupPositions(joint_model_groups_[0], joints_result);

        std::vector<double> joints_result_pos(6);
        geometry_msgs::Pose TPoseTemp = pose_req;
        TPoseTemp.orientation.w = 0.0;
        TPoseTemp.orientation.x = 1.0;
        TPoseTemp.orientation.y = 0.0;
        TPoseTemp.orientation.z = 0.0;
        bool found_ikO = kinematic_states_[0]->setFromIK(joint_model_groups_[0], TPoseTemp, 2, 0.01);
        if (found_ikO)
        {
            kinematic_states_[0]->copyJointGroupPositions(joint_model_groups_[0], joints_result_pos); //de jv saco posicion de joints 0 a 5
            joints_result[0] = joints_result_pos[0];
            joints_result[1] = joints_result_pos[1];
            joints_result[2] = joints_result_pos[2];
            joints_result[3] = joints_result_pos[3];
            joints_result[4] = joints_result_pos[4];
            //joints_result[5] = joints_result_pos[5];
            //std::cout<<"last joint todo: "<<jv[5]<<std::endl;
            //std::cout<<"last joint simple: "<<jvT[5]<<std::endl;
        }

        ////  joints_result[1] *= -1;
        //  joints_result[4] *= -1;
        //  joints_result[5] *= -1;
        // Print("joints", joints_result[0], joints_result[1], joints_result[2], joints_result[3], joints_result[4], joints_result[5]);
        goale = arm.makeArmUpTrajectory(joints_result);

        //int numpoints6 = goale.trajectory.points.size()-1;//escoger el punto final ya que empieza desde 0
        tiempo_traj = goale.trajectory.points[0].time_from_start; //tiempo del punto final
        delay_time = std::chrono::microseconds(int(tiempo_traj.toSec() * 1000000));

        if (delay_time.count() > 100000)
        {
            // Print("Delay time", delay_time.count());
            delay_time = std::chrono::microseconds(100000);
        }
    }
    else
    {
        // PrintPose("NO solution for pose request!", pose_req);
    }
    return goale;
}

bool Ed_Pmov::Request_Movement_byJointsTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
{
    if (goal.trajectory.points.size() > 0)
    {
        arm.startTrajectory(goal); //Inicio de trayectoria en GAZEBO

        // Print("delay time ", delay_time.count());
       // std::this_thread::sleep_for(delay_time);
        return true;
    }
    else
    {
        return false;
    }
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
    // tic();
    // process_mtx.lock();
    bool found_ik = kinematic_states_[0]->setFromIK(joint_model_groups_[0], CheckPose, 1, 0.0008);
    //process_mtx.unlock();
    //Print("check pose",found_ik,toc().count());
    return found_ik;
}
inline bool Ed_Pmov::Check_Collision_Indx(std::vector<double> Position, int index_f)
//type - 1 para solo posicion y 2 para posicion y orientacion juntas
{
    geometry_msgs::Pose CheckPose;
    CheckPose.position.x = Position[0];
    CheckPose.position.y = Position[1];
    CheckPose.position.z = Position[2];

    CheckPose.orientation.w = 0.0;
    CheckPose.orientation.x = 0.0;
    CheckPose.orientation.y = 1.0;
    CheckPose.orientation.z = 0.0;
    bool found_ik = kinematic_states_[index_f]->setFromIK(joint_model_groups_[index_f], CheckPose, 1, 0.0004);
    return found_ik;
}

void Ed_Pmov::FeedCollisionCheck_Queue(VectorDbl eeff_point, VectorDbl eeff_point_traslation, int region)
{
    PositionResults position;
    position.Position = eeff_point;
    position.Position_Only_T = eeff_point_traslation;
    position.region = region;
    //Print("feeding values",eeff_point.size(),eeff_point_traslation.size(),region);

    eeff_positions_input_queue.push(position);
    return;
}
void Ed_Pmov::ComputeThread_CollisionCheck(int index)
{
    computing_thread_.push_back(std::thread([&]() {
        int index_th = index;
        Print("THREAD CREATION", index_th);
        PositionResults eeff_position;
        PositionResults position_results_t;
        while (true)
        {
            bool input_state = eeff_positions_input_queue.pop(eeff_position);
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
                std::this_thread::sleep_for(std::chrono::microseconds(1));
            }
        }
        Print("END OF THREAD", index_th);
    }));
    std::this_thread::sleep_for(std::chrono::milliseconds(35));
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
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    }
    return result;
}

geometry_msgs::Pose Ed_Pmov::getCurrentPose()
{
    Eigen::Affine3d end_effector_state;
    if (load_joint_states_sub)
    { //real
        currentJoints = getCurrentJoints();
        //PrintCurrentJoints("==");
        auto joints_fake = group.getCurrentJointValues();
        //  Print("JOINTS FAKE ", joints_fake[0], joints_fake[1], joints_fake[2], joints_fake[3], joints_fake[4], joints_fake[5]);
        //currentJoints[1] += 2 * PI;
        kinematic_states_[0]->setJointGroupPositions(joint_model_groups_[0], currentJoints);
        end_effector_state = kinematic_states_[0]->getGlobalLinkTransform("link_motor_mx282");
        tf::poseEigenToMsg(end_effector_state, currentPose);
    }
    else
    { //simm

        currentPose = group.getCurrentPose("link_motor_mx282").pose;
        //cout<<"CURRENTORIG Position: x: "<<currentPose.position.x<<", y: "<<currentPose.position.y<<", z: "<<currentPose.position.z<<"\n";
        //cout<<"CURRENTORIG Rotation: x: "<<currentPose.orientation.x<<", y: "<<currentPose.orientation.y<<", z: "<<currentPose.orientation.z<<", w: "<<currentPose.orientation.w<<"\n";
    }

    return currentPose;
}

std::vector<double> Ed_Pmov::getCurrentJoints()
{
    if (load_joint_states_sub)
    { //real
        currentJoints = ArmJointsState->GetCurrentArmJoints();
        //Print(" Current Joints: ", currentJoints[0], currentJoints[1], currentJoints[2], currentJoints[3], currentJoints[4], currentJoints[5]);

        // currentJoints[0] +=  PI/2.0;
    }
    else
    { //simm
        currentJoints = group.getCurrentJointValues();
        // cout<<"JOINT : "<<currentJoints[1]<<endl;
        kinematic_states_[0]->setJointGroupPositions(joint_model_groups_[0], currentJoints);
        Eigen::Affine3d end_effector_state = kinematic_states_[0]->getGlobalLinkTransform("link_motor_mx282");

        Eigen::Quaterniond q_pos(end_effector_state.rotation());
        auto tr = end_effector_state.translation();

        // ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
        //ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
        //cout<<"Rotation: x: "<<q_pos.x()<<", y: "<<q_pos.y()<<", z: "<<q_pos.z()<<", w: "<<q_pos.w()<<"\n";
    }

    auto currentJointsT = currentJoints;
    return currentJointsT;
}
void Ed_Pmov::PrintCurrentPose(std::string workspace = ">>")
{
    currentPose = getCurrentPose();
    Print(workspace + ", Crrnt Position x,y,z: ", currentPose.position.x, currentPose.position.y, currentPose.position.z);
    Print(workspace + ", Crrnt Orientation ow,ox,oy,oz: ", currentPose.orientation.w, currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z);
    return;
}
void Ed_Pmov::PrintCurrentJoints(std::string workspace = ">>")
{
    currentJoints = getCurrentJoints();
    if (currentJoints.size() == 6)
        Print(workspace + ", Current Joints: ", currentJoints[0], currentJoints[1], currentJoints[2], currentJoints[3], currentJoints[4], currentJoints[5]);
    else
        Print("!! Number of joints is not corect", currentJoints.size());
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
