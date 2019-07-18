#ifndef ED_PMOV_CODE
#define ED_PMOV_CODE

#include "ed_pmov.hpp"
#define PI 3.14159265
int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_program");
    ros::NodeHandle nodle_handle;
    ros::Rate loop_rate(20);
    /*
    //type - 1 para solo posicion y 2 para posicion y orientacion juntas
    robot_model_loader::RobotModelLoader robot_model_loader("robot1/robot_description");
    robot_model_loader::RobotModelLoader robot_model_loader1("robot1/robot_description");
    robot_model_loader::RobotModelLoader robot_model_loader2("robot1/robot_description");
    robot_model::RobotModelPtr kinematic_model;
    robot_model::RobotModelPtr kinematic_model1;
    robot_model::RobotModelPtr kinematic_model2;
    kinematic_model = robot_model_loader.getModel();
    kinematic_model1 = robot_model_loader1.getModel();
    kinematic_model2 = robot_model_loader2.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    robot_state::RobotStatePtr kinematic_state1(new robot_state::RobotState(kinematic_model1));
    robot_state::RobotStatePtr kinematic_state2(new robot_state::RobotState(kinematic_model2));
    kinematic_state->setToDefaultValues();
    kinematic_state1->setToDefaultValues();
    kinematic_state2->setToDefaultValues();
    const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("pro_arm_aux");
    const robot_state::JointModelGroup *joint_model_group1 = kinematic_model->getJointModelGroup("pro_arm_aux1");
    const robot_state::JointModelGroup *joint_model_group2 = kinematic_model->getJointModelGroup("pro_arm_aux2");
*/
    geometry_msgs::Pose CheckPose;
    CheckPose.position.x = 0.18;
    CheckPose.position.y = 0.28;
    CheckPose.position.z = 0.72;

    CheckPose.orientation.w = 0.0;
    CheckPose.orientation.x = 0.0;
    CheckPose.orientation.y = 1;
    CheckPose.orientation.z = 0.0;
    geometry_msgs::Pose CheckPoseA = CheckPose;
    //tic();
    // process_mtx.lock();
    // Print("calculation",index);
    int thread_index = 0;
    std::queue<geometry_msgs::Pose> results;

    thread_index++;
    int th_indx = thread_index;
    float i = 0;
    int factor = 1;
    IK_Solver ik;

    std::mutex positions_mtx, results_mtx;
    std::queue<geometry_msgs::Pose> eeff_positions_queue;
    std::queue<std::pair<bool, geometry_msgs::Pose>> eeff_positions_results;
    int index = 0;
    std::thread th;
    ik.ComputeThread_CollisionCheck(0);
    ik.ComputeThread_CollisionCheck(1);
    ik.ComputeThread_CollisionCheck(2);
    /*
    runThread(ik.kinematic_state, ik.joint_model_group, ik.positions_mtx, ik.results_mtx, ik.eeff_positions_queue,
              ik.eeff_positions_results, th, 0);

    std::this_thread::sleep_for(std::chrono::milliseconds(105));
    int index2 = 1;
    std::thread th1;
    runThread(ik.kinematic_state1, ik.joint_model_group1, ik.positions_mtx, ik.results_mtx, ik.eeff_positions_queue,
              ik.eeff_positions_results, th1, 1);
*/
    std::this_thread::sleep_for(std::chrono::milliseconds(105));

    while (true)
    {
        CheckPoseA.position.x = 0.2 * sin(i * PI);
        i = i + (factor * 0.001);
        if (i >= 2 || i <= -2)
            factor *= -1;

        //ik.eeff_positions_queue.push(CheckPoseA);
        ik.FeedCollisionCheck_Queue(CheckPoseA);

        std::pair<bool, geometry_msgs::Pose> result;
           
            result = ik.RetrieveResults();
            bool found_ik = result.first;
            if (found_ik)
            {

                results.push(result.second);
            } 
            cout << "results size stored outside: " << results.size() << endl;
    

        if (results.size() > 100)
        {
            results.pop();
        }
        // cout << found_ik << "found, size: " << results.size() << endl;
         //   std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}

#endif