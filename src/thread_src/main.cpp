#ifndef MAIN_APP
#define MAIN_APP
#define REAL_ROBOTS
#include "uav_arm_tools.hpp"

using namespace std;
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
    ros::Rate loop_rate(30);
    Printer Print;
    std::vector<double> joint_valuesT(6);
    std::mutex m;
    joint_valuesT[0] = -PI / 2 + 0.1;
    joint_valuesT[1] = 2.0; //PI/2;
    joint_valuesT[2] = 2.0; //PI/2;
    joint_valuesT[3] = 0.0; // PI/2;
    joint_valuesT[4] = 0.0;
    joint_valuesT[5] = 0.0; // PI/2;
    rrt_planif::RRT RRT_model(image_size, d_prv, d_pr_m, prof_expl, scale, num_nodes_per_region, load_joints_state_sub);
    RRT_model.ArmModel.SendMovement_byJointsValues(joint_valuesT);

    PredNs::Prediction Predict_B(image_size, d_prv, d_pr_m, prof_expl, Map_size, scale, rrt_extension, rad_int, rad_ext);
    ObstacleMapGen ObstacleMap(Map_size, scale, image_size, rad_int, rad_ext, laser_topic);
    RobotCommands Robot_Commands(odom_str, UAV_position_x, UAV_position_y);

    ua_ns::uav_arm_tools UavArm_tools(rad_int, rad_ext, armMinAltitude, controller_topic, Docking_Alt_Lim_, DockingFactor);

    RRT_model.ArmModel.PrintModelInfo();

    RRT_model.ArmModel.PrintCurrentPose("====>STARTING POSE ::::");
    float alturap = armMinAltitude; //0.21

    geometry_msgs::Pose target_pose = RRT_model.ArmModel.getCurrentPose();

    Angles IAngleMark = UavArm_tools.ConvPosetoAngles(target_pose);
    Print("PET ANGLES yaw,roll,pitch", IAngleMark.yaw, IAngleMark.roll, IAngleMark.pitch);

    target_pose.orientation.w = 0.0; //0.1
    target_pose.orientation.x = 0.0; //0.10
    target_pose.orientation.y = 1.0;
    target_pose.orientation.z = 0.0;

    target_pose.position.x = UAV_position_x; //0.1
    target_pose.position.y = UAV_position_y;
    target_pose.position.z = alturap;

    UavArm_tools.setArmPoseReq(target_pose);
    //target_pose = UavArm_tools.getArmPoseReq();
    bool reqState = RRT_model.ArmModel.ReqMovement_byPose(target_pose);
    sleep(2.0);
    /* target_pose.position.x = -0.17; //0.1
    target_pose.position.y = 0.2;
     reqState = RRT_model.ArmModel.ReqMovement_byPose(target_pose);
    sleep(4.0);
    target_pose.position.x = -0.17; //0.1
    target_pose.position.y = -0.2;
     reqState = RRT_model.ArmModel.ReqMovement_byPose(target_pose);
    sleep(8.0);
    target_pose.position.x = 0.17; //0.1
    target_pose.position.y = -0.2;
     reqState = RRT_model.ArmModel.ReqMovement_byPose(target_pose);
    sleep(5.0);
*/
    geometry_msgs::Pose target_posea = RRT_model.ArmModel.getCurrentPose();

    IAngleMark = UavArm_tools.ConvPosetoAngles(target_posea);
    Print("PETA ANGLES yaw,roll,pitch", IAngleMark.yaw, IAngleMark.roll, IAngleMark.pitch);

    //if (reqState==true)
    UavArm_tools.UpdateArmCurrentPose(RRT_model.ArmModel.getCurrentPose());

    int TrackingState = 0;
    int ThreadStep = 0;
    geometry_msgs::Pose CurrentRequest, CurrentRequest_Thread = target_posea;
    std::mutex openCV_mutex;

    //  namedWindow(window_name_B, cv::WINDOW_NORMAL);

    //namedWindow(window_name, cv::WINDOW_NORMAL);
    //cv::resizeWindow(window_name, image_size, image_size);
    auto rrt_threadB = std::thread([&]() {
        string window_name_B = "Prediction + RRT Image";
        ros::Rate loop_rate_thread(30);
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        bool sequence_loop_th = false;
        //sleep(6.0);
        auto clB = std::chrono::high_resolution_clock::now();
        while (ros::ok())
        {
            while (sequence_loop_th && ros::ok())
            {
                //Ed_Pmov ARMMdl=RRT_modelA.Get_ArmModel();
                //RRT_modelB.Load_ArmModel(ARMMdl);
                RRT_model.Load_Adv(Predict_B.Get_Adv());
                RRT_model.Load_TR(Predict_B.Get_TR());
                RRT_model.Load_TRbr(Predict_B.Get_TRbr());
                RRT_model.Load_UAV_Velocity(Predict_B.Get_UAV_Velocity());
                RRT_model.ResetImagePtraj();
                if (RRT_model.get_TR_Size() > 0 && !Predict_B.get_Stop_RRT_Flag()) //trajA.xval.size()>0 &&
                {
                    try
                    {
                        RRT_model.RRT_SequenceB();
                    }
                    catch (const std::exception &e)
                    {
                        std::cerr << "RRT exeption: " << e.what() << '\n';
                    }

                    Predict_B.Load_Nodes(RRT_model.GetNodes(), RRT_model.GetVicinity());
                }

                sequence_loop_th = RRT_model.getLoopState();

                clB = std::chrono::high_resolution_clock::now();
                // RRT_modelB.ArmModel.getDelayTime();
                //std::this_thread::sleep_for(std::chrono::milliseconds(80));

                ros::spinOnce();
                loop_rate_thread.sleep();
            }
            while (!sequence_loop_th && ros::ok())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                sequence_loop_th = RRT_model.getLoopState();
                //ros::spinOnce();
                loop_rate_thread.sleep();
            }
        }
    });

    std::mutex arm_control_msg_mtx;
    geometry_msgs::Pose ArmRequest = CurrentRequest_Thread;
    bool fresh_request = false;
    auto rrt_threadA = std::thread([&]() {
        string window_name = "Prediction + RRT - EEFF Path Calculation";

#ifdef OPENCV_DRAW
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);
#endif
        //std::unique_lock<std::mutex> lck(mtx_main);
        auto clA = std::chrono::high_resolution_clock::now();
        geometry_msgs::Pose CurrentArmPose;
        int NoVisualContact_count = 0;
        cv::Rect myROI(round(1 * image_size / 5), round(1 * image_size / 4) + 50, round(3 * image_size / 5), round(2 * image_size / 4) - 10);

        while (ros::ok())
        {

            Predict_B.ClearImage_Ptraj();
            Predict_B.Load_Map(ObstacleMap.get_Map(), ObstacleMap.get_Obs_Points(), ObstacleMap.get_Obs_Points_Thick()); //Load Obstacle Map
            //Predict_B.Draw_Map();
            ObstacleMap.Load_UGV_state(Robot_Commands.ugv_state);
            //CurrentArmPose = RRT_model.ArmModel.getCurrentPose(); //desde el brazo
            geometry_msgs::Pose NextArmRequest = CurrentRequest_Thread;

            auto timeStep = std::chrono::high_resolution_clock::now();
            CurrentArmPose = RRT_model.ArmModel.getCurrentPose();
            // auto joints1 = RRT_model.ArmModel.getCurrentJoints();
            UavArm_tools.UpdateArmCurrentPose(CurrentArmPose);
            //RRT_model.ArmModel.tic();

            if (UavArm_tools.getTrackingState() == 1 || UavArm_tools.getTrackingState() == 20)
            { //Tracking OK
                // Robot_Commands.Calculate_and_Send_Commands(LocalUAVPose);
                TrackingState++;
                //Publicar aqui la pose absoluta del UAV para su control, tracking system
                if (TrackingState > 50)
                    TrackingState = 50;
            }
            else
            {
                NoVisualContact_count++;
                Robot_Commands.Send_Empty_Commands();
                if (TrackingState == 0)
                {
                    RRT_model.loop_end();
                }

                RRT_model.reset_nodes_reordered();
                UavArm_tools.PIDReset(); //reset, set zeros to errors and integrals
                TrackingState = 0;
                UavArm_tools.ArmPoseReq_decreaseAlt(0.02); //modifies the arm request to lower the end effector
                UavArm_tools.counter = 0;
                if (NoVisualContact_count > 10)
                {
                    NextArmRequest.position.x = UAV_position_x;
                    NextArmRequest.position.y = UAV_position_y;
                    NextArmRequest.position.z = alturap;
                    CurrentRequest_Thread = NextArmRequest;
                    NoVisualContact_count = 0;
                }
                //
                //NextArmRequest = Predict_B.NoTarget_Sequence(target_posea);
            }
            if (TrackingState > 0 && UavArm_tools.Controller_Commands.tracking_process)
            {

                UavArm_tools.counter_addOne(); // para envio espaciado de orientaciones
                                               // m.lock();
                //UavArm_tools.uavPose_to_ArmPoseReq_full(); // for rrt process
                CurrentArmPose = RRT_model.ArmModel.getCurrentPose();
                UavArm_tools.UpdateArmCurrentPose(CurrentArmPose);
                UavArm_tools.uavPose_to_ArmPoseReq_arm(); // for visual servoing
                //m.unlock();
                //UavArm_tools.setAltitudeRequest(UavArm_tools.getMinArmAltitude()); //change ArmPoseReq z value
                UavArm_tools.CalculateDockingAltitude();
                float trust_factor = 0.5;
                if (UavArm_tools.Controller_Commands.docking_process)
                {
                    trust_factor = 0.0;
                }
                else
                {
                    trust_factor = 0.5;
                }

                UavArm_tools.setAltitudeRequest(UavArm_tools.getEEFFAltitude());
                //CurrentRequest_Thread = UavArm_tools.getArmPoseReqFull();// with mutex
                CurrentRequest_Thread = UavArm_tools.getArmPoseReq(); // with mutex

                Predict_B.Load_UGV_State(Robot_Commands.ugv_state); //sequential
                Predict_B.Planif_SequenceA(CurrentRequest_Thread, CurrentArmPose);
                Predict_B.Charge_Nodes();
                RRT_model.loop_start();
                Predict_B.RRT_Path_Generation();

                NextArmRequest = Predict_B.Selection_Function(trust_factor);
            }
            arm_control_msg_mtx.lock();
            fresh_request = true;
            ArmRequest = NextArmRequest;
            arm_control_msg_mtx.unlock();
            //CurrentRequest = UavArm_tools.getArmPoseReq();
            // RRT_model.ArmModel.PrintPose("Req", NextArmRequest);
            //std::chrono::microseconds elapsed_time = RRT_model.ArmModel.toc();

            // RRT_model.loop_end();
            // RRT_model.ArmModel.Sleep(elapsed_time); //sleep the resulting time
            // RRT_model.loop_start();

            //RRT_model.ArmModel.ReqMovement_byPose_Moveit(NextArmRequest);

            //RRT_model.ArmModel.ReqMovement_byPose_FIx_Orientation(NextArmRequest);

#ifdef OPENCV_DRAW
            // cv::Mat imageA = Predict_B.getImage_Ptraj();

            cv::Mat croppedImage = Predict_B.getImage_Ptraj()(myROI);
            //   openCV_mutex.lock();

            cv::imshow(window_name, croppedImage);
            cv::waitKey(1);
#endif
            // openCV_mutex.unlock();
            //std::this_thread::sleep_for(std::chrono::milliseconds(35));
            // loop_rate.sleep();
            // cout << "=====> SEQUENCE A TIME: " << RRT_model.ArmModel.toc(clA).count() << endl;
            Print("===--->SEQUENCE A TIME ", RRT_model.ArmModel.toc(clA).count());
            clA = std::chrono::high_resolution_clock::now();
        }
    });

    //========================CONTROL THREAD===========================================================

    auto threadControl = std::thread([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        ros::Rate loop_rateControl(30);
        double non_tracking_height_corr = 0.2;
        double y_correction = 0.0;
        geometry_msgs::Pose OldLocalUAVPose = target_posea;
        deque<double> localUAV_Vel;

        while (ros::ok())
        {
            if (UavArm_tools.Controller_Commands.docking_process && UavArm_tools.Controller_Commands.tracking_process)
            {
                non_tracking_height_corr = -0.16;
            }

            if (UavArm_tools.getTrackingState() == 1 || UavArm_tools.getTrackingState() == 20)
            {
                geometry_msgs::Pose LocalUAVPose = UavArm_tools.Calc_LocalUAVPose();

                //if (UavArm_tools.Controller_Commands.tracking_process)
                {
                    Robot_Commands.Calculate_and_Send_Commands(LocalUAVPose, non_tracking_height_corr, y_correction);
                }
                y_correction = 0.0;

                localUAV_Vel.push_back(sqrt(((OldLocalUAVPose.position.x - LocalUAVPose.position.x) * (OldLocalUAVPose.position.x - LocalUAVPose.position.x)) +
                                            ((OldLocalUAVPose.position.y - LocalUAVPose.position.y) * (OldLocalUAVPose.position.y - LocalUAVPose.position.y))));

                double accumm_UAV_vel = 0.0;
                int uav_vel_size = localUAV_Vel.size();
                for (int i = 0; i < uav_vel_size; i++)
                {
                    accumm_UAV_vel += localUAV_Vel[i];
                }
                accumm_UAV_vel /= uav_vel_size;
                if (uav_vel_size > 5)
                {
                    localUAV_Vel.pop_front();
                }
                Predict_B.Set_UAV_Velocity(accumm_UAV_vel);

                OldLocalUAVPose = LocalUAVPose;
            }

            arm_control_msg_mtx.lock();
            geometry_msgs::Pose CurrentArmRequest = ArmRequest;
            bool fresh_request_local = fresh_request;
            fresh_request = false;
            arm_control_msg_mtx.unlock();

            //  y_correction = CurrentArmRequest.position.y;

            if (fresh_request_local && UavArm_tools.Controller_Commands.tracking_process) //tracking
            {

                non_tracking_height_corr = -0.1;
                control_msgs::FollowJointTrajectoryGoal ArmGoal = RRT_model.ArmModel.Req_Joints_byPose_FIx_Orientation(CurrentArmRequest);

                if (ArmGoal.trajectory.points.size() > 0)
                {
                    ArmGoal = RRT_model.SteerJoints(ArmGoal);
                    Print("JOINTS REQUEST", ArmGoal.trajectory.points[0].positions[0], ArmGoal.trajectory.points[0].positions[1]);

                    RRT_model.ArmModel.Request_Movement_byJointsTrajectory(ArmGoal);
                }

                //RRT_model.ArmModel.ReqMovement_byPose_FIx_Orientation(CurrentRequest_Thread); // CurrentRequest_Thread   NextArmRequest

                UavArm_tools.Load_PID_time(RRT_model.ArmModel.getDelayTime().count() / 1000000);
            }
            else
            {
                if (!UavArm_tools.Controller_Commands.tracking_process) //no tracking
                {
                    non_tracking_height_corr = 0.1;
                    auto ArmGoal = RRT_model.ArmModel.Req_Joints_byPose_FIx_Orientation(target_posea); //return to origin

                    if (ArmGoal.trajectory.points.size() > 0)
                    {
                        ArmGoal = RRT_model.SteerJoints(ArmGoal);
                        Print("JOINTS REQUEST", ArmGoal.trajectory.points[0].positions[0], ArmGoal.trajectory.points[0].positions[1]);
                        RRT_model.ArmModel.Request_Movement_byJointsTrajectory(ArmGoal);
                    }
                }
                else
                {
                    non_tracking_height_corr = -0.1;
                }
            }
            loop_rateControl.sleep();
        }
    });

    // if (rrt_thread.joinable()) rrt_thread.join();
    //if (main_thread.joinable()) main_thread.join();

    rrt_threadB.join();
    rrt_threadA.join();
    threadControl.join();
}
#endif
