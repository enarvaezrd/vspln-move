#ifndef MAIN_APP
#define MAIN_APP
#include "uav_arm_tools.hpp"

using namespace std;
int main(int argc, char **argv)
{
    //==============================CONFIGURATION PARAMETERS===========================

    int image_size = 600;

    int d_prv = 5;     // profundidad de datos previos disponibles para prediccion
    int d_pr_m = 3;    // datos previos a usar para calculo de mean values
    int prof_expl = 9; // Profundidad de exploracion  Esz=prof_f
    int Map_size = image_size;
    float max_dimm = 1.0;
    double rrt_extension = 0.06; //extension of each rrt step for regression 0.38 0.28
    int num_nodes_per_region = prof_expl + 1;
    double rad_int = 0.18; //0.245
    double rad_ext = 0.37; //0.41
    double armMinAltitude = 0.57;
    bool load_joints_state_sub, real_robots;
    std::string controller_topic = "/joy";
    double Docking_Alt_Lim_ = 0.87;
    double DockingFactor = 0.0625;
    map<int, ua_ns::Positions2D> marker_offsets;
    ua_ns::Positions2D marker1;
    marker1.x = {0};
    marker1.y = {0};
    ua_ns::Positions2D marker2;
    marker2.x = {0};
    marker2.y = {0};
    marker_offsets.insert(std::pair<int, ua_ns::Positions2D>(0, marker1));
    marker_offsets.insert(std::pair<int, ua_ns::Positions2D>(2, marker2));
#ifdef REAL_ROBOTS
    load_joints_state_sub = true;
    string odom_str = "/robot1/odom"; ///robot1/robotnik_base_control/odom  ;   /robot1/odom
    string laser_topic = "/scan";     ///robot1/front_laser/scan SIMM ;  /scan REAL
    real_robots = true;
#else
    string odom_str = "/robot1/odom";                ///robot1/robotnik_base_control/odom  ;   /robot1/odom
    string laser_topic = "/robot1/front_laser/scan"; ///robot1/front_laser/scan SIMM ;  /scan REAL
    load_joints_state_sub = false;
    real_robots = false;
#endif

    double UAV_position_x = -0.25;
    double UAV_position_y = 0.1;
    //==================================================================================================================

    ros::init(argc, argv, "arm_program");
    ros::NodeHandle nodle_handle;

    Printer Print;
    std::vector<double> joint_values_storage(6), joint_values_storage_transition(6);
    std::mutex m;
    joint_values_storage[0] = -PI / 2;
    joint_values_storage[1] = PI / 2 + 0.1;  //PI/2;
    joint_values_storage[2] = -PI / 2 - 0.3; //PI/2;
    joint_values_storage[3] = -PI / 2;       // PI/2;
    joint_values_storage[4] = 1.86;
    joint_values_storage[5] = -PI + 0.7; // PI/2;

    joint_values_storage_transition = joint_values_storage;
    joint_values_storage_transition[2] = 0.5;
    joint_values_storage_transition[5] = 0.0;

    sleep(1.0);
    rrt_planif::RRT RRT_model(image_size, d_prv, d_pr_m, prof_expl, max_dimm, num_nodes_per_region, load_joints_state_sub);
    RRT_model.ArmModel.SendMovement_byJointsValues(joint_values_storage);

    PredNs::Prediction Predict_B(image_size, d_prv, d_pr_m, prof_expl, Map_size, max_dimm, rrt_extension, rad_int, rad_ext);
    ObstacleMapGen ObstacleMap(Map_size, max_dimm, image_size, rad_int, rad_ext, laser_topic);
    RobotCommands Robot_Commands(odom_str, UAV_position_x, UAV_position_y, real_robots);

    ua_ns::uav_arm_tools UavArm_tools(rad_int, rad_ext, armMinAltitude, controller_topic, Docking_Alt_Lim_, DockingFactor, real_robots, marker_offsets);
    sleep(10.0);
    auto image_MarkerARM_Pos = cv::Mat(image_size, image_size, CV_8UC3, cv::Scalar(255, 255, 255));
    int scale = floor(image_size / (2.0 * max_dimm));

    RRT_model.ArmModel.PrintModelInfo();

    RRT_model.ArmModel.PrintCurrentPose("====>STARTING POSE ::::");
    float alturap = armMinAltitude; //0.21

    geometry_msgs::Pose target_pose = RRT_model.ArmModel.getCurrentPose();

    auto ArmGoal = RRT_model.ArmModel.Req_Joints_byPose_FIx_Orientation(target_pose); //return to origin

    if (ArmGoal.trajectory.points.size() > 0)
    {
        // ArmGoal = RRT_model.SteerJoints(ArmGoal);
        Print("JOINTS REQUEST STARTING", ArmGoal.trajectory.points[0].positions[0], ArmGoal.trajectory.points[0].positions[1],
              ArmGoal.trajectory.points[0].positions[2], ArmGoal.trajectory.points[0].positions[3],
              ArmGoal.trajectory.points[0].positions[4], ArmGoal.trajectory.points[0].positions[5]);
        RRT_model.ArmModel.Request_Movement_byJointsTrajectory(ArmGoal);
    }

    Angles IAngleMark = UavArm_tools.ConvPosetoAngles(target_pose);
    Print("PET ANGLES yaw,roll,pitch", IAngleMark.yaw, IAngleMark.roll, IAngleMark.pitch);
    sleep(1.0);
    target_pose.orientation.w = 0.0; //0.1
    target_pose.orientation.x = 0.0; //0.10
    target_pose.orientation.y = 1.0;
    target_pose.orientation.z = 0.0;

    target_pose.position.x = UAV_position_x; //0.1
    target_pose.position.y = UAV_position_y;
    target_pose.position.z = alturap;

    geometry_msgs::Pose target_pose_storage = target_pose;
    target_pose_storage.orientation.w = 0.0;    //0.1
    target_pose_storage.orientation.x = -0.707; //0.10
    target_pose_storage.orientation.y = -0.707;
    target_pose_storage.orientation.z = 0.0;

    target_pose_storage.position.x = 0.3; //0.1
    target_pose_storage.position.y = 0.0;
    target_pose_storage.position.z = alturap;

    UavArm_tools.setArmPoseReq(target_pose);
    //target_pose = UavArm_tools.getArmPoseReq();
    RRT_model.ArmModel.SendMovement_byJointsValues(joint_values_storage);
    sleep(3.0);
    bool reqState = RRT_model.ArmModel.ReqMovement_byPose(target_pose);
    sleep(1.0);
    RRT_model.ArmModel.PrintCurrentPose("====> POSE WITH POSE REQUEST ::::");
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
   
*/
    sleep(12.0);
    geometry_msgs::Pose target_posea = RRT_model.ArmModel.getCurrentPose();
    geometry_msgs::Pose target_pose_noTracking = target_pose;
    target_pose_noTracking.position.x = UAV_position_x; //0.1
    target_pose_noTracking.position.y = 0.0;

    RRT_model.ArmModel.PrintCurrentPose("====>POSE TRACKING TARGET ::::");
    UavArm_tools.setArmPoseReq(target_posea);

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
    ros::AsyncSpinner spinner(2);
    spinner.start();

    auto rrt_threadB = std::thread([&]() {
        string window_name_B = "Prediction + RRT Image";
        ros::Rate loop_rate_thread(70);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
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
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                sequence_loop_th = RRT_model.getLoopState();
                ros::spinOnce();
                loop_rate_thread.sleep();
            }
        }
    });

    std::mutex arm_control_msg_mtx;
    geometry_msgs::Pose ArmRequest = CurrentRequest_Thread;
    bool fresh_request = false;
    cv::Mat image_general = cv::Mat(image_size, image_size, CV_8UC3, cv::Scalar(255, 255, 255));
    geometry_msgs::Pose CurrentPose_General = CurrentRequest_Thread;
    int Accumulated_Tracking_State = 0;
    auto rrt_threadA = std::thread([&]() {
        string window_name = "Prediction + RRT - EEFF Path Calculation";

#ifdef OPENCV_DRAW
        // cv::namedWindow(window_name, cv::WINDOW_NORMAL);
#endif
        //std::unique_lock<std::mutex> lck(mtx_main);
        ros::Rate loop_rate(40);
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
            // auto joints1 = RRT_model.ArmModel.ReadCurrentJoints();
            UavArm_tools.UpdateArmCurrentPose(CurrentArmPose);

            //RRT_model.ArmModel.tic();

            if (UavArm_tools.getTrackingState() > 0 || UavArm_tools.getTrackingState() == 20)
            { //Tracking OK
                // Robot_Commands.Calculate_and_Send_Commands(LocalUAVPose); done in control thread
                TrackingState++;
                //Publicar aqui la pose absoluta del UAV para su control, tracking system
                if (TrackingState > 50)
                    TrackingState = 50;
            }
            else
            {
                NoVisualContact_count++;
                // std::this_thread::sleep_for(std::chrono::milliseconds(5));
                if (NoVisualContact_count > 155) //2 secs
                {
                    Print("Tracking state", UavArm_tools.getTrackingState(), NoVisualContact_count);

                    RRT_model.reset_nodes_reordered();
                    UavArm_tools.PIDReset(); //reset, set zeros to errors and integrals
                    TrackingState = 0;
                    //UavArm_tools.ArmPoseReq_decreaseAlt(0.02); //modifies the arm request to lower the end effector
                    UavArm_tools.counter = 0;
                    RRT_model.loop_end();
                    NextArmRequest = target_posea;
                    /*NextArmRequest.position.y = UAV_position_y;
                    NextArmRequest.position.z = alturap;*/
                    CurrentRequest_Thread = NextArmRequest;
                    UavArm_tools.CalculateDockingAltitude();
                    NextArmRequest.position.z = UavArm_tools.getEEFFAltitude();
                    Print("Altitude", NextArmRequest.position.z);
                    UavArm_tools.setArmPoseReq(NextArmRequest);
                    NoVisualContact_count = 0;
                }
                else
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }

                //
                //NextArmRequest = Predict_B.NoTarget_Sequence(target_posea);
            }
            if (TrackingState > 5 && UavArm_tools.Controller_Commands.tracking_process)
            {
                UavArm_tools.counter_addOne(); // para envio espaciado de orientaciones
                                               // m.lock();
                //UavArm_tools.uavPose_to_ArmPoseReq_full(); // for rrt process
                //CurrentArmPose = RRT_model.ArmModel.getCurrentPose();
                //UavArm_tools.UpdateArmCurrentPose(CurrentArmPose);
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
                    trust_factor = 0.2;
                }

                UavArm_tools.setAltitudeRequest(UavArm_tools.getEEFFAltitude());
                //CurrentRequest_Thread = UavArm_tools.getArmPoseReqFull();// with mutex
                CurrentRequest_Thread = UavArm_tools.getArmPoseReq(); // with mutex
                if (true || UavArm_tools.Controller_Commands.docking_process)
                    NextArmRequest = CurrentRequest_Thread;
                else
                {
                    //Print("Calculated UAV pose ", CurrentRequest_Thread.position.x, CurrentRequest_Thread.position.y);
                    Predict_B.Load_UGV_State(Robot_Commands.ugv_state); //sequential
                    Predict_B.Planif_SequenceA(CurrentRequest_Thread, CurrentArmPose, UavArm_tools.Controller_Commands.docking_process);
                    Predict_B.Charge_Nodes();
                    RRT_model.loop_start();
                    Predict_B.RRT_Path_Generation();

                    NextArmRequest = Predict_B.Selection_Function(trust_factor); //CurrentRequest_Thread;
                }
            }
            arm_control_msg_mtx.lock();
            fresh_request = true;
            ArmRequest = NextArmRequest;
#ifdef OPENCV_DRAW
            image_general = Predict_B.getImage_Ptraj();
#endif
            Accumulated_Tracking_State = TrackingState;
            CurrentPose_General = CurrentArmPose;
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

            /* 
            cv::Mat fullImage = Predict_B.getImage_Ptraj();
            cv::circle(fullImage, cv::Point(round((CurrentRequest_Thread.position.x + max_dimm) * scale), round((CurrentRequest_Thread.position.y + max_dimm) * scale)),
                       4, cv::Scalar(0, 200, 0), -1, 8);
            cv::Mat CroppedImage = fullImage(myROI);
               openCV_mutex.lock();

            cv::imshow(window_name, CroppedImage);
            cv::waitKey(1);
            */
            // openCV_mutex.unlock();
#endif
            //std::this_thread::sleep_for(std::chrono::milliseconds(35));
            loop_rate.sleep();
            ros::spinOnce();
            //cout << "=====> SEQUENCE A TIME: " << RRT_model.ArmModel.toc(clA).count() << endl;
            /*  double LoopA_time = RRT_model.ArmModel.toc(clA).count();
            if (LoopA_time > 100000)
                Print("===--->SEQUENCE A TIME HUGE!!!!", LoopA_time);
            if (LoopA_time < 30000)
                Print("===--->SEQUENCE A TIME LESS!!!!", LoopA_time);
            clA = std::chrono::high_resolution_clock::now();*/
        }
    });

    //========================CONTROL THREAD===========================================================

    auto threadControl = std::thread([&]() {

#ifdef OPENCV_DRAW
        cv::namedWindow("Marker pos", cv::WINDOW_NORMAL);
#endif
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        ros::Rate loop_rateControl(35);
        double non_tracking_height_corr = 0.2;
        double x_correction = 0.0;
        double y_correction = 0.0;
        geometry_msgs::Pose OldLocalUAVPose = target_posea;
        deque<double> localUAV_Vel;
        cv::Mat Image_control_loop;
        bool comes_from_storage = false;
        auto timestamp = std::chrono::high_resolution_clock::now();
        while (ros::ok())
        {
            auto timestamp1 = std::chrono::high_resolution_clock::now();
            arm_control_msg_mtx.lock();
            geometry_msgs::Pose CurrentArmRequest = ArmRequest;
            bool fresh_request_local = fresh_request;
            fresh_request = false;
#ifdef OPENCV_DRAW
            geometry_msgs::Pose CurrentArm_pose = CurrentPose_General;

            image_general.copyTo(Image_control_loop);
#endif
            int tracking_state_local = Accumulated_Tracking_State;
            arm_control_msg_mtx.unlock();
            geometry_msgs::Pose LocalUAVPose;
            Image_control_loop = cv::Mat(image_size, image_size, CV_8UC3, cv::Scalar(255, 255, 255));

            if (tracking_state_local > 3)
            {
                LocalUAVPose = UavArm_tools.Calc_LocalUAVPose();
                // RRT_model.ArmModel.PrintPose("UAV Marker", LocalUAVPose);
                //if (UavArm_tools.Controller_Commands.tracking_process)
                if (UavArm_tools.Controller_Commands.docking_process)
                {
                    x_correction = 0.08;
                    y_correction = -0.04;
                    non_tracking_height_corr = -0.3;
                }
                else
                {
                    x_correction = 0.0;
                    y_correction = 0.0;
                    non_tracking_height_corr = 0.0;
                }
                if (UavArm_tools.Controller_Commands.docking_process && UavArm_tools.Controller_Commands.tracking_process)
                {
                    non_tracking_height_corr = -0.4;
                }
                Robot_Commands.Calculate_and_Send_Commands(LocalUAVPose, non_tracking_height_corr, x_correction, y_correction);

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
            else
            {
                Robot_Commands.Send_Empty_Commands();
            }

#ifdef OPENCV_DRAW
            cv::circle(Image_control_loop, cv::Point(round((LocalUAVPose.position.x + max_dimm) * scale), round((LocalUAVPose.position.y + max_dimm) * scale)),
                       5, cv::Scalar(200, 0, 0), -1, 8);
            //Print("Current arm pose", CurrentArm_pose.position.x, CurrentArm_pose.position.y);
            cv::circle(Image_control_loop, cv::Point(round((CurrentArm_pose.position.x + max_dimm) * scale), round((CurrentArm_pose.position.y + max_dimm) * scale)),
                       5, cv::Scalar(0, 250, 0), -1, 8);
            cv::circle(Image_control_loop, cv::Point(round((CurrentArmRequest.position.x + max_dimm) * scale), round((CurrentArmRequest.position.y + max_dimm) * scale)),
                       5, cv::Scalar(0, 0, 250), -1, 8);
#endif
            //  Print("Current pose request", CurrentArmRequest.position.x, CurrentArmRequest.position.y);
            //  y_correction = CurrentArmRequest.position.y;
            if (fresh_request_local && UavArm_tools.Controller_Commands.tracking_process && !UavArm_tools.Controller_Commands.storage_process) //tracking
            {
                if (comes_from_storage)
                {
                    RRT_model.ArmModel.SendMovement_byJointsValues(joint_values_storage_transition);
                    sleep(3.7);
                    comes_from_storage = false;
                }
                control_msgs::FollowJointTrajectoryGoal ArmGoal = RRT_model.ArmModel.Req_Joints_byPose_FIx_Orientation(CurrentArmRequest);

                if (ArmGoal.trajectory.points.size() > 0)
                {
                    // ArmGoal = RRT_model.SteerJoints(ArmGoal);
                    //  Print("JOINTS REQUEST", ArmGoal.trajectory.points[0].positions[0], ArmGoal.trajectory.points[0].positions[1],
                    //  ArmGoal.trajectory.points[0].positions[2], ArmGoal.trajectory.points[0].positions[3],
                    //      ArmGoal.trajectory.points[0].positions[4], ArmGoal.trajectory.points[0].positions[5]);

                    RRT_model.ArmModel.Request_Movement_byJointsTrajectory(ArmGoal);
                }

                //RRT_model.ArmModel.ReqMovement_byPose_FIx_Orientation(CurrentRequest_Thread); // CurrentRequest_Thread   NextArmRequest

                UavArm_tools.Load_PID_time(RRT_model.ArmModel.getDelayTime().count() / 1000000.0);
            }
            else
            {
                if (!UavArm_tools.Controller_Commands.tracking_process && !UavArm_tools.Controller_Commands.storage_process) //no tracking
                {
                    if (comes_from_storage)
                    {
                        RRT_model.ArmModel.SendMovement_byJointsValues(joint_values_storage_transition);
                        sleep(3.7);
                        comes_from_storage = false;
                    }
                    auto ArmGoal = RRT_model.ArmModel.Req_Joints_byPose_FIx_Orientation(target_pose_noTracking); //return to origin

                    if (ArmGoal.trajectory.points.size() > 0)
                    {
                        // ArmGoal = RRT_model.SteerJoints(ArmGoal);
                        //  Print("JOINTS REQUEST", ArmGoal.trajectory.points[0].positions[0], ArmGoal.trajectory.points[0].positions[1],
                        //     ArmGoal.trajectory.points[0].positions[2], ArmGoal.trajectory.points[0].positions[3],
                        //       ArmGoal.trajectory.points[0].positions[4], ArmGoal.trajectory.points[0].positions[5]);
                        RRT_model.ArmModel.Request_Movement_byJointsTrajectory(ArmGoal);
                    }
                }
                if (UavArm_tools.Controller_Commands.storage_process)
                {

                    RRT_model.ArmModel.SendMovement_byJointsValues(joint_values_storage);
                    sleep(1.0);
                    comes_from_storage = true;
                }
            }
            timestamp1 = std::chrono::high_resolution_clock::now();

#ifdef OPENCV_DRAW
            //  openCV_mutex.lock();
            cv::imshow("Marker pos", Image_control_loop);
            cv::waitKey(1);
            // cout << "=====> SEQUENCE CONTROL TIME opencv: " << RRT_model.ArmModel.toc(timestamp1).count() << endl;
#endif
            // cout << "=====> SEQUENCE CONTROL TIME: " << RRT_model.ArmModel.toc(timestamp).count() << endl;

            // openCV_mutex.unlock();
            loop_rateControl.sleep();
            timestamp = std::chrono::high_resolution_clock::now();
        }
    });

    // if (rrt_thread.joinable()) rrt_thread.join();
    //if (main_thread.joinable()) main_thread.join();

    rrt_threadB.join();
    rrt_threadA.join();
    threadControl.join();
    ros::waitForShutdown();
}
#endif
