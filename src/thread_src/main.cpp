#ifndef MAIN_APP
#define MAIN_APP
#define threadMode


#include "uav_arm_tools.hpp"

using namespace std;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_program");
    ros::NodeHandle nodle_handle;
    ros::Rate loop_rate(30);
    Printer Print;
    rrt_planif::RRT RRT_model,RRT_modelB;
    ua_ns::uav_arm_tools UavArm_tools;
    sleep(2.0);

    std::vector<double> joint_valuesT(6);
    std::mutex m;
    joint_valuesT[0] = 0.0;
    joint_valuesT[1] = -PI/2;
    joint_valuesT[2] = -PI/2;
    joint_valuesT[3] = 0.0;// PI/2;
    joint_valuesT[4] = 0.0;
    joint_valuesT[5] = PI/2;

    RRT_model.ArmModel.SendMovement_byJointsValues(joint_valuesT);
    sleep(4.0);
    RRT_model.ArmModel.PrintCurrentPose("STARTING POSE");

    float alturap=0.43;//0.21

    geometry_msgs::Pose target_pose = RRT_model.ArmModel.getCurrentPose();


   ua_ns::Angles IAngleMark =UavArm_tools.ConvPosetoAngles(target_pose);
   Print("PET ANGLES yaw,roll,pitch", IAngleMark.yaw,IAngleMark.roll,IAngleMark.pitch);

    /*target_pose.orientation.w = 0.0;//0.1
    target_pose.orientation.x = 0.0;//0.1
    target_pose.orientation.y = 1.0;
    target_pose.orientation.z = 0.0;*/

    target_pose.position.x = 0.28;//0.1
    target_pose.position.y = 0.01;
    target_pose.position.z = alturap;

    UavArm_tools.setArmPoseReq(target_pose);
    target_pose = UavArm_tools.getArmPoseReq();
    bool reqState=RRT_model.ArmModel.ReqMovement_byPose(target_pose,1);
    sleep(3.0);
    geometry_msgs::Pose target_posea = RRT_model.ArmModel.getCurrentPose();


    IAngleMark =UavArm_tools.ConvPosetoAngles(target_posea);
    Print("PETA ANGLES yaw,roll,pitch", IAngleMark.yaw,IAngleMark.roll,IAngleMark.pitch);

    //if (reqState==true)
        UavArm_tools.UpdateArmCurrentPose(RRT_model.ArmModel.getCurrentPose());

    int TrackingState=0;
    int ThreadStep=0;
    geometry_msgs::Pose CurrentRequest, CurrentRequest_Thread;
#ifdef threadMode
   auto rrt_thread = std::thread([&](){
       ros::Rate loop_rate_thread(20);
       std::this_thread::sleep_for(std::chrono::milliseconds(2000));
       bool sequence_loop_th=false;
       sleep(10.0);
       auto clB=std::chrono::high_resolution_clock::now();
       while(ros::ok())
        {
           while (sequence_loop && ros::ok()&& RRT_model.get_finish())
           {    Print("=====Step=====");
                RRT_model.Load_TR(RRT_modelB.Get_TR());
                RRT_model.Load_TRbr(RRT_modelB.Get_TRbr());
                
                RRT_model.ResetImagePtraj();
                RRT_model.RRT_SequenceB();
                sequence_loop_th = RRT_model.getLoopState();

            #ifdef OPENCV_DRAW
                const cv::Mat image = RRT_model.getImage_Ptraj();
                cv::imshow("Image1",image);
                cv::waitKey(1);
            #endif


                RRT_model.ArmModel.getDelayTime();
                //std::this_thread::sleep_for(std::chrono::milliseconds(80));
                loop_rate_thread.sleep();
                Print("-- get loop state");
                sequence_loop = RRT_model.getLoopState();
            //ros::spinOnce();
           }
           while(!sequence_loop && ros::ok())
           {
               Print("RRT paused ");
               std::this_thread::sleep_for(std::chrono::milliseconds(1));
               sequence_loop = RRT_model.getLoopState();
               loop_rate_thread.sleep();
               //ros::spinOnce();
           }
          
        }
        });
#endif

        //std::unique_lock<std::mutex> lck(mtx_main);
    while(ros::ok())
    {
        geometry_msgs::Pose CurrentArmPose;
        CurrentArmPose = RRT_model.ArmModel.getCurrentPose();//desde el brazo

        UavArm_tools.UpdateArmCurrentPose(CurrentArmPose);
        RRT_model.ArmModel.tic();
        //cout<<"Estado de marker: "<<UavArm_tools.getTrackingState() <<endl;
        if (UavArm_tools.getTrackingState() == 1 || UavArm_tools.getTrackingState() == 20)
        {//Tracking OK

            geometry_msgs::Pose AbsUAVPose= UavArm_tools.Calc_AbsoluteUAVPose(); //not used for now
            TrackingState++;
            //Publicar aqui la pose absoluta del UAV para su control, tracking system
            if (TrackingState > 50) TrackingState = 50;
        }
        else
        {
            #ifdef threadMode
            RRT_model.loop_end();
            #endif
            RRT_model.reset_nodes_reordered();
            UavArm_tools.PIDReset(); //reset, set zeros to errors and integrals
            TrackingState=0;
            UavArm_tools.ArmPoseReq_decreaseAlt(0.02); //modifies the arm request to lower the end effector
            UavArm_tools.counter=0;
        }

        if (TrackingState > 1)
        {
            #ifdef threadMode
            RRT_model.loop_start();
            #endif
            UavArm_tools.counter_addOne();
            //actualiza desde la posicion del brazo
            //RRT_model.ArmModel.PrintCurrentPose("PR");
            #ifdef threadMode
            m.lock();
            //UavArm_tools.uavPose_to_ArmPoseReq_full(); //for rrt process           
            UavArm_tools.uavPose_to_ArmPoseReq_arm();  //for visual servoing
            m.unlock();
            #else
            UavArm_tools.uavPose_to_ArmPoseReq_arm();  //for visual servoing
            #endif

            UavArm_tools.setAltitudeRequest(UavArm_tools.getMinArmAltitude());
            #ifndef threadMode
            CurrentRequest_Thread = UavArm_tools.getArmPoseReqFull();//with mutex
            RRT_model.RRT_Sequence(CurrentRequest_Thread);
            #endif

        }

        CurrentRequest = UavArm_tools.getArmPoseReq();
        //RRT_model.ArmModel.PrintPose("Req",CurrentRequest);
        Print(">>time stp 1");
        std::chrono::microseconds  elapsed_time =RRT_model.ArmModel.toc();
        #ifdef threadMode
       // RRT_model.loop_end();
        RRT_model.ArmModel.Sleep(elapsed_time); //sleep the resulting time
        //RRT_model.loop_start();
        Print("thread");
        #else
        Print("no thread");
        RRT_model.ArmModel.Sleep(elapsed_time); //sleep the resulting time
        #endif
        Print(">>sleep stp 2");    
        RRT_model.ArmModel.ReqMovement_byPose(CurrentRequest ,2);//type 1 with normal execution, type 2 for last joint preference
        Print(">>sleep stp 21"); 
        UavArm_tools.PIDdata.time = RRT_model.ArmModel.getDelayTime().count()/1000000;
        Print(">>current pose stp 3");
        CurrentArmPose = RRT_model.ArmModel.getCurrentPose();
        UavArm_tools.UpdateArmCurrentPose(CurrentArmPose);
        //RRT_model.ArmModel.PrintCurrentPose("FN");
        Print(">>finish stp 4" );

       /* if (recognition.joinable()&&ThreadStep>5){
            cout<<"step11"<<endl;
            //recognition.join();//check appropiate moment to join the thread
            cout<<"step12"<<endl;
            ThreadStep=0;
        }*/
       // std::thread t(&rrt_planif::RRT::RRT_Sequence,RRT_model,CurrentRequest);
        //t.join();
        cv::imshow("Image1",RRT_model.getImage_Ptraj());
        cv::waitKey(1);
        loop_rate.sleep();
    }

   // if (rrt_thread.joinable()) rrt_thread.join();
//if (main_thread.joinable()) main_thread.join();

}


#endif
































