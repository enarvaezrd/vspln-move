#ifndef MAIN_APP
#define MAIN_APP


#include "uav_arm_tools.hpp"

using namespace std;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_program");
    ros::NodeHandle nodle_handle;
    ros::Rate loop_rate(20);
    Printer Print;
    rrt_planif::RRT RRT_modelA, RRT_modelB;
    ua_ns::uav_arm_tools UavArm_tools;
    sleep(2.0);

    RRT_modelA.ArmModel.PrintModelInfo();
    RRT_modelB.ArmModel.PrintModelInfo();
    std::vector<double> joint_valuesT(6);
    std::mutex m;
    joint_valuesT[0] = 0.0;
    joint_valuesT[1] = -PI/1.8;
    joint_valuesT[2] = -PI/1.8;
    joint_valuesT[3] = 0.0;// PI/2;
    joint_valuesT[4] = 0.0;
    joint_valuesT[5] = PI/1.8;

    RRT_modelA.ArmModel.SendMovement_byJointsValues(joint_valuesT);
    sleep(4.0);
    RRT_modelA.ArmModel.PrintCurrentPose("STARTING POSEAAAA");
    float alturap=0.43;//0.21

    geometry_msgs::Pose target_pose = RRT_modelA.ArmModel.getCurrentPose();


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
    bool reqState=RRT_modelA.ArmModel.ReqMovement_byPose(target_pose,1);
    sleep(3.0);
    geometry_msgs::Pose target_posea = RRT_modelA.ArmModel.getCurrentPose();


    IAngleMark =UavArm_tools.ConvPosetoAngles(target_posea);
    Print("PETA ANGLES yaw,roll,pitch", IAngleMark.yaw,IAngleMark.roll,IAngleMark.pitch);

    //if (reqState==true)
        UavArm_tools.UpdateArmCurrentPose(RRT_modelA.ArmModel.getCurrentPose());

    int TrackingState=0;
    int ThreadStep=0;
    geometry_msgs::Pose CurrentRequest, CurrentRequest_Thread;

   auto rrt_threadB = std::thread([&](){
       ros::Rate loop_rate_thread(20);
       std::this_thread::sleep_for(std::chrono::milliseconds(2000));
       bool sequence_loop_th=false;
       sleep(5.0);
       auto clB=std::chrono::high_resolution_clock::now();
       while(ros::ok())
        {
           while (sequence_loop_th && ros::ok())
           {    Print("=====Step=====");
                //Ed_Pmov ARMMdl=RRT_modelA.Get_ArmModel();
                //RRT_modelB.Load_ArmModel(ARMMdl);
                RRT_modelB.Stop_RRT_flag=RRT_modelA.Stop_RRT_flag;
                rrt_planif::Etraj trajA=RRT_modelA.Get_TR();
                RRT_modelB.Load_TR(trajA);
                Print("b1",trajA.zval.size(),trajA.zval[0]);
                int trb =RRT_modelA.Get_TRbr();
                RRT_modelB.Load_TRbr(trb);
                RRT_modelB.ResetImagePtraj();
                if (trajA.xval.size()>0 && !RRT_modelB.Stop_RRT_flag)
                    RRT_modelB.RRT_SequenceB();
                else 
                    Print("RRT not applied, Traj too short or RRT flag stop");
                sequence_loop_th = RRT_modelB.getLoopState();
                Print("b33",sequence_loop_th);
                cv::Mat image = RRT_modelB.getImage_Ptraj();
                cv::imshow("ImageSeqB",image);
                cv::waitKey(1); 
               // RRT_modelB.ArmModel.getDelayTime();
                //std::this_thread::sleep_for(std::chrono::milliseconds(80));
                
            //ros::spinOnce();
           }
           while(!sequence_loop_th && ros::ok())
           {
               Print("RRT paused ");
               std::this_thread::sleep_for(std::chrono::milliseconds(100));
               sequence_loop_th = RRT_modelB.getLoopState();
               //ros::spinOnce();
           }
        }
        });
 auto rrt_threadA = std::thread([&](){
        //std::unique_lock<std::mutex> lck(mtx_main);
    while(ros::ok())
    {
        geometry_msgs::Pose CurrentArmPose;
        CurrentArmPose = RRT_modelA.ArmModel.getCurrentPose();//desde el brazo

        UavArm_tools.UpdateArmCurrentPose(CurrentArmPose);
        RRT_modelA.ArmModel.tic();
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
            Print("ENTER IN RESET of B");
            RRT_modelB.loop_end();
            
            RRT_modelA.reset_nodes_reordered();
            UavArm_tools.PIDReset(); //reset, set zeros to errors and integrals
            TrackingState=0;
            UavArm_tools.ArmPoseReq_decreaseAlt(0.02); //modifies the arm request to lower the end effector
            UavArm_tools.counter=0;
        }

        if (TrackingState > 1)
        {
            RRT_modelB.loop_start();
            UavArm_tools.counter_addOne(); //para envio espaciado de orientaciones
            m.lock();
            //UavArm_tools.uavPose_to_ArmPoseReq_full(); //for rrt process           
            UavArm_tools.uavPose_to_ArmPoseReq_arm();  //for visual servoing
            m.unlock();

            UavArm_tools.setAltitudeRequest(UavArm_tools.getMinArmAltitude());
            
            CurrentRequest_Thread = UavArm_tools.getArmPoseReqFull();//with mutex  

            //RRT_modelA.ArmModel.PrintPose("req th pose ",CurrentRequest_Thread);        
            //RRT_modelA.Load_NdsReord(RRT_modelB.Get_NdsReord());
            RRT_modelA.RRT_SequenceA(CurrentRequest_Thread);
            
            

        }

        //CurrentRequest = UavArm_tools.getArmPoseReq();
        //RRT_model.ArmModel.PrintPose("Req",CurrentRequest);
       /* std::chrono::microseconds  elapsed_time =RRT_modelA.ArmModel.toc();
     
       // RRT_model.loop_end();
        RRT_modelA.ArmModel.Sleep(elapsed_time); //sleep the resulting time
        //RRT_model.loop_start();
        
        RRT_modelA.ArmModel.Sleep(elapsed_time); //sleep the resulting time
           
        RRT_modelA.ArmModel.ReqMovement_byPose(CurrentRequest_Thread ,2);//type 1 with normal execution, type 2 for last joint preference
        
        UavArm_tools.PIDdata.time = RRT_modelA.ArmModel.getDelayTime().count()/1000000;       
        CurrentArmPose = RRT_modelA.ArmModel.getCurrentPose();
        UavArm_tools.UpdateArmCurrentPose(CurrentArmPose);     
*/
            cv::Mat imageA=RRT_modelA.getImage_Ptraj();
            //cv::imshow("Image11",imageA);
            //cv::waitKey(2); 
            std::this_thread::sleep_for(std::chrono::milliseconds(35));
    }
    });
   // if (rrt_thread.joinable()) rrt_thread.join();
//if (main_thread.joinable()) main_thread.join();
rrt_threadB.detach();
rrt_threadA.join();
}


#endif
































