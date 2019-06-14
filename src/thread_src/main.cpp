#ifndef MAIN_APP
#define MAIN_APP


#include "uav_arm_tools.hpp"

using namespace std;
int main(int argc, char** argv)
{
    //===========VARIABLES===================

    int image_size=800;
     
    int d_prv = 5;      // profundidad de datos previos disponibles para prediccion
    int d_pr_m = 3;     // datos previos a usar para calculo de mean values
    int prof_expl = 13;  // Profundidad de exploracion  Esz=prof_f
    int Map_size = 4000;
    float scale = 1.0;
    //=======================================
    ros::init(argc, argv, "arm_program");
    ros::NodeHandle nodle_handle;
    ros::Rate loop_rate(30);
    Printer Print;
    rrt_planif::RRT RRT_model;
    PredNs::Prediction Predict_B(image_size,d_prv, d_pr_m, prof_expl,Map_size,scale);
    ObstacleMapGen ObstacleMap(Map_size,scale);

    ua_ns::uav_arm_tools UavArm_tools;
    sleep(1.0);

    RRT_model.ArmModel.PrintModelInfo();
    std::vector<double> joint_valuesT(6);
    std::mutex m;
    cout<<"start"<<endl;
    joint_valuesT[0] = 0.0;
    joint_valuesT[1] = -2.0;//PI/2;
    joint_valuesT[2] = -2.0;//PI/2;
    joint_valuesT[3] = 0.0;// PI/2;
    joint_valuesT[4] = 0.0;
    joint_valuesT[5] = 0.0;// PI/2;

    RRT_model.ArmModel.SendMovement_byJointsValues(joint_valuesT);
    sleep(1.0);
    RRT_model.ArmModel.PrintCurrentPose("STARTING POSEAAAA");
    float alturap=0.72;//0.21

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
    sleep(1.0);
    geometry_msgs::Pose target_posea = RRT_model.ArmModel.getCurrentPose();


    IAngleMark =UavArm_tools.ConvPosetoAngles(target_posea);
    Print("PETA ANGLES yaw,roll,pitch", IAngleMark.yaw,IAngleMark.roll,IAngleMark.pitch);

    //if (reqState==true)
        UavArm_tools.UpdateArmCurrentPose(RRT_model.ArmModel.getCurrentPose());

    int TrackingState=0;
    int ThreadStep=0;
    geometry_msgs::Pose CurrentRequest, CurrentRequest_Thread;

   auto rrt_threadB = std::thread([&](){
       ros::Rate loop_rate_thread(30);
       std::this_thread::sleep_for(std::chrono::milliseconds(6000));
       bool sequence_loop_th=false;
       //sleep(6.0);
       auto clB=std::chrono::high_resolution_clock::now();
       while(ros::ok())
        {
           while (sequence_loop_th && ros::ok())
           {    //Print("=====Step=====");
                //Ed_Pmov ARMMdl=RRT_modelA.Get_ArmModel();
                //RRT_modelB.Load_ArmModel(ARMMdl);
                RRT_model.Load_Adv(Predict_B.Get_Adv());
                RRT_model.Load_TR(Predict_B.Get_TR());
                RRT_model.Load_TRbr(Predict_B.Get_TRbr());
                RRT_model.ResetImagePtraj();
                Print("stop flag",Predict_B.get_Stop_RRT_Flag(),RRT_model.get_TR_Size());
                if (RRT_model.get_TR_Size()>0 &&!Predict_B.get_Stop_RRT_Flag())   //trajA.xval.size()>0 &&
                   { Print("entering");RRT_model.RRT_SequenceB(); 
                Predict_B.Load_Nodes(RRT_model.GetNodes());}

               /* else 
                {
                    Print("RRT not applied, Traj too short or RRT flag stop");
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }*/
                sequence_loop_th = RRT_model.getLoopState();
                //Print("b33",sequence_loop_th);
                #ifdef OPENCV_DRAW
                    cv::Mat image = RRT_model.getImage_Ptraj();
                    //cv::imshow("ImageSeqB",image);
                    //cv::waitKey(1); 
                #endif
                //std::cout<<"Seq B time: "<<RRT_modelB.ArmModel.toc(clB).count()<<std::endl;
                Print("==SEQUENCE B TIME ",RRT_model.ArmModel.toc(clB).count());
                clB=std::chrono::high_resolution_clock::now();
               // RRT_modelB.ArmModel.getDelayTime();
                //std::this_thread::sleep_for(std::chrono::milliseconds(80));
                
            //ros::spinOnce();
            loop_rate_thread.sleep();
           }
           while(!sequence_loop_th && ros::ok())
           {
               Print("RRT paused ");
               std::this_thread::sleep_for(std::chrono::milliseconds(100));
               sequence_loop_th = RRT_model.getLoopState();
               //ros::spinOnce();
               loop_rate_thread.sleep();
           }
        }
        });
 auto rrt_threadA = std::thread([&](){
        //std::unique_lock<std::mutex> lck(mtx_main);
        auto clA=std::chrono::high_resolution_clock::now();
    while(ros::ok())
    {

        Predict_B.Load_Map(ObstacleMap.get_Map());//Load Obstacle Map
        Predict_B.Draw_Map();
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
            Print("ENTER IN RESET of B");
            RRT_model.loop_end();
            
            RRT_model.reset_nodes_reordered();
            UavArm_tools.PIDReset(); //reset, set zeros to errors and integrals
            TrackingState=0;
            UavArm_tools.ArmPoseReq_decreaseAlt(0.02); //modifies the arm request to lower the end effector
            UavArm_tools.counter=0;
        }

        if (TrackingState > 1)
        {
            UavArm_tools.counter_addOne(); //para envio espaciado de orientaciones
            m.lock();
            //UavArm_tools.uavPose_to_ArmPoseReq_full(); //for rrt process
            UavArm_tools.uavPose_to_ArmPoseReq_arm();  //for visual servoing
            m.unlock();
           
            UavArm_tools.setAltitudeRequest(UavArm_tools.getMinArmAltitude());
            
            CurrentRequest_Thread = UavArm_tools.getArmPoseReqFull();//with mutex

            //RRT_modelA.ArmModel.PrintPose("req th pose ",CurrentRequest_Thread);
            //RRT_modelA.Load_NdsReord(RRT_modelB.Get_NdsReord());
            Predict_B.Planif_SequenceA(CurrentRequest_Thread);
            Predict_B.Charge_Nodes();
            Predict_B.Selection();
            RRT_model.loop_start();
        }


        //CurrentRequest = UavArm_tools.getArmPoseReq();
        //RRT_model.ArmModel.PrintPose("Req",CurrentRequest);
        std::chrono::microseconds  elapsed_time =RRT_model.ArmModel.toc();

       // RRT_model.loop_end();
        RRT_model.ArmModel.Sleep(elapsed_time); //sleep the resulting time
        //RRT_model.loop_start();
       
        RRT_model.ArmModel.ReqMovement_byPose(CurrentRequest_Thread ,2); //type 1 with normal execution, type 2 for last joint preference
        
        UavArm_tools.PIDdata.time = RRT_model.ArmModel.getDelayTime().count()/1000000;
        CurrentArmPose = RRT_model.ArmModel.getCurrentPose();
        UavArm_tools.UpdateArmCurrentPose(CurrentArmPose);

            cv::Mat imageA=Predict_B.getImage_Ptraj();
            cv::imshow("Image11",imageA);
         cv::waitKey(1); 
            //std::this_thread::sleep_for(std::chrono::milliseconds(35));
            loop_rate.sleep();
//            Print("SEQUENCE A TIME ",RRT_modelA.ArmModel.toc(clA).count());
            clA=std::chrono::high_resolution_clock::now();
    }
});
auto laser_thread = std::thread([&](){
    ros::Rate loop_rate_map(30);
    while (ros::ok())
    {
        ObstacleMap.CreateMap();
        loop_rate_map.sleep();
    }
});

   // if (rrt_thread.joinable()) rrt_thread.join();
//if (main_thread.joinable()) main_thread.join();
rrt_threadB.detach();
rrt_threadA.join();
}
#endif
































