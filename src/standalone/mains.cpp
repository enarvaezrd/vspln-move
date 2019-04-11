#ifndef MAIN_APP
#define MAIN_APP


#include "rrt_functionss.hpp"

using namespace std;
int main(int argc, char** argv)
{
    std::mutex mtx,mtxA;
    rrt_planif::RRT RRT_model;
    geometry_msgs::Pose CurrentRequest;
    CurrentRequest.orientation.w =0;
    CurrentRequest.orientation.x =0;
    CurrentRequest.orientation.y =0;
    CurrentRequest.orientation.z =0;
    CurrentRequest.position.x =0;
    CurrentRequest.position.y =0;
    CurrentRequest.position.z =0;
     Printer Print;
     Print("=============================================Start Program======================================");
    
    double count=0,b=1;
    PredNs::Prediction Predict_B;
    bool RRT_Calc_state=false;

    auto rrt_thread = std::thread([&](){
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

     cv::namedWindow("ImagepTraj", 0);
    cv::resizeWindow("ImagepTraj", 1500,1500);
        bool sequence_loop_th=false;
        bool Thread_Run=true;
        int cnTh=0;
        auto clB=std::chrono::high_resolution_clock::now();
        while(Thread_Run){
            while (sequence_loop_th )
            {  
                Print("=======Step=====",cnTh);
                //const rrt_planif::Etraj etr=RRT_modelB.Get_TR();
                RRT_model.Load_Adv(Predict_B.Get_Adv());
                RRT_model.Load_TR(Predict_B.Get_TR());
                RRT_model.Load_TRbr(Predict_B.Get_TRbr());
                RRT_model.ResetImagePtraj();

                RRT_model.RRT_SequenceB();
                Print("load");
                Predict_B.Load_Nodes(RRT_model.GetNodes());
                Print("loaded");
                sequence_loop_th = RRT_model.getLoopState();
            #ifdef OPENCV_DRAW
                const cv::Mat image = RRT_model.getImage();
                const cv::Mat imagePt = RRT_model.getImage_Ptraj();

                //cv::imshow("Image1B",image);

                cv::imshow("ImagepTraj",imagePt);
                cv::waitKey(1);
            #endif
                
                cnTh++;
                Print("//  finish RRT", cnTh);
                //std::this_thread::sleep_for(std::chrono::milliseconds(1));
                Print("SEQUENCE B TIME ",RRT_model.toc(clB).count());
                clB=std::chrono::high_resolution_clock::now();
            }
            while(!sequence_loop_th){
                Print("RRT paused ");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                sequence_loop_th = RRT_model.getLoopState();
                Print("RRT paused ",sequence_loop_th);
            }
        }  
    });
 auto rrt_threadA = std::thread([&](){
    /*cv::namedWindow("Image1", 0);
    cv::resizeWindow("Image1", 1500,1500);*/
    double cnt1=0.0;
    bool sequence_loop=true;
    int steps=200;
   auto clA=std::chrono::high_resolution_clock::now();
    while (sequence_loop)
        {
            //Print("=======Step=====",cnt1);
            cnt1++;
            count = count+b;
            //if (count> 100) b=-1;
            //if (count<-100) b=1;
            CurrentRequest.position.x = 0.3*cos(PI*count/steps);
            CurrentRequest.position.y = 0.3*sin(PI*count/steps);
            CurrentRequest.position.z = 0.5;
            //Print("Current point",CurrentRequest.position.x,count);

                
             
                //RRT_modelB.Load_NdsReord(RRT_model.Get_NdsReord());
                Predict_B.Planif_SequenceA(CurrentRequest);              
            Predict_B.Charge_Nodes();
            Predict_B.Selection();
              
            cv::Mat imageA;

            //mtx.lock();
        #ifdef OPENCV_DRAW
            imageA=Predict_B.getImage_Ptraj();
            cv::imshow("Image1",imageA);
            cv::waitKey(1);
        #endif
            RRT_model.loop_start();
            //Print("finish, now pause");
            std::this_thread::sleep_for(std::chrono::milliseconds(80));
            //Print("finish already paused");
            Print("SEQUENCE A TIME ",RRT_model.toc(clA).count());
            clA=std::chrono::high_resolution_clock::now();
        }
 });
Print("finish main paused");
rrt_thread.detach();
rrt_threadA.join();
}
#endif