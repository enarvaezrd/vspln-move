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
    rrt_planif::RRTA RRT_modelB;
    //cv::namedWindow("Image1",cv::WINDOW_NORMAL);
    bool RRT_Calc_state=false;

    auto rrt_thread = std::thread([&](){
    bool sequence_loop_th=false;
    bool Thread_Run=true;
    int cnTh=0;
    while(Thread_Run){
        while (sequence_loop_th && RRT_model.get_finish())
        {   mtx.lock();
            const rrt_planif::Etraj etr=RRT_modelB.Get_TR();
            RRT_model.Load_TR(etr);
            RRT_model.Load_TRbr(RRT_modelB.Get_TRbr());
            RRT_model.Load_Img(RRT_modelB.getImage_Ptraj());
           
            RRT_model.RRT_SequenceB();
           
            const cv::Mat image = RRT_model.getImage_Ptraj();
            
            
            sequence_loop_th = RRT_model.getLoopState();
            cv::imshow("Image1",image);
            cv::waitKey(2);
            mtx.unlock();
            cnTh++;
            Print("//  finish RRT", cnTh);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
    double cnt1=0.0;
    bool sequence_loop=true;
    int steps=300;
   
    while (sequence_loop)
        {mtxA.lock();
            Print("=======Step=====",cnt1);
            cnt1++;
            count = count+b;
            //if (count> 100) b=-1;
            //if (count<-100) b=1;
            CurrentRequest.position.x = 0.3*cos(PI*count/steps);
            CurrentRequest.position.y = 0.3*sin(PI*count/steps);
            Print("Current point",CurrentRequest.position.x,count);

                
                //RRT_modelB.SetArmPose(CurrentRequest);
                RRT_modelB.Load_NdsReord(RRT_model.Get_NdsReord());
                RRT_modelB.RRT_SequenceA(CurrentRequest);
               
            cv::Mat imageA;

            //mtx.lock();

            imageA=RRT_modelB.getImage_Ptraj();
            cv::imshow("Image11",imageA);
            cv::waitKey(1); 
              
            //mtx.unlock();
            RRT_model.loop_start();
            //Print("finish, now pause");
            std::this_thread::sleep_for(std::chrono::milliseconds(35));
            //Print("finish already paused");
            mtxA.unlock();
        }
 });
Print("finish main paused");
rrt_threadA.join();
rrt_thread.join();

}
#endif