#ifndef MAIN_APP
#define MAIN_APP

#include <thread>
#include <chrono>
#include "rrt_functionss.hpp"

using namespace std;
int main(int argc, char** argv)
{

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
    bool sequence_loop=true;
    double count=0,b=1,cnt1=0;
    while (sequence_loop)
           {
               Print("=======Step=====",cnt1);
               cnt1++;
                count = count+b;
                if (count> 100) b=-1;
                if (count<-100) b=1;
                CurrentRequest.position.x = count/250;
                Print("Current point",CurrentRequest.position.x,count);
                RRT_model.RRT_Sequence(CurrentRequest);
                 Print("end rrt,now image");
                 cv::Mat image=cv::Mat::zeros( 400, 400, CV_8UC3 );
                 image=RRT_model.getImage_Ptraj();
                 Print("imagesize",image.size().width);
                cv::imshow("Image1",image);
                cv::waitKey(1);
                Print("finish, now pause");
               std::this_thread::sleep_for(std::chrono::milliseconds(100));
               Print("finish already paused");

           }
}
#endif