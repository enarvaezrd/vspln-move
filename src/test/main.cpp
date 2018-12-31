#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <thread>

int var_int;

class inth
{

public:
   // inth(inth const &) = delete;             // delete the copy constructor
  //  void operator=(inth const &) = delete;
    inth()
    {
        running=true;

    }

    void loop(){

        while(running){
                   // do stuff
            addone();
        std::cout<<"inside "<<getinside()<<std::endl;
        usleep(50000);
        }
    }
    void addone(){inside=inside+1;var_int++;}
    int getinside(){return inside;}
    void exitLoop(){
           running = false;
       }

private:
    int inside;
    bool running;
};




int main(int argc, char** argv)
{
  int rate_b = 1; // 1 Hz
  var_int=0;
  ros::init(argc, argv, "mt_node");
  ros::Time::init();

  inth insideclass;


  auto thread_b = std::thread([&](){
    insideclass.loop();
  });

int cont=0;
  ros::Rate loop_rate(10); // 10 Hz
  while (ros::ok())
  { cont++;
      if (cont>50) insideclass.exitLoop();
    std_msgs::Empty msg;
    //pub_a.publish(msg);


    // process any incoming messages in this thread
    int testout=insideclass.getinside();
    //std::cout<<"outside "<<testout<<std::endl;
    std::cout<<"Var int "<<var_int<<std::endl;
    loop_rate.sleep();
    ros::spinOnce();
  }

  // wait the second thread to finish
 if (thread_b.joinable()) thread_b.join();

  return 0;
}
