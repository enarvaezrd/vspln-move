
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#define PI 3.141592654
#include <thread>
#include <chrono>
#include<mutex>
#define PRINT

struct Position_{
    double x,y,z;
};
struct Orientation_{
    double w,x,y,z;
};
namespace geometry_msgs{
class Pose{
    
    public:
    Pose(){};
    Orientation_ orientation;
    Position_ position;
};
}
class Printer{
public:
    Printer(){}
    void operator()(std::string str, double a = -11111, double b = -11111 , double c = -11111,double d = -11111,double e = -11111,double f = -11111)
    {
        
        #ifndef PRINT 
            return;
        #endif 
        std::vector<double> input;
        input.push_back(a);
        input.push_back(b);
        input.push_back(c);
        input.push_back(d);
        input.push_back(e);
        input.push_back(f);
        int strSize=0;
        std::cout<<"> "<<str<<": ";
        strSize += 4;
        strSize += str.size();
        for(auto i : input )
        {
            //std::cout<<i;
            if (i != -11111)
            {
                std::ostringstream str_s;
                str_s << i;
                std::string str_t = str_s.str();
                std::cout<<i<<", ";
                strSize += 2;
                strSize += str_t.size();
            }

        }
        std::cout<<std::endl;
       /* for (int i = 0; i < strSize; i++)
        {
            std::cout<<'\b';
        }*/
        return;
    }

};
