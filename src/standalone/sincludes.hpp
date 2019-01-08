
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#define PI 3.141592654
struct Position{
    double x,y,z;
};
struct Orientation{
    double w,x,y,z;
};
namespace geometry_msgs{
class Pose{
    
    public:
    Pose(){};
    Orientation orientation;
    Position position;
};
}
class Printer{
public:
    Printer(){}
    void operator()(std::string str, double a = -1111, double b = -1111 , double c = -1111,double d = -1111,double e = -1111,double f = -1111)
    {
        #ifdef VERBOSE 
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
            if (i != -1111)
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
        for (int i = 0; i < strSize; i++)
        {
            std::cout<<'\b';
        }
        return;
    }

};
