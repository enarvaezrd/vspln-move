
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#define PI 3.141592654
#include <thread>
#include <chrono>
#include <mutex>
#include <algorithm>
#include <random>
#define PRINT
#define OPENCV_DRAW

using namespace std;  
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

typedef vector<double> VectorDbl;
typedef vector<int> VectorInt;

typedef std::chrono::high_resolution_clock Clock;
struct Nodes{
   vector<VectorDbl >  coord;
   vector<VectorDbl >  coordT;
   VectorDbl cost;
   VectorDbl costParent;
   vector<int >   parent;
   vector<int >   id;  //No usado por ahora
   vector<int >   region;
   int N;                   //Numero de nodos activos
};

struct Node{
     VectorDbl coord;
     VectorDbl coordT;
     double cost;
     double costParent;
     int parent;
     int id;
     int region;
};
struct Etraj { //Trajectory vector
     VectorDbl xval;
     VectorDbl yval;
     VectorDbl zval;
     VectorDbl w;
     VectorDbl x;
     VectorDbl y;
     VectorDbl z;
};
struct Position { //Only position
     double xval;
     double yval;
     double zval;
};
struct Positions { //Only positions
     VectorDbl xval;
     VectorDbl yval;
     VectorDbl zval;
};
namespace rrtns{
struct MeanValues{
    double vx,vy,vz;
};
}
struct Vicinity{
   vector<VectorDbl >  TP;
   vector<vector<long double> >  R;
   //std::vector<std::vector<VectorDbl > > RP;
   vector<VectorDbl > angles;
   VectorDbl N;
   int L;
};


class Printer{
public:
    Printer(){}
    void operator()(std::string str, double a = -11111, double b = -11111 , double c = -11111,double d = -11111,double e = -11111,double f = -11111,double g = -11111)
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
        input.push_back(g);
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
        std::cout<<"\n";
       /* for (int i = 0; i < strSize; i++)
        {
            std::cout<<'\b';
        }*/
        return;
    }

};
