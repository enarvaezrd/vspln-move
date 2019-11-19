#ifndef INCLUDE_LIBS
#define INCLUDE_LIBS

//ed_pmov
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <queue>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
//traj_client

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>

//uav_arm_tools
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

//Prediction
#include <sensor_msgs/LaserScan.h>

//UAV control message
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <thread>
#include <future>
#include <mutex>
#include <condition_variable>

#define OPENCV_DRAW
#define PI 3.141592654
#define PRINT
#define SREAMING
#define WIRELESS_CONTROLLER
using namespace std;
struct Position_
{
    double x = 0.0, y = 0.0, z = 0.0;
};
struct Velocity_
{
    double dx = 0.0, dy = 0.0, dz = 0.0;
};

struct Orientation_
{
    double w = 0.0, x = 0.0, y = 0.0, z = 0.0;
};
struct RobotState_
{
    Position_ position;
    Orientation_ orientation; 
    Velocity_ velocity_linear;
    Velocity_ velocity_angular;
};
struct Angles
{
    float yaw = 0.0, roll = 0.0, pitch = 0.0;
};

struct Quat
{
    double x = 0.0, y = 0.0, z = 0.0, w = 0.0;
};

typedef vector<double> VectorDbl;
typedef vector<int> VectorInt;

typedef std::chrono::high_resolution_clock Clock;
struct Nodes
{
    vector<VectorDbl> coord;
    vector<VectorDbl> coordT;
    VectorDbl cost;
    VectorDbl costParent;
    vector<int> parent;
    vector<int> id; //No usado por ahora
    vector<int> region;
    int N; //Numero de nodos activos
};

struct Node
{
    VectorDbl coord;
    VectorDbl coordT;
    double cost;
    double costParent;
    int parent;
    int id;
    int region;
};
struct Etraj
{ //Trajectory vector
    VectorDbl xval;
    VectorDbl yval;
    VectorDbl zval;
    VectorDbl w;
    VectorDbl x;
    VectorDbl y;
    VectorDbl z;
};
struct Position
{ //Only position
    double xval;
    double yval;
    double zval;
};
struct Positions
{ //Only positions
    VectorDbl xval;
    VectorDbl yval;
    VectorDbl zval;
};
namespace rrtns
{
struct MeanValues
{
    double vx, vy, vz;
};
} // namespace rrtns
struct Vicinity
{
    vector<VectorDbl> TP;
    vector<vector<long double>> R;
    //std::vector<std::vector<VectorDbl > > RP;
    vector<VectorDbl> angles;
    VectorDbl N;
    int L;
};
struct LaserDataS
{
    VectorDbl ranges;
    VectorDbl intensities;
    float range_min;
    float range_max;
    float min_angle;
    float max_angle;
    float angle_increment;
    int size;
    bool state;
};

class NumberCorrection
{
public:
    NumberCorrection() {}
    void MinMax_Correction(double &value, double max_value)
    {
        if (value > max_value)
            value = max_value;
        if (value < -max_value)
            value = -max_value;
        return;
    }
    void MinMax_Correction(float &value, double max_value)
    {
        if (value > max_value)
            value = max_value;
        if (value < -max_value)
            value = -max_value;
        return;
    }
};

class Printer
{
public:
    Printer() {}
    void operator()(std::string str_namespace, bool debug=true,
                    double a = -1111, double b = -1111,
                    double c = -1111, double d = -1111,
                    double e = -1111, double f = -1111,
                    double g = -1111)
    {
#ifndef PRINT
        return;
#endif

#ifndef DEBUG
        if (debug)
        {
            return;
        }
#endif

        std::vector<double> input;
        input.push_back(a);
        input.push_back(b);
        input.push_back(c);
        input.push_back(d);
        input.push_back(e);
        input.push_back(f);
        input.push_back(g);
        int strSize = 0;
        std::cout << "> " << str_namespace << ": ";
        strSize += 4;
        strSize += str_namespace.size();
        for (auto i : input)
        {
            //std::cout<<i;
            if (i != -1111)
            {
                std::ostringstream str_s;
                str_s << i;
                std::string str_t = str_s.str();
                std::cout << i << ", ";
                strSize += 2;
                strSize += str_t.size();
            }
        }
        std::cout << '\b';
        std::cout << ' ';
        std::cout << "\n";
        for (int i = 0; i < strSize; i++)
        {
            std::cout << '\b';
        }
        return;
    }
};
class TextStream
{
public:
    int limiter;
    bool activated;

    TextStream(string text_file) : limiter(-110),
                                   activated(true)
    {
        outputfile.open(text_file);
        start_time = std::chrono::system_clock::now();
    }
    ~TextStream()
    {
        outputfile.close();
    }

    void write_Data(double data)
    {
        if (activated)
            outputfile << data << ";";
    }
    void write_Data(string data)
    {
        if (activated)
            outputfile << data << ";";
    }
    void write_Data(VectorDbl data)
    {
        if (activated)
            for (int i = 0; i < data.size(); i++)
                outputfile << data[i] << ";";
    }
    void write_TimeStamp()
    {
        if (activated)
        {
            auto time_point = std::chrono::system_clock::now();
            std::chrono::duration<double> diff = time_point - start_time;

            outputfile << diff.count() << ";";

            auto tse = time_point.time_since_epoch();
            auto now_ms = std::chrono::duration_cast<std::chrono::microseconds>(tse);
            auto now_s = std::chrono::duration_cast<std::chrono::seconds>(tse);
            auto jst_ms = now_ms - now_s;

            outputfile << jst_ms.count() << ";" << limiter << ";\n";
        }
    }

    void jump_line()
    {
        if (activated)
            outputfile << "\n";
    }
    ofstream outputfile;
    std::chrono::system_clock::time_point start_time;
};

#endif
