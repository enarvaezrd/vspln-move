#ifndef PREDICITON
#define PREDICTION

#include "ed_pmov.hpp"

using namespace std;
namespace PredNs
{
class Prediction
{

public:
    Prediction(int img_size_, int d_prv_, int d_pr_m_, int prof_expl_, int map_size_,
               float max_dimm_, double rrt_extension_, float rad_int_, float rad_ext_) : image_size(img_size_),
                                                                                         d_prv(d_prv_),
                                                                                         d_pr_m(d_pr_m_),
                                                                                         prof_expl(prof_expl_),
                                                                                         MapSize(map_size_ + 1),
                                                                                         max_dimm(max_dimm_),
                                                                                         rrt_extension(rrt_extension_),
                                                                                         rad_int(rad_int_),
                                                                                         rad_ext(rad_ext_)
    {
        adv = 1;
        acum_values = 0;
        image_Ptraj = cv::Mat(image_size, image_size, CV_8UC3, cv::Scalar(255, 255, 255));
        White_Imag = cv::Mat(image_size, image_size, CV_8UC3, cv::Scalar(255, 255, 255));
        maxsc = max_dimm;
        scale = floor(image_size / (2.0 * maxsc));
        acum_x.resize((d_prv + 1));
        acum_y.resize((d_prv + 1));
        for (int i = 0; i < (d_prv + 1); i++)
        {
            acum_x[i] = 0.0;
            acum_y[i] = 0.0;
        } //inicializacion en ceros
        for (int i = 0; i < prof_expl / 5; i++)
        {
            Colors.push_back(cv::Scalar(0, 96, 220));
            Colors.push_back(cv::Scalar(125, 196, 245));
            Colors.push_back(cv::Scalar(106, 168, 45));
            Colors.push_back(cv::Scalar(40, 52, 171));
            Colors.push_back(cv::Scalar(30, 2, 1));
            Colors.push_back(cv::Scalar(165, 142, 59));
            Colors.push_back(cv::Scalar(114, 67, 69));
        }
        NodesAvailable = false;
        NodesCharged = false;
        New_Nodes_from_RRT = false;
        PathPlanning_Available = false;
        PathPlanningAdvancing_Index = 0;
        first_tr = true;
        
#ifdef STREAMING
        Text_Stream_RRTData = new TextStream("/home/edd/catkin_ws/src/ed_pmov/rrtdata_v1.txt");
        Text_Stream_RRTData->write_Data("x");
        Text_Stream_RRTData->write_Data("y");
        Text_Stream_RRTData->write_Data("z");
        Text_Stream_RRTData->write_Data("index");
        Text_Stream_RRTData->write_Data("type");
        Text_Stream_RRTData->write_TimeStamp();
#endif
        HalfMapSize = (MapSize - 1) / 2;
        MapResolution = (MapSize - 1) / (max_dimm * 2.0);
        ugv_state_factor = 0.5; //40%
        UAV_Velocity = 0.0;
        int cn = 0;
        for (int i = 0; i < prof_expl + adv; i++)
        {
            tr_order_indexes.push_back(1 + round(prof_expl / 2) + cn);

            if (i % 2 == 0)
            {
                cn++;
            }
            cn *= -1;
        }
        for (int i = 0; i < prof_expl + adv; i++)
        {
            cout << "NUMBERS " << tr_order_indexes[i] << endl;
        }
    }
    void Average_OldTrajectory();
    void Trajectory_Prediction(geometry_msgs::Pose Marker_Abs_Pose, geometry_msgs::Pose, bool);
    void Regression(VectorDbl x, VectorDbl y, int ndatos, int it, int order, VectorDbl &coeffs);
    void CheckandFix_Boundaries(VectorDbl &x, VectorDbl &y, int &prof_e);
    struct rrtns::MeanValues XYMean_Calculation(geometry_msgs::Pose Marker_Abs_Pose);

    void Load_Map(std::vector<VectorInt> Map, std::vector<Position> OP, std::vector<Position> OP_thick)
    {
        ObstacleMap = Map;
        Obstacle_Points_thick = OP_thick;
        Obstacle_Points = OP;
        return;
    }
    void Draw_Map();
    void Check_Recover_Trajectory(bool);
    void Check_Recover_Trajectory_tendency();
    Etraj Tr_to_Cells(Etraj tr);

    Position_ RealPosition_to_Cells(Position_ Pos);
    double Cell_to_Real(int point_cell);
    bool Check_Map_Coord(int x, int y);
    void SmoothTrajectory();

    void SmoothTrajectory_Average(int, int);

    double Get_UAV_Velocity()
    {
        Velocity_mtx.lock();
        double vel = UAV_Velocity;
        Velocity_mtx.unlock();
        return vel;
    }
    void Set_UAV_Velocity(double vel)
    {
        Velocity_mtx.lock();
        UAV_Velocity = vel;
        Velocity_mtx.unlock();
        return;
    }

    const Etraj Get_TR()
    {
        TP_Mtx.lock();
        Etraj trajectory = Tr;
        TP_Mtx.unlock();
        return trajectory;
    }
    int Get_Adv() { return adv; }
    const int Get_TRbr() { return tr_brk; }
    void Planif_SequenceA(geometry_msgs::Pose Marker_Abs_Pose, geometry_msgs::Pose, bool); //extraer vecindad

    geometry_msgs::Pose NoTarget_Sequence(geometry_msgs::Pose Marker_Abs_Pose); //No quad
    //const Nodes Get_Nodes(){return nodes;}
    void Load_Nodes(Nodes nds, Vicinity vdr)
    {
        NodesMtx.lock();
        nodes_cpy = nds;
        rrt_vicinity_copy = vdr;
        NodesAvailable = true;
        New_Nodes_from_RRT = true;
        PathPlanningAdvancing_Index = 0;
        NodesMtx.unlock();
        return;
    } //Called by B loop
    void Charge_Nodes()
    {
        if (NodesAvailable)
        {
            NodesMtx.lock();
            nodes = nodes_cpy;
            rrt_vicinity = rrt_vicinity_copy;
            NodesAvailable = false;
            NodesCharged = true;
            NodesMtx.unlock();
        }
        return;
    } //Called by A loop
    double eeff_min_height;
    const cv::Mat getImage_Ptraj() { return image_Ptraj; }
    double Distance(VectorDbl P0, VectorDbl P1);
    void RRT_Path_Generation();

    geometry_msgs::Pose Selection_Function(float);

    bool get_Stop_RRT_Flag()
    {
        flagMtx.lock();
        bool flag = Stop_RRT_flag;
        flagMtx.unlock();
        return flag;
    }
    void ClearImage_Ptraj();
    void Load_UGV_State(RobotState_ ugv_state_)
    {
        ugv_state = ugv_state_;
    }

    void InsertElement_in_Vector(VectorDbl &vector, int Position, double value);
    int Img(double point);
    double rad_to_deg(double rad);

private:
    struct rrtns::MeanValues mean;

    VectorDbl acum_x;
    VectorDbl acum_y;
    int tr_brk;
    int d_prv;     // profundidad de datos previos disponibles para prediccion
    int d_pr_m;    // datos previos para el calculo de mean values
    int prof_expl; // Profundidad de exploracion  Esz=prof_f
    int adv;       //How many points we are advancing the rrt

    cv::Mat White_Imag, image_Ptraj;
    Vicinity rrt_vicinity, rrt_vicinity_copy;
    Etraj Tr, Tr_Original;
    Etraj Tr_old, Tr_temp;
    std::mutex TP_Mtx;
    int image_size;
    double maxsc;
    double scale;
    double acum_values;
    double rrt_extension;
    std::vector<cv::Scalar> Colors;
    bool Stop_RRT_flag;
    Printer Print;
    int first_tr;
    Nodes nodes_cpy, nodes;
    mutex NodesMtx, flagMtx;
    bool NodesAvailable;
    bool NodesCharged;
    TextStream *Text_Stream_RRTData;

    float max_dimm;
    int MapSize;
    int HalfMapSize;
    double MapResolution;
    RobotState_ ugv_state;
    float ugv_state_factor;
    float rad_int, rad_ext;
    std::vector<VectorInt> ObstacleMap;
    std::vector<Position> Obstacle_Points, Obstacle_Points_thick;
    std::mutex Velocity_mtx;
    bool New_Nodes_from_RRT;
    double UAV_Velocity;
    VectorInt PathPlanning_Indexes;
    bool PathPlanning_Available;
    int PathPlanningAdvancing_Index;
    geometry_msgs::Pose Marker_Pose_Manipulator_Coords;
    NumberCorrection num;
    VectorInt tr_order_indexes;
};
} // namespace PredNs

#endif //PREDICTION
