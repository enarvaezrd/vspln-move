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
               float scale_, double rrt_extension_) : image_size(img_size_),
                                                           d_prv(d_prv_),
                                                           d_pr_m(d_pr_m_),
                                                           prof_expl(prof_expl_),
                                                           MapSize(map_size_ + 1),
                                                           max_dimm(scale_),
                                                           rrt_extension(rrt_extension_)
    {
        adv = 1;
        acum_values = 0;
        image_Ptraj = cv::Mat(image_size, image_size, CV_8UC3, cv::Scalar(255, 255, 255));
        White_Imag = cv::Mat(image_size, image_size, CV_8UC3, cv::Scalar(255, 255, 255));
        maxsc = max_dimm;
        scale = floor(image_size / (2.0 * maxsc));
        acum_x.resize((d_prv + 1));
        acum_y.resize((d_prv + 1));
        for (int i = 0; i < (d_prv); i++)
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
        first_tr = true;
        Text_Stream_TR = new TextStream("/home/edd/catkin_ws/src/ed_pmov/data_trajectory.txt");
        Text_Stream_Path = new TextStream("/home/edd/catkin_ws/src/ed_pmov/data_path.txt");
        HalfMapSize = (MapSize - 1) / 2;
        MapResolution = (MapSize - 1) / (max_dimm * 2.0);
        ugv_state_factor = 0.15; //10%
    }

    void Trajectory_Prediction(geometry_msgs::Pose Marker_Abs_Pose);
    void Regression(VectorDbl x, VectorDbl y, int ndatos, int it, int order, VectorDbl &coeffs);
    void CheckandFix_Boundaries(VectorDbl &x, VectorDbl &y, int &prof_e);
    struct rrtns::MeanValues XYMean_Calculation(geometry_msgs::Pose Marker_Abs_Pose);

    void Load_Map(std::vector<VectorInt> Map, std::vector<Position> OP)
    {
        ObstacleMap = Map;
        Obstacle_Points = OP;
        return;
    }
    void Draw_Map();
    void Check_Recover_Trajectory();
    Etraj Tr_to_Cells(Etraj tr);
    double Cell_to_Real(int point_cell);
    bool Check_Map_Coord(int x, int y);
    void SmoothTrajectory();

    const Etraj Get_TR() { return Tr; }
    int Get_Adv() { return adv; }
    const int Get_TRbr() { return tr_brk; }
    void Planif_SequenceA(geometry_msgs::Pose Marker_Abs_Pose); //extraer vecindad
    //const Nodes Get_Nodes(){return nodes;}
    void Load_Nodes(Nodes nds, Vicinity vdr)
    {
        NodesMtx.lock();
        nodes_cpy = nds;
        rrt_vicinity_copy=vdr;
        NodesMtx.unlock();
        NodesAvailable = true;
        return;
    } //Called by B loop
    void Charge_Nodes()
    {
        if (NodesAvailable)
        {   NodesMtx.lock();
            nodes = nodes_cpy;
            rrt_vicinity = rrt_vicinity_copy;
            NodesMtx.unlock();
            NodesAvailable = false;
            NodesCharged = true;
        }
        return;
    } //Called by A loop
    double eeff_min_height;
    const cv::Mat getImage_Ptraj() { return image_Ptraj; }
    double Distance(VectorDbl P0, VectorDbl P1);
    void Selection();
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
    Etraj Tr;
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
    TextStream *Text_Stream_TR;
    TextStream *Text_Stream_Path;

    float max_dimm;
    int MapSize;
    int HalfMapSize;
    double MapResolution;
    RobotState_ ugv_state;
    float ugv_state_factor;

    std::vector<VectorInt> ObstacleMap;
    std::vector<Position> Obstacle_Points;
};
} // namespace PredNs

#endif //PREDICTION
