#ifndef RRT_PLANIF
#define RRT_PLANIF
#include "prediction.hpp"

using namespace std;

namespace rrt_planif
{

class RRT
{

public:
    Ed_Pmov ArmModel;

    RRT(int image_size_, int d_prv_, int d_pr_m_, int prof_expl_, int max_dimm_, int NumNodesToAdd_, bool load_joint_states_sub, float EPS_) : ArmModel(load_joint_states_sub),

                                                                                                                                   d_prv(d_prv_), d_pr_m(d_pr_m_),
                                                                                                                                   prof_expl(prof_expl_), image_size(image_size_),
                                                                                                                                   NumNodesToAdd(NumNodesToAdd_),
                                                                                                                                   max_dimm(max_dimm_), EPS(EPS_)
    {
        sequence_loop = false;
        MaxOldNodesReg = NumNodesToAdd; // Max number of nodes to save
        image = cv::Mat(image_size, image_size, CV_8UC3, cv::Scalar(255, 255, 255));
        image_Ptraj = cv::Mat(image_size, image_size, CV_8UC3, cv::Scalar(255, 255, 255));
        White_Imag = cv::Mat(image_size, image_size, CV_8UC3, cv::Scalar(255, 255, 255));

        acum_values = 0;
        nm = 70;
        pt = prof_expl;
        nodes_reordered = 0;

        vdr.TP.resize(pt);
        vdr.R.resize(pt);
        //vdr.RP.resize(pt);
        vdr.angles.resize(pt);
        vdr.N.resize(pt);
        for (int i = 0; i < pt / 5; i++)
        {
            Colors.push_back(cv::Scalar(0, 96, 220));
            Colors.push_back(cv::Scalar(125, 196, 245));
            Colors.push_back(cv::Scalar(106, 168, 45));
            Colors.push_back(cv::Scalar(40, 52, 171));
            Colors.push_back(cv::Scalar(30, 2, 1));
            Colors.push_back(cv::Scalar(165, 142, 59));
            Colors.push_back(cv::Scalar(114, 67, 69));
        }
        for (int i = 0; i < pt; i++)
        {
            vdr.TP[i].resize(7); //4 positions 4 orientations
            vdr.R[i].resize(3);  //3 radius, each axis
            vdr.R[i][0] = 0.01;
            vdr.R[i][2] = 0.01;
            vdr.R[i][1] = (0.005 + ((i * i * 1.0) / 3000)); //fixed radius
            //vdr.RP[i].resize(1);
            //vdr.RP[i][1].resize(7);
            vdr.angles[i].resize(3);
            vdr.N[i] = 0;
        }

        nodes.coord.resize(prof_expl); //longitud dinamica, empieza con el minimo
        nodes.coordT.resize(prof_expl);
        nodes.cost.resize(prof_expl);
        nodes.costParent.resize(prof_expl);
        nodes.parent.resize(prof_expl);
        nodes.id.resize(prof_expl);
        nodes.region.resize(prof_expl);
        nodes.N = 0;
        for (int i = 0; i < prof_expl; i++)
        {
            nodes.coord[i].resize(3);
            nodes.coordT[i].resize(3);
        }

        r_exterior = 0.45;
        r_interior = 0.08;
        f_dist = 0.1;
        maxsc = max_dimm;
        scale = floor(image_size / (2.0 * maxsc));
        finish = true;
        TrajNodesIncluded = 2;
        EmptyNodes.N = 0;
        EmptyNodes.coord.resize(TrajNodesIncluded); //longitud dinamica, empieza con el minimo
        EmptyNodes.coordT.resize(TrajNodesIncluded);
        EmptyNodes.cost.resize(TrajNodesIncluded);
        EmptyNodes.costParent.resize(TrajNodesIncluded);
        EmptyNodes.parent.resize(TrajNodesIncluded);
        EmptyNodes.id.resize(TrajNodesIncluded);
        EmptyNodes.region.resize(TrajNodesIncluded);
        OldNodes = EmptyNodes;
        OldNodesLoaded = false;
        first_tr = false;
        Stop_RRT_flag = true;
        Stretch_Extension = 2; //2 nodes
        r = 0.01;              //Radio de nodos cercanos Revisar  0.009 0.014
#ifdef STREAMING
        Text_Stream = new TextStream("/home/edd/catkin_ws/src/ed_pmov/data_rrt.txt");
        Text_Stream->write_Data("x");
        Text_Stream->write_Data("y");
        Text_Stream->write_Data("z");
        Text_Stream->write_TimeStamp();
#endif
        emptyMatrix.resize(3);
        for (int i = 0; i < 3; i++)
        {
            emptyMatrix[i].resize(3);
            for (int j = 0; j < 3; j++)
            {
                emptyMatrix[i][j] = 0.0;
            }
        }
        for (int i = 0; i < prof_expl; i++)
        {
            Old_Nodes_Added_Reg.push_back(0);
        }
        Old_Positions.xval.resize(5);
        Old_Positions.yval.resize(5);
        Old_Positions.zval.resize(5);
    }

    void Trajectory_Prediction(geometry_msgs::Pose Marker_Abs_Pose);
    void Initialize_VicinityRRT();
    void Node_Filter();
    void Nodes_Reorder();

    void setEEFFMinHeight(double min_eef_alt) { eeff_min_height = min_eef_alt; }
    //void Regression(VectorDbl x,VectorDbl y,int ndatos,int it,int order, VectorDbl &coeffs);
    //void CheckandFix_Boundaries(VectorDbl  &x, VectorDbl  &y, int &prof_e);
    //struct MeanValues XYMean_Calculation(geometry_msgs::Pose Marker_Abs_Pose);
    void delete_branch(int);
    int Add_Node(int, int);

    void RetrieveNodes(int);

    void RRT_Generation();
    void RRT_AddValidCoord(VectorDbl, VectorDbl, int);
    void RRT_AddOldCoords();
    void RRT_SequenceB();
    //void RRT_SequenceB();

    cv::Mat getImage() { return image; }
    cv::Mat getImage_Ptraj() { return image_Ptraj; }
    void Load_TR(const Etraj traj)
    {
        Tr = traj;
        return;
    }
    void Load_Adv(int Adv)
    {
        adv = Adv;
        return;
    }
    void Load_TRbr(const int trbr)
    {
        tr_brk = trbr;
        return;
    }

    void Initialize_Transf_Matrices(vector<VectorDbl> &Rpitch, vector<VectorDbl> &Rroll, vector<VectorDbl> &Ryaw, int &It);
    VectorDbl Transform(VectorDbl Point, int It, vector<VectorDbl> &Rpitch, vector<VectorDbl> &Rroll, vector<VectorDbl> &Ryaw);
    VectorDbl Translation(VectorDbl, int);
    VectorDbl Matrix_Vector_MultiplyA(vector<VectorDbl> Matrix, VectorDbl Vector);
    VectorDbl Angles_Calculation(VectorDbl P0, VectorDbl P1);
    VectorDbl Angles_Calculation(VectorDbl P0, VectorDbl P1, VectorDbl P2);
    double Distance(VectorDbl P0, VectorDbl P1);
    bool Check_Boundaries(VectorDbl Point);
    void Extract_Node_from_Nodes(Node &node, Nodes nodes, int nIndx);
    VectorDbl steer(VectorDbl qr, VectorDbl qn, double min_ndist, double EPS);
    void Insert_Node_in_Nodes(Nodes &nodes, int nIndx, Node node);
    void Push_Nodes_Elem_in_Nodes(Nodes &nodesR, int);
    bool getLoopState() { return sequence_loop; }
    void reset_nodes_reordered() { nodes_reordered = 0; }
    control_msgs::FollowJointTrajectoryGoal SteerJoints(control_msgs::FollowJointTrajectoryGoal goal);

    void PrintNode(cv::Mat, VectorDbl);
    void loop_start();
    void loop_end();

    //void Load_ArmModel( Ed_Pmov *ArmMd){ArmModel=ArmMd;return;}
    //Ed_Pmov Get_ArmModel(){return ArmModel;}
    const Etraj Get_TR()
    {
        TP_Mtx.lock();
        Etraj TrT = Tr;
        TP_Mtx.unlock();
        return TrT;
    }
    const int Get_TRbr() { return tr_brk; }
    void Load_NdsReord(const int nds)
    {
        nodes_reordered = nds;
        return;
    }
    const int Get_NdsReord() { return nodes_reordered; }
    void Load_Img(const cv::Mat img)
    {
        image_Ptraj = img;
        return;
    }
    const bool get_finish() { return finish; }
    void ResetImagePtraj()
    {
#ifdef OPENCV_DRAW
        White_Imag.copyTo(image_Ptraj);
#endif
        return;
    }
    int Img(double point);
    double rad_to_deg(double rad);
    Nodes GetNodes() { return nodes; } //use carefully, at the end of sequence B

    void Load_UAV_Velocity(double vel) { UAV_Velocity = vel; }
    Vicinity GetVicinity() { return vdr; }
    void Draw_RRT();
    void Initialize_Inv_Transf_Matrices(vector<VectorDbl> &Rpitch, vector<VectorDbl> &Rroll, vector<VectorDbl> &Ryaw, int &It);

    VectorDbl Rotation(VectorDbl, vector<VectorDbl>, vector<VectorDbl>, vector<VectorDbl>);
    bool Stop_RRT_flag;
    int get_TR_Size() { return Tr.xval.size(); }

private:
    Etraj Tr;
    Etraj Tr_old, Tr_temp;
    Vicinity vdr;
    Nodes nodes;
    Nodes OldNodes;
    Nodes EmptyNodes;
    //struct MeanValues mean;
    cv::Mat image, image_Ptraj;
    cv::Mat White_Imag;
    int image_size;

    bool sequence_loop;
    int vicinities_init;
    int d_prv;     // profundidad de datos previos disponibles para prediccion
    int d_pr_m;    // datos previos para el calculo de mean values
    int prof_expl; // Profundidad de exploracion  Esz=prof_f
    int nm;        //numero maximo de muestras en cada region
    int pt;        //Puntos de trayectoria Esz en matlab
    float NumNodesToAdd;
    int nodes_reordered;
    int tr_brk;
    int adv;
    double acum_values;
    double eeff_min_height;
    double r_exterior;
    double r_interior;
    double maxsc;
    double scale;
    int max_dimm;
    double f_dist;
    Printer Print;
    std::chrono::time_point<std::chrono::high_resolution_clock> tic_clock_time;
    geometry_msgs::Pose CurrentRequest_Simm;
    bool finish;
    mutex mtxA;
    bool OldNodesLoaded;
    int MaxOldNodesReg;
    std::vector<cv::Scalar> Colors;
    std::mutex TP_Mtx;
    bool first_tr;
    int Stretch_Extension;
    double r;   //Radio de nodos cercanos Revisar  0.009 0.014
    float EPS; //Maximo movimiento Revisar  0.005  0.007
    int TrajNodesIncluded;
    std::vector<int> Old_Nodes_Added_Reg;
    TextStream *Text_Stream;
    vector<VectorDbl> emptyMatrix;
    double UAV_Velocity;
    Etraj Old_Positions;
};

} // namespace rrt_planif

#endif
