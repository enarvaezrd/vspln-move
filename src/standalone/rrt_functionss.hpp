#ifndef RRT_PLANIF
#define RRT_PLANIF

#include "sincludes.hpp"


using namespace std;
typedef vector<double> VectorDbl;
namespace rrt_planif
{

struct Nodes{
   vector<VectorDbl >  coord;
   VectorDbl cost;
   vector<int >   parent;
   vector<int >   id;  //No usado por ahora
   int N;                   //Numero de nodos activos
};

struct Node{
     VectorDbl coord;
     double cost;
     int parent;
     int id;
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
struct MeanValues{
    double vx,vy,vz;
};

struct Vicinity{
   vector<VectorDbl >  TP;
   vector<vector<long double> >  R;
   //std::vector<std::vector<VectorDbl > > RP;
   vector<VectorDbl > angles;
   VectorDbl N;
   int L;
};


class RRT
{

public:
    
    RRT(){
        sequence_loop=false;
        image_size=800;
        d_prv = 5;      // profundidad de datos previos disponibles para prediccion
        d_pr_m = 3;     // datos previos a usar para calculo de mean values
        prof_expl = 10;  // Profundidad de exploracion  Esz=prof_f
        image  = cv::Mat( image_size, image_size, CV_8UC3,cv::Scalar(255,255,255) );
        image_Ptraj = cv::Mat( image_size, image_size, CV_8UC3 ,cv::Scalar(255,255,255));
        White_Imag = cv::Mat( image_size, image_size, CV_8UC3 ,cv::Scalar(255,255,255));
        acum_x.resize((d_prv+1));
        acum_y.resize((d_prv+1));

        for(int i=0;i<(d_prv);i++) {acum_x[i]=0.0;  acum_y[i]=0.0;} //inicializacion en ceros
        acum_values = 0;
        nm = 70;
        pt = prof_expl;
        nodes_reordered=0;

        vdr.TP.resize(pt);
        vdr.R.resize(pt);
        //vdr.RP.resize(pt);
        vdr.angles.resize(pt);
        vdr.N.resize(pt);
        for(int i=0;i<pt;i++)
        {
            vdr.TP[i].resize(7);//4 positions 4 orientations
            vdr.R[i].resize(3); //3 radius, each axis
            vdr.R[i][0] = 0.01;
            vdr.R[i][2] = 0.01;
            vdr.R[i][1] = (0.005+((i*i*1.0)/3000)); //fixed radius
            //vdr.RP[i].resize(1);
            //vdr.RP[i][1].resize(7);
            vdr.angles[i].resize(3);
            vdr.N[i]=0;
        }

        nodes.coord.resize(prof_expl);//longitud dinamica, empieza con el minimo
        nodes.cost.resize(prof_expl);
        nodes.parent.resize(prof_expl);
        nodes.id.resize(prof_expl);
        nodes.N=0;
        for(int i=0;i<prof_expl;i++)
        {
            nodes.coord[i].resize(3);
        }
        r_exterior = 0.45;
        r_interior = 0.08;
        maxsc = 0.45;
        scale = floor(image_size/(2*maxsc));
        f_dist=0.1;
        NumNodesToAdd = (prof_expl*1.0);
        finish =true;
    }

    void Trajectory_Prediction(geometry_msgs::Pose Marker_Abs_Pose);
    void Initialize_VicinityRRT();
    void Node_Filter();
    void Nodes_Reorder();

    void setEEFFMinHeight(double min_eef_alt) {eeff_min_height = min_eef_alt;}
    void Regression(VectorDbl x,VectorDbl y,int ndatos,int it,int order, VectorDbl &coeffs);
    void CheckandFix_Boundaries(VectorDbl  &x, VectorDbl  &y, int &prof_e);
    struct MeanValues XYMean_Calculation(geometry_msgs::Pose Marker_Abs_Pose);
    void delete_branch( int indx);
    void Add_Node(int It);

    void RRT_Generation();
    void RRT_Sequence(geometry_msgs::Pose Marker_Abs_Pose);

    cv::Mat getImage(){ return image;}
    const cv::Mat getImage_Ptraj(){ return image_Ptraj;}
   
    void       Initialize_Transf_Matrices(vector<VectorDbl > &Rpitch,vector<VectorDbl > &Rroll,vector<VectorDbl > &Ryaw, int &It);
    VectorDbl  Transform(VectorDbl Point, int It,vector<VectorDbl > &Rpitch,vector<VectorDbl > &Rroll,vector<VectorDbl > &Ryaw);
    VectorDbl  Matrix_Vector_MultiplyA(vector<VectorDbl > Matrix, VectorDbl Vector );
    VectorDbl  Angles_Calculation( VectorDbl P0,  VectorDbl P1);
    VectorDbl  Angles_Calculation( VectorDbl P0,  VectorDbl P1,  VectorDbl P2);
    double     Distance(VectorDbl P0, VectorDbl P1);
    bool       Check_Boundaries(VectorDbl Point);    
    void       Extract_Node_from_Nodes(Node &node, Nodes &nodes, int nIndx);
    VectorDbl  steer(VectorDbl qr,VectorDbl qn,double min_ndist,double EPS);
    void       Insert_Node_in_Nodes(Nodes &nodes,int nIndx, Node node);
    bool getLoopState(){return sequence_loop;}
    void reset_nodes_reordered(){nodes_reordered=0;}
    void PrintNode(cv::Mat ,VectorDbl );
    void loop_start();
    void loop_end();
    //For Simulator=====================
    void tic();
    long double toc(double tic_clock);
    long double toc();
    bool Check_CollisionA(std::vector<double> , int );
    void SetArmPose(geometry_msgs::Pose Pose){CurrentRequest_Simm=Pose;return;}
    geometry_msgs::Pose GetArmPose(){return CurrentRequest_Simm;}
    void RRT_SequenceA(geometry_msgs::Pose Marker_Abs_Pose);
    void RRT_SequenceB();
    void Load_TR(const Etraj traj){Tr=traj;return;}
    const Etraj Get_TR(){return Tr;}
    void Load_TRbr(const int trbr){tr_brk=trbr;return;}
    const int Get_TRbr(){return tr_brk;}
    void Load_NdsReord(const int nds){nodes_reordered=nds;return;}
    const int Get_NdsReord(){return nodes_reordered;}
    void Load_Img(const cv::Mat img){image_Ptraj=img;return;}
    const bool get_finish(){return finish;}
    void ResetImagePtraj(){White_Imag.copyTo(image_Ptraj);return;}
private:
    Etraj Tr;
    Etraj Tr_old,Tr_temp;
    Vicinity vdr;
    Nodes nodes;
    struct MeanValues mean;
    VectorDbl acum_x;
    VectorDbl acum_y;
    cv::Mat image ,image_Ptraj;
    cv::Mat White_Imag;
    int image_size;

    bool sequence_loop;
    int vicinities_init;
    int d_prv;      // profundidad de datos previos disponibles para prediccion
    int d_pr_m;     // datos previos para el calculo de mean values
    int prof_expl;  // Profundidad de exploracion  Esz=prof_f
    int nm;//numero maximo de muestras en cada region
    int pt;//Puntos de trayectoria Esz en matlab
    float NumNodesToAdd;
    int nodes_reordered;
    int tr_brk;
    double  acum_values;
    double eeff_min_height;
    double r_exterior;
    double r_interior;
    double maxsc;
    double scale;
    double f_dist;
    Printer Print;
    double tic_clock_time;
    geometry_msgs::Pose CurrentRequest_Simm;
    bool finish;
    mutex mtxA;
};

}


#endif
