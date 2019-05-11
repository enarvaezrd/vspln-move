#ifndef RRT_PLANIF
#define RRT_PLANIF

#include "prediction.hpp"

using namespace std;  
namespace rrt_planif
{
class RRT
{

public:

    RRT(){
        sequence_loop=false;
        image_size=800;
        d_prv = 5;      // profundidad de datos previos disponibles para prediccion
        d_pr_m = 3;     // datos previos a usar para calculo de mean values
        prof_expl = 13;  // Profundidad de exploracion  Esz=prof_f
        NumNodesToAdd = (prof_expl*1.5); //number of nodes to add in each region
        MaxOldNodesReg = NumNodesToAdd; // Max number of nodes to save
        image  = cv::Mat( image_size, image_size, CV_8UC3,cv::Scalar(255,255,255));
        image_Ptraj = cv::Mat( image_size, image_size, CV_8UC3 ,cv::Scalar(255,255,255));
        White_Imag = cv::Mat( image_size, image_size, CV_8UC3 ,cv::Scalar(255,255,255));

        nm = 70;
        pt = prof_expl;
        nodes_reordered=0;

        vdr.TP.resize(pt);
        vdr.R.resize(pt);
        //vdr.RP.resize(pt);
        vdr.angles.resize(pt);
        vdr.N.resize(pt);
        for (int i=0;i<pt/5;i++)
        {
            Colors.push_back(cv::Scalar(0,96,220));
            Colors.push_back(cv::Scalar(125,196,245));
            Colors.push_back(cv::Scalar(106,168,45));
            Colors.push_back(cv::Scalar(40,52,171));
            Colors.push_back(cv::Scalar(30,2,1));
            Colors.push_back(cv::Scalar(165,142,59));
            Colors.push_back(cv::Scalar(114,67,69));
         
        }
        for(int i=0;i<pt;i++)
        {
            Old_Nodes_Added_Reg.push_back(0);
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

        nodes.coord.resize(prof_expl); //longitud dinamica, empieza con el minimo
        nodes.coordT.resize(prof_expl);
        nodes.cost.resize(prof_expl); 
        nodes.costParent.resize(prof_expl); 
        nodes.parent.resize(prof_expl); 
        nodes.id.resize(prof_expl); 
        nodes.region.resize(prof_expl); 
        nodes.N=0; 
        for(int i=0;i<prof_expl;i++)
        {
            nodes.coord[i].resize(3);
            nodes.coordT[i].resize(3); 
        }
        r_exterior = 0.45;
        r_interior = 0.08;
        maxsc = 0.45;
        scale = floor(image_size/(2*maxsc));
        f_dist=0.1;
        
        finish =true;
        EmptyNodes.N=0;
        OldNodes=EmptyNodes;
        OldNodesLoaded=false;
        first_tr=true;
        Stretch_Extension=2;  //2 nodes 
        r=0.01  ;   //Radio de nodos cercanos Revisar  0.009 0.014
        EPS=0.004; //Maximo movimiento Revisar  0.005  0.007
        TrajNodesIncluded=2;
    }

    void Initialize_VicinityRRT();
    void Node_Filter();
    void Nodes_Reorder();

    void setEEFFMinHeight(double min_eef_alt) {eeff_min_height = min_eef_alt;}
    void delete_branch( int indx);
    void Add_Node(int It);

    void RRT_Generation();
    void RRT_AddValidCoord(VectorDbl, VectorDbl,int);
    void RRT_AddOldCoords();
    void RRT_Sequence(geometry_msgs::Pose Marker_Abs_Pose);

    cv::Mat getImage(){ return image;}
    const cv::Mat getImage_Ptraj(){ return image_Ptraj;}
   
    void Load_TR(const Etraj traj){Tr=traj;return;}
    void Load_Adv(int Adv){adv=Adv;return;}

    void Load_TRbr(const int trbr){tr_brk=trbr;return;}
    void       Initialize_Transf_Matrices(vector<VectorDbl > &Rpitch,vector<VectorDbl > &Rroll,vector<VectorDbl > &Ryaw, int &It);
    VectorDbl  Transform(VectorDbl Point, int It,vector<VectorDbl > &Rpitch,vector<VectorDbl > &Rroll,vector<VectorDbl > &Ryaw);
    VectorDbl  Translation(VectorDbl , int );
    VectorDbl  Matrix_Vector_MultiplyA(vector<VectorDbl > Matrix, VectorDbl Vector );
    VectorDbl  Angles_Calculation( VectorDbl P0,  VectorDbl P1);
    VectorDbl  Angles_Calculation( VectorDbl P0,  VectorDbl P1,  VectorDbl P2);
    double     Distance(VectorDbl P0, VectorDbl P1);
    bool       Check_Boundaries(VectorDbl Point);    
    void       Extract_Node_from_Nodes(Node &node, Nodes &nodes, int nIndx);
    VectorDbl  steer(VectorDbl qr,VectorDbl qn,double min_ndist,double EPS);
    void       Insert_Node_in_Nodes(Nodes &nodes,int nIndx, Node node);
    void       Push_Nodes_Elem_in_Nodes(Nodes &nodesR, int);
    bool getLoopState(){return sequence_loop;}
    void reset_nodes_reordered(){nodes_reordered=0;}

    void PrintNode(cv::Mat ,VectorDbl );
    void loop_start();
    void loop_end();
    //For Simulator=====================
    void tic();
    std::chrono::microseconds toc();
    std::chrono::time_point<std::chrono::high_resolution_clock>  tic_o();
    std::chrono::microseconds toc(std::chrono::time_point<std::chrono::high_resolution_clock> );

    bool Check_CollisionA(std::vector<double> , int );
    void SetArmPose(geometry_msgs::Pose Pose){CurrentRequest_Simm=Pose;return;}
    geometry_msgs::Pose GetArmPose(){return CurrentRequest_Simm;}
    void RRT_SequenceA(geometry_msgs::Pose Marker_Abs_Pose);
    void RRT_SequenceB();
   
    void Load_NdsReord(const int nds){nodes_reordered=nds;return;}
    const int Get_NdsReord(){return nodes_reordered;}
    void Load_Img(const cv::Mat img){image_Ptraj=img;return;}
    const bool get_finish(){return finish;}
    void ResetImagePtraj(){
        #ifdef OPENCV_DRAW 
        White_Imag.copyTo(image_Ptraj);White_Imag.copyTo(image);
        #endif
        return;}
    int Img(double point);
    double rad_to_deg(double rad);
    Nodes GetNodes(){return nodes;} //use carefully, at the end of sequence B
    void Stretch_the_Cord();
    void Draw_RRT();
    void Initialize_Inv_Transf_Matrices(vector<VectorDbl > &Rpitch,vector<VectorDbl > &Rroll,vector<VectorDbl > &Ryaw, int &It);
    VectorDbl Rotation(VectorDbl ,vector<VectorDbl > ,vector<VectorDbl > ,vector<VectorDbl > );
private:

    Etraj Tr;
    Vicinity vdr;
    Nodes nodes;
    Nodes OldNodes;
    Nodes EmptyNodes;
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
    int adv;
    float NumNodesToAdd;
    int nodes_reordered;
    int tr_brk;
    double eeff_min_height;
    double r_exterior;
    double r_interior;
    double maxsc;
    double scale;
    double f_dist;
    Printer Print;
    std::chrono::time_point<std::chrono::high_resolution_clock>  tic_clock_time;
    geometry_msgs::Pose CurrentRequest_Simm;
    bool finish;
    mutex mtxA;
    bool OldNodesLoaded;
    int MaxOldNodesReg;
    std::vector<cv::Scalar> Colors;
    int  first_tr;
    int Stretch_Extension;
    double r  ;   //Radio de nodos cercanos Revisar  0.009 0.014
    double EPS; //Maximo movimiento Revisar  0.005  0.007
    int TrajNodesIncluded;
    std::vector<int> Old_Nodes_Added_Reg;
};

}


#endif
