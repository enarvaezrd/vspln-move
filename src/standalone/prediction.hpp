#ifndef PREDICITON
#define PREDICTION

#include "sincludes.hpp"

using namespace std;  
namespace PredNs{
class Prediction{

    public:
    Prediction(){
        
        image_size=800;
        d_prv = 5;      // profundidad de datos previos disponibles para prediccion
        d_pr_m = 3;     // datos previos a usar para calculo de mean values
        prof_expl = 10;  // Profundidad de exploracion  Esz=prof_f

        acum_values = 0;
        image_Ptraj = cv::Mat( image_size, image_size, CV_8UC3 ,cv::Scalar(255,255,255));
        White_Imag = cv::Mat( image_size, image_size, CV_8UC3 ,cv::Scalar(255,255,255));
        maxsc = 0.45;
        scale = floor(image_size/(2*maxsc));
        acum_x.resize((d_prv+1));
        acum_y.resize((d_prv+1));
        f_dist=0.1;
        for(int i=0;i<(d_prv);i++) {acum_x[i]=0.0;  acum_y[i]=0.0;} //inicializacion en ceros
        for (int i=0;i<prof_expl/5;i++)
            {
            Colors.push_back(cv::Scalar(0,96,220));
            Colors.push_back(cv::Scalar(125,196,245));
            Colors.push_back(cv::Scalar(106,168,45));
            Colors.push_back(cv::Scalar(40,52,171));
            Colors.push_back(cv::Scalar(30,2,1));
            Colors.push_back(cv::Scalar(165,142,59));
            Colors.push_back(cv::Scalar(114,67,69));    
        }
        NodesAvailable=false;
        first_tr=true;
    }
    
    
    void Trajectory_Prediction(geometry_msgs::Pose Marker_Abs_Pose);
    void Regression(VectorDbl x,VectorDbl y,int ndatos,int it,int order, VectorDbl &coeffs);
    void CheckandFix_Boundaries(VectorDbl  &x, VectorDbl  &y, int &prof_e);
    struct rrtns::MeanValues XYMean_Calculation(geometry_msgs::Pose Marker_Abs_Pose);
    
    const Etraj Get_TR(){return Tr;}
    const int Get_TRbr(){return tr_brk;}
    void Planif_SequenceA(geometry_msgs::Pose Marker_Abs_Pose);//extraer vecindad
    //const Nodes Get_Nodes(){return nodes;}
    void Load_Nodes(Nodes nds){ NodesMtx.lock();nodes_cpy = nds;NodesMtx.unlock();NodesAvailable=true; return;} //Called by B loop
    void Charge_Nodes(){  NodesMtx.lock(); nodes = nodes_cpy ;NodesMtx.unlock();  return;}  //Called by A loop
    double eeff_min_height;
    const cv::Mat getImage_Ptraj(){ return image_Ptraj;}
    private:
    struct rrtns::MeanValues mean;

    void Selection();

    VectorDbl acum_x;
    VectorDbl acum_y;
    int tr_brk;
    int d_prv;      // profundidad de datos previos disponibles para prediccion
    int d_pr_m;     // datos previos para el calculo de mean values
    int prof_expl;  // Profundidad de exploracion  Esz=prof_f

    double  acum_values;
    cv::Mat White_Imag ,image_Ptraj;
    Etraj Tr;
    Etraj Tr_old,Tr_temp;
std::mutex TP_Mtx;
    int image_size;
    double maxsc;
    double scale;
    double f_dist;

    std::vector<cv::Scalar> Colors;
    bool Stop_RRT_flag;
    Printer Print;
    int  first_tr;
    Nodes nodes_cpy, nodes;
    mutex NodesMtx;
    bool NodesAvailable;
    };
}

#endif //PREDICTION
