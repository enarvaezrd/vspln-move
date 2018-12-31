#include "armfcn.h"

const robot_state::JointModelGroup* joint_model_group ;
//Funciones generales
struct quat {
    double x,y,z,w;
};

struct oldPositions{
    std::vector<double> x,y;
};
struct PIDarm{
    double ex;
    double ey;
    double ez;
    double time;//time in seconds
    double integralx;
    double integraly;
    double Kp;
    double Ki;
    double Kd;
};
//Para PID de brazo
PIDarm PIDdata;


struct quat toQuaternion(double yaw_Mark, geometry_msgs::Pose cpose) //convertir a quaternion con modificaciones de roll y pitch fijos
{       quat qc;
        qc.x=cpose.orientation.x; qc.y=cpose.orientation.y;
        qc.z=cpose.orientation.z; qc.w=cpose.orientation.w;
        double ysqrc = qc.y * qc.y;
        double t3c = +2.0 * (qc.w * qc.z + qc.x * qc.y);
        double t4c = +1.0 - 2.0 * (ysqrc + qc.z * qc.z);
        double yaw_eeff = std::atan2(t3c, t4c);//end effector
        double roll;
        double  pitch;
            double diffyaw=0;
        int mico=1; //1 para mico, 0 para ed_arm 2 para dynamixel arm
            if (mico==1)
            {
               yaw_eeff-=PI/2;
                //cout<<"=======YawMark==== "<<yaw_Mark<<" =====YAW EEFF==== "<<yaw_eeff<<endl;

                /*if ( yaw_eeff >= 0 )  yaw_eeff =  yaw_eeff - PI;
                if ( yaw_eeff < 0 )   yaw_eeff =  PI + yaw_eeff;

                if ( yaw_eeff >= (PI) )  yaw_eeff =  yaw_eeff - 2*PI;
                if ( yaw_eeff < (-PI) )  yaw_eeff =  2*PI + yaw_eeff;

                if ( yaw_Mark >= 0 )   yaw_Mark =  (yaw_Mark - PI);//
                if ( yaw_Mark < 0 )    yaw_Mark =  (PI + yaw_Mark);
                cout<<"=======YawMark2==== "<<yaw_Mark<<" =====YAW EEFF2==== "<<yaw_eeff<<endl;

                diffyaw =(yaw_Mark-yaw_eeff)*1.0;
                yaw_eeff+=diffyaw;
                cout<<"Final yaw eeff"<<yaw_eeff<<" yaws diff: "<<diffyaw<<endl;
*/              if ( yaw_Mark >= 0.0 )  { yaw_Mark =  (yaw_Mark - PI);}//
                else{
                if ( yaw_Mark < 0.0 )    yaw_Mark =  (PI + yaw_Mark);}


                diffyaw =(yaw_Mark)*0.5;


                // cout<<"=======YawMark2==== "<<yaw_Mark<<" =====YAW EEFF2==== "<<yaw_eeff<<endl;

                yaw_eeff+=diffyaw;
              //  cout<<"=======Diffference ==== "<<diffyaw <<" =====YAW EEFF Final==== "<<yaw_eeff<<endl;

               // yaw_eeff=0;

                roll=0;//para mico 0, para ed_dual_arm -PI/2PhD Course
                pitch=PI;//para mico PI, para ed_dual_arm 0
            }
            else
            {
                if (mico==2)
                {

                    //cout<<"=======YawMarkOriginal==== "<<yaw_Mark;
                    yaw_Mark=yaw_eeff+(yaw_Mark);
                    //cout<<"=======YawMarkGLobal==== "<<yaw_Mark<<" =====YAW EEFF==== "<<yaw_eeff<<endl;
                    if (yaw_Mark>=(17*PI/15)) {yaw_Mark=yaw_Mark-(2*PI);}
                    else {
                        if (yaw_Mark<=(-17*PI/15)) yaw_Mark=yaw_Mark+(2*PI);
                    }
                    if (yaw_Mark>=(2.99*PI)) yaw_Mark=(2.99*PI);
                    if (yaw_Mark<=(-2.99*PI)) yaw_Mark=-(2.99*PI);
                    double diffpe=yaw_Mark-yaw_eeff;
                    double newyaw;
                    if (abs(diffpe)>PI) {newyaw=yaw_eeff+(1.0*diffpe);}
                    else {newyaw=yaw_eeff+(0.2*diffpe);}
                   yaw_eeff =0;//newyaw;

                   // cout<<"=======YawMarkGlobalFIxed==== "<<yaw_Mark<<" =====YAW EEFF==== "<<yaw_eeff<<endl;
                }
                else
                {
                    yaw_Mark=yaw_eeff+(yaw_Mark);
                    if (yaw_Mark>=(17*PI/15)) {yaw_Mark=yaw_Mark-(2*PI);}
                    else {
                        if (yaw_Mark<=(-17*PI/15)) yaw_Mark=yaw_Mark+(2*PI);
                    }
                    if (yaw_Mark>=(5*PI/6.01)) yaw_Mark=(5*PI/6.01);
                    if (yaw_Mark<=(-5*PI/6.01)) yaw_Mark=(-5*PI/6.01);
                    double diffpe=yaw_Mark-yaw_eeff;
                    double newyaw;
                    if (abs(diffpe)>PI) {newyaw=yaw_eeff+(1.0*diffpe);}
                    else {newyaw=yaw_eeff+(0.2*diffpe);}
                    yaw_eeff =newyaw;
                }

                roll=0;
                pitch=PI;
            }
        yaw_eeff-=PI/2 ;//Hay un offset siempre en el end effector


        quat q1;
        double t0 = std::cos(yaw_eeff   * 0.5);
        double t1 = std::sin(yaw_eeff   * 0.5);
        double t2 = std::cos(roll  * 0.5);
        double t3 = std::sin(roll  * 0.5);
        double t4 = std::cos(pitch * 0.5);
        double t5 = std::sin(pitch * 0.5);

        q1.w = t0 * t2 * t4 + t1 * t3 * t5;
        q1.x = t0 * t3 * t4 - t1 * t2 * t5;
        q1.y = t0 * t2 * t5 + t1 * t3 * t4;
        q1.z = t1 * t2 * t4 - t0 * t3 * t5;
        return q1;
}

struct quat ConvtoQuaternion(double pitch, double roll, double yaw)
{
        quat q1;
        double t0 = std::cos(yaw * 0.5);
        double t1 = std::sin(yaw * 0.5);
        double t2 = std::cos(roll * 0.5);
        double t3 = std::sin(roll * 0.5);
        double t4 = std::cos(pitch * 0.5);
        double t5 = std::sin(pitch * 0.5);
        q1.w = t0 * t2 * t4 + t1 * t3 * t5;
        q1.x = t0 * t3 * t4 - t1 * t2 * t5;
        q1.y = t0 * t2 * t5 + t1 * t3 * t4;
        q1.z = t1 * t2 * t4 - t0 * t3 * t5;
        return q1;
}

struct quat ConvtoAngles(geometry_msgs::Pose mpose) //Convertir quaternion de pose en angulos
{
    quat q1;
    quat q;
    q.x=mpose.orientation.x; q.y=mpose.orientation.y;
    q.z=mpose.orientation.z; q.w=mpose.orientation.w;
    double ysqr = q.y * q.y;
    // roll (x-axis rotation)
    double t00 = +2.0 * (q.w * q.x + q.y * q.z);
    double t11 = +1.0 - 2.0 * (q.x * q.x + ysqr);
    double roll = std::atan2(t00, t11);

    // pitch (y-axis rotation)
    double t22 = +2.0 * (q.w * q.y - q.z * q.x);
    t22 = t22 > 1.0 ? 1.0 : t22;
    t22 = t22 < -1.0 ? -1.0 : t22;
    double pitch = std::asin(t22);

    // yaw (z-axis rotation)
    double t33 = +2.0 * (q.w * q.z + q.x * q.y);
    double t44 = +1.0 - 2.0 * (ysqr + q.z * q.z);
    double yaw = std::atan2(t33, t44);
    q1.x=roll;q1.y=pitch;q1.z=yaw;
    return q1;
}

geometry_msgs::Pose markpose;
int state;

void Mark_Handler(const geometry_msgs::PoseArray& ma)
{
     state = ma.poses.size();
     if ( state ==1){
         markpose = ma.poses[0];
     }
     else{
         state=0;
     }
}

geometry_msgs::Pose uavPose_to_armPose(geometry_msgs::Pose UAVmarkPose, geometry_msgs::Pose ArmPose, geometry_msgs::Pose currentArmPose,double cont3,geometry_msgs::Pose oldPose,oldPositions &op, float tr, PIDarm &PIDdata ) //Funcion de convertir pose del UAV a movimiento del Brazo, contiene restricciones basicas de circulos interno y externo
{
    quat IAngleMark =ConvtoAngles(UAVmarkPose);
    oldPositions opT=op;
    quat quaternion=toQuaternion(IAngleMark.z,currentArmPose);
    quat poseCurrent =ConvtoAngles(currentArmPose);
    poseCurrent.z-=PI/2; //por el offset hay que hacer creer al sistema que la orientacion es esta
    //Transformacion en rotacion==========================================================================
    float xc1 = (markpose.position.x) * sin(poseCurrent.z) +  (markpose.position.y) * cos (poseCurrent.z);
    float yc1 = (markpose.position.x) * cos(poseCurrent.z) -  (markpose.position.y) * sin (poseCurrent.z);
    //====================================================================================================

    if (cont3>1)
    { //Envio de orientaciones cada determinado numero de iteraciones

        ArmPose.orientation.x= quaternion.x;
        ArmPose.orientation.y= quaternion.y;
        ArmPose.orientation.z= quaternion.z;
        ArmPose.orientation.w= quaternion.w;
    }

    float cx,cy,corg;
    corg = tr*0.009;//0.0065

    if (tr==1.0)
    {   //Desactiva PID y entrega calculo de posicion directa

        cx = tr*xc1;
        cy = tr*yc1;

        if (cx> corg) cx= corg;
        if (cx<-corg) cx=-corg;
        if (cy> corg) cy= corg;
        if (cy<-corg) cy=-corg;
    }
    else
    {   //PID implementation
        //Error calculation
        corg=0.15;
        double errorx = 0-xc1;
        double errory = 0-yc1;


        //Proportional terms
        double Poutx = PIDdata.Kp * errorx;
        double Pouty = PIDdata.Kp * errory;

        //Derivative terms
        double derx=0,dery=0;
        if (PIDdata.time !=0.0)
        {
        derx = (errorx-PIDdata.ex) / PIDdata.time;
        dery = (errory-PIDdata.ey) / PIDdata.time;
        }

        double Doutx = PIDdata.Kd * derx;
        double Douty = PIDdata.Kd * dery;

        //INTEGRAL terms
        PIDdata.integralx += errorx * PIDdata.time;
        PIDdata.integraly += errory * PIDdata.time;
        double Ioutx = PIDdata.Ki * PIDdata.integralx;
        double Iouty = PIDdata.Ki * PIDdata.integraly;

        double outputx = Poutx + Ioutx + Doutx;
        double outputy = Pouty + Iouty + Douty;
        // Restrict to max/min
            if( outputx >=  corg ) outputx = corg;
            if( outputx <= -corg ) outputx = -corg;

            if( outputy >=  corg ) outputy = corg;
            if( outputy <= -corg ) outputy = -corg;

            PIDdata.ex=errorx;
            PIDdata.ey=errory;
            cx=-outputx;
            cy=-outputy;
            //cout<<"errorx: "<<errorx<<", error y: "<<errory<<" output x: "<<cx<<" output y"<<outputy<<endl;
           // cx=0.0;
           //cy=0.0;

    }


    ArmPose.position.x += cx;
    ArmPose.position.y -= cy;



    float max=0.4;
    if (  ArmPose.position.x >= max)  ArmPose.position.x = max;//limitaciones rectangulares
    if (  ArmPose.position.x <= -max) ArmPose.position.x = -max;
    if (  ArmPose.position.y >= max)  ArmPose.position.y = max;
    if (  ArmPose.position.y <= -max) ArmPose.position.y = -max;

    double cat1, cat2, offx,offy;
    offx=0.0;
    offy=0.0;
    cat1= ArmPose.position.x-offx;
    cat2= ArmPose.position.y-offy;
    double rad=sqrt((cat1*cat1)+(cat2*cat2));

    double theta=0;
    double radext=0.4;//0.19 small dynamixel
    if (rad>=radext){//Circulo externo
        if(ArmPose.position.x<0){
            theta=PI+atan(ArmPose.position.y/ArmPose.position.x);
        }
        else{
            theta=atan(ArmPose.position.y/ArmPose.position.x);
        }
        ArmPose.position.y=radext*sin(theta);
        ArmPose.position.x=radext*cos(theta);
    }

    float sx=0,sy=0;
    double radint=0.09;
    if (rad<=radint){  //Circulo interno
        float xf=ArmPose.position.x, yf=ArmPose.position.y;
        float xo=oldPose.position.x, yo=oldPose.position.y;

        float dx=1.5*(xf-xo)/1;
        float dy=1.5*(yf-yo)/1, maxC=1.0;

        if ((xf>=0) && (-dx > maxC*xf)){
            dx=-maxC*xf/1;}
        else{

            if ((xf<0) && (-dx<maxC*xf))
            {dx=-maxC*xf/1;}
        }

        if ((yf>=0) && (-dy>maxC*yf)){
            dy=-maxC*yf/1;
        }
        else
        {
            if ((yf<0) && (-dy<maxC*yf))
           {dy=-maxC*yf/1;}
        }

        for (int i=0;i<5;i++)
        {
            op.x[i]=op.x[i+1];
            op.y[i]=op.y[i+1];
        }

        op.x[5]=dx;
        op.y[5]=dy;
        double dxa=(op.x[0]+op.x[1]+op.x[2]+op.x[3]+op.x[4]+op.x[5])/6;
        double dya=(op.y[0]+op.y[1]+op.y[2]+op.y[3]+op.y[4]+op.y[5])/6;
        dx=dxa;
        dy=dya;

        if ((xo>0)&&(xf<0)||(xo<0)&&(xf>0))
        {sx=xf-offx+(dx/30);}
        else
        {sx=xf-offx+dx;}
        if ((yo>0)&&(yf<0)||(yo<0)&&(yf>0))
        {sy=yf-offy+(dy/30);}
        else
        {sy=yf-offy+dy;}

       theta=std::atan2(sy,sx);

       double xf3=radint*cos(theta)+offx;
       double yf3=radint*sin(theta)+offy;

       double corrx=0.08*(xf3-currentArmPose.position.x);
       double corry=0.08*(yf3-currentArmPose.position.y);

       if (dx>0&&corrx<0) corrx=0;
       if (dx<0&&corrx>0) corrx=0;
       if (dy>0&&corry<0) corry=0;
       if (dy<0&&corry>0) corry=0;
       if (corrx>corg)  corrx= corg;
       if (corrx<-corg) corrx=-corg;
       if (corry>corg)  corry= corg;
       if (corry<-corg) corry=-corg;
       ArmPose.position.x += corrx ;
       ArmPose.position.y += corry ;
    }
    if(tr>0.3)
        op=opT;

    return ArmPose;
}

bool sendPose_ArmMov(geometry_msgs::Pose TPose,katana_tutorials::FollowJointTrajectoryClient &arm, robot_state::RobotStatePtr &kinematic_state,double &time1,ros::Publisher joints_pub)
{
    bool found_ik,found_ikO;
    geometry_msgs::Pose TPoseTemp;


    found_ik = kinematic_state->setFromIK(joint_model_group, TPose, 1, 0.01);

    ros::Duration tiempo_traj(0.0);

    if (found_ik)  //Cuando exista una solucion kinematica
    {
        std::vector<double> jv,jvO;
        control_msgs::FollowJointTrajectoryGoal goale;
        kinematic_state->copyJointGroupPositions(joint_model_group, jvO); //de jvO saco orientacion de ultima joint 6
        TPoseTemp=TPose;
        TPoseTemp.orientation.w = 0.0;
        TPoseTemp.orientation.x = 0.0;
        TPoseTemp.orientation.y =-1.0;
        TPoseTemp.orientation.z = 0.0;

        found_ikO = kinematic_state->setFromIK(joint_model_group, TPoseTemp, 1, 0.01);
        if (found_ikO)
        {

            kinematic_state->copyJointGroupPositions(joint_model_group, jv);  //de jv saco posicion de joints 0 a 5
             jvO[0] = jv[0];
             jvO[1] = jv[1];
             jvO[2] = jv[2];
             jvO[3] = jv[3];
             jvO[4] = jv[4];
             std::cout<<"last joint todo: "<<jvO[5]<<std::endl;
             std::cout<<"last joint simple: "<<jv[5]<<std::endl;
        }
        goale = arm.makeArmUpTrajectory(jvO);
        int numpoints6 = goale.trajectory.points.size()-1;//escoger el punto final ya que empieza desde 0
        tiempo_traj = goale.trajectory.points[numpoints6].time_from_start; //tiempo del punto final
        time1 = tiempo_traj.toSec();

        arm.startTrajectory(goale);//Inicio de trayectoria en GAZEBO
        //std::cout<<"DURACION DE TRAJECTORIA: "<<time1<<std::endl;
        //usleep(1000000*(time1+0.0)); //0.1 para robot real
        joints_pub.publish(goale);
    }
    else
    {
        std::cout<<"No se encuentra solucion kinematica!"<<std::endl;
    }
    return found_ik;
}

bool Check_Collision( std::vector<double> Position, int type, robot_state::RobotStatePtr &kinematic_state)
//type - 1 para solo posicion y 2 para posicion y orientacion juntas
{
    geometry_msgs::Pose CheckPose;
    cout<<" chequeo: 0"<<endl;
    CheckPose.position.x=Position[0];
    CheckPose.position.y=Position[1];
    CheckPose.position.z=Position[2];
    cout<<" chequeo: 1"<<endl;

    if (type==2) //cambiar a 3
    {
        cout<<" chequeo: tipo2  "<<endl;
        CheckPose.orientation.w=Position[3];
        CheckPose.orientation.x=Position[4];
        CheckPose.orientation.y=Position[5];
        CheckPose.orientation.z=Position[6];
    }
    else
    {
        cout<<" chequeo: tpo1   "<<endl;
        CheckPose.orientation.w=  0.0;
        CheckPose.orientation.x= -0.707096;
        CheckPose.orientation.y= -0.707096;
        CheckPose.orientation.z=  0.0;
    }

    cout<<kinematic_state->getRobotModel()<<endl;
    cout<<" chequeo: antes x: "<<CheckPose.position.x<<" y: "<<CheckPose.position.y<<"z "<<CheckPose.position.z<<" ox: "<<CheckPose.orientation.x<<" oy: "<<CheckPose.orientation.y<<" oz: "<<CheckPose.orientation.z<<" ow: "<<CheckPose.orientation.w<<endl;
  bool found_ik;
  found_ik= kinematic_state->setFromIK(joint_model_group, CheckPose, 1, 0.001);
cout<<" chequeo: "<<found_ik<<endl;
  //if (found_ik==1) cout<<"In check -- Found_? "<<found_ik<<endl;

return found_ik;

}




