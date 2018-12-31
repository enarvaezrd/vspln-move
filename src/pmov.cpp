//#include "armfcn.h"
#include "rrtfunctions.h"



int main(int argc, char** argv)
{
    // Init the ROS node

    ros::init(argc, argv, "follow_joint_trajectory_client");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);


    //=======================MOVEIT GROUP=====================================================================================================================
    moveit::planning_interface::MoveGroupInterface group("pro_arm");
    group.setPlannerId("RRTConnectkConfigDefault");//PRMstarkConfigDefault---RRTConnectkConfigDefault--RRTkConfigDefault--PRMkConfigDefault--RRTstarkConfigDefault
    group.setGoalTolerance(0.004);//0.004
    group.setGoalOrientationTolerance(0.004);//0.008
    group.setPlanningTime(0.01);
    //=========================================================================================================================================================
    katana_tutorials::FollowJointTrajectoryClient arm; //Creacion de objeto de clase para mover el brazo



    //=======================PARA CINEMATICA INVERSA:=========================================================================================================

   robot_model_loader::RobotModelLoader robot_model_loadera("robot1/robot_description");   //--

   robot_model::RobotModelPtr kinematic_model = robot_model_loadera.getModel();   //--
   robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model)); //--  

   joint_model_group = kinematic_model->getJointModelGroup("pro_arm");
     //=========================================================================================================================================================

    //=======================NODE SUSCRIPTORS==================================================================================================================
    ros::Subscriber submark = nh.subscribe("/tag_detections_pose", 1, Mark_Handler);  //Marker pose

    ros::Publisher joints_pub=nh.advertise< control_msgs::FollowJointTrajectoryGoal>("/joints_data", 1);


    //=========================================================================================================================================================

    //=======================ENVIO DE PRIMERA TRAYECTORIA======================================================================================================

    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
    group_variable_values[0] = 0.0;
    group_variable_values[1] = -4*PI/6;
    group_variable_values[2] = -4*PI/6;
    group_variable_values[3] = 0.0;
    group_variable_values[4] = 0.0;
    group_variable_values[5] = 0.0;
    group.setJointValueTarget(group_variable_values);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan); //Llamada de Planificacion en move_group
    std::cout<<success<<std::endl;
    if (success==moveit_msgs::MoveItErrorCodes::SUCCESS)
        std::cout<<"Hay solucion"<<std::endl;
    else
        std::cout<<"NO hay solucion"<<std::endl;
    control_msgs::FollowJointTrajectoryGoal goaled;
    goaled = arm.makeArmUpTrajectory(group_variable_values);
    arm.startTrajectory(goaled);//Inicio de trayectoria en GAZEBO
    //=========================================================================================================================================================
   sleep(4.0);
    std_msgs::Int8 flmess;
    flmess.data=0;
    geometry_msgs::Pose quad_pose, endeff_pose;
    int state_ac=0,ac_angle=0;
    double cont3=0.0;
    float alturap=0.39;//0.21
    geometry_msgs::Pose currentPose=group.getCurrentPose().pose;
    geometry_msgs::Pose target_pose1=currentPose;
    std::cout<<"Orientaciones"<<target_pose1.orientation.w<<" x "<<target_pose1.orientation.x<<" y "<<target_pose1.orientation.y<<" z "<<target_pose1.orientation.z<<std::endl;
    std::cout<<"Posiciones"<<target_pose1.position.x<<" x "<<target_pose1.position.y<<" y "<<target_pose1.position.z<<" z "<<std::endl;


    target_pose1.position.x = 0.18;//0.1
    target_pose1.position.y = 0.0;
    target_pose1.position.z = alturap;

    bool fk = kinematic_state->setFromIK(joint_model_group, target_pose1, 1, 0.1);
   cout<<"fk paso"<<fk<<endl;
    std::vector<double> jv;
    control_msgs::FollowJointTrajectoryGoal goale;
    kinematic_state->copyJointGroupPositions(joint_model_group, jv);
    goale = arm.makeArmUpTrajectory(jv);
    arm.startTrajectory(goale);//Inicio de trayectoria en GAZEBO

    double time=0.0;long double t1;
    sleep(5.0);
    geometry_msgs::Pose arm_pose =  target_pose1,marker_pose;

     oldPositions op_ci;//oldPos_ci en uav_arm_tools
     op_ci.x.resize(6);
     op_ci.y.resize(6);
     for (int i=0;i<6;i++)
     {
         op_ci.x[i]=0;
         op_ci.y[i]=0;
     }
     //===========================================VALORES DE PID DEL BRAZO ROBOTICO============================================================================
     PIDdata.ex =0;PIDdata.ey=0;PIDdata.ez=0;PIDdata.time=0;PIDdata.integralx=0;PIDdata.integraly=0;
     PIDdata.Kp = 0.25;
     PIDdata.Kd = 0.05;
     PIDdata.Ki = 0.04;

    //============================================INICIALIZACION DE VARIABLES PARA MOVIMIENTO==================================================================

    int d_prv = 5;  // profundidad de datos previos para prediccion
    int d_prm = 2;  // datos previos para valores medios de diferencias
    int prof_e = 8; // Profundidad de exploracion  Esz=prof_f
    std::vector<double> acum_x(d_prv+1); std::vector<double> acum_y(d_prv+1);

    for(int i=0;i<(d_prv);i++) {acum_x[i]=0.0;  acum_y[i]=0.0;} //inicializacion en ceros

    double vx,vy, vcnt=0;
    int nm=70;//numero maximo de muestras en cada region
    int pt=prof_e;//Puntos de trayectoria Esz en matlab

    std::vector<double> xval, yval;
    int flag1=0; //Para conteo de inicializacion de trayectoria, solo para detener asignacion en la primera iteracion
    Etraj Tr_old;//Para realizar composicion de trayectoria


    Vicinity VD;
    VD.TP.resize(pt);
    VD.R.resize(pt);
    VD.RP.resize(pt);
    VD.angles.resize(pt);
    VD.N.resize(pt);
    for(int i=0;i<pt;i++)
    {
        VD.TP[i].resize(7);
        VD.R[i].resize(3);
        VD.R[i][0] = 0.0;VD.R[i][1] = 0.0;
        VD.R[i][2] = (0.005+((i*i*1.0)/3000));
        VD.RP[i].resize(nm);
        for (int j=0;j<nm;j++)
        {
            VD.RP[i][j].resize(7);
        }
        VD.angles[i].resize(3);
        VD.N[i]=0;
    }
   geometry_msgs::Pose oldPosem = markpose;
   PRMData PRM_Data;
    PRM_Data.Point = arm_pose;
    PRM_Data.ind_PRM_P.resize(pt);
    PRM_Data.ind_PRM_V.resize(pt);

    //=========================================================================================================================================================
    //==================================================INICIALIZACION DE VARIABLES PARA RRT===============================================================

     Nodes nodes;
     int maxnodes=prof_e;//Se decide hacerlo con longitud dinamica
     nodes.coord.resize(maxnodes);
     nodes.cost.resize(maxnodes);
     nodes.parent.resize(maxnodes);
     nodes.id.resize(maxnodes);
     nodes.N=0;
     for(int i=0;i<maxnodes;i++)
     {
         nodes.coord[i].resize(3);
     }
     Vicinity vdr;
     vdr.TP.resize(pt);
     vdr.R.resize(pt);
     vdr.RP.resize(pt);
     vdr.angles.resize(pt);
     vdr.N.resize(pt);
     for(int i=0;i<pt;i++)
     {
         vdr.TP[i].resize(7);
         vdr.R[i].resize(3);
         VD.R[i][0] = 0.0;VD.R[i][1] = 0.0;
         VD.R[i][2] = (0.005+((i*i*1.0)/3000));
         VD.RP[i].resize(1);

             VD.RP[i][1].resize(7);

         VD.angles[i].resize(3);
         VD.N[i]=0;
     }




    //=========================================================================================================================================================


 geometry_msgs::Pose oldPose=markpose;
 geometry_msgs::Pose PlanifPose=markpose;

 while (ros::ok())
    {
        prof_e = pt;
 image = cv::Mat::zeros( 400, 400, CV_8UC3 );
  image1 = cv::Mat::zeros( 400, 400, CV_8UC3 );
        clock_t t;
        t = clock();
        //clock_t tStart = clock(); //Time measurement

        flmess.data=0;
        if (state==1|| state==20) {     //Cuando el tracking este OK Calcular pose relativa de Quadrotor
           // cout<<"=====================================================MarkerPose=============="<<markpose.position.x<<" "<<markpose.position.y<<endl;
            quat IAngleMark1 =ConvtoAngles(markpose);
            geometry_msgs::Pose currentPose1=group.getCurrentPose().pose;
            endeff_pose = currentPose1;
            quat quaternion1=toQuaternion((IAngleMark1.z),currentPose1);//std::cout<<"x"<<currentPose.orientation.x<<" y "<<currentPose.orientation.y<<" z "<<currentPose.orientation.z<<" w "<<currentPose.orientation.w<<std::endl;
            quat poseCurrent1 =ConvtoAngles(currentPose1);
            float xc11 = (markpose.position.x) * sin(poseCurrent1.z) +  (markpose.position.y) * cos (poseCurrent1.z);
            float yc11 = (markpose.position.x) * cos(poseCurrent1.z) -  (markpose.position.y) * sin (poseCurrent1.z);
            quad_pose = arm_pose;
            quad_pose.orientation.x= quaternion1.x;
            quad_pose.orientation.y= quaternion1.y;
            quad_pose.orientation.z= quaternion1.z;
            quad_pose.orientation.w= quaternion1.w;
            quad_pose.position.x -= xc11;
            quad_pose.position.y += yc11;
            //pub2.publish(quad_pose); //Para controlar el Quad sin entrar en el control del brazo
            //pub3.publish(endeff_pose); //Para enviar informacion al nodo de acontrol,
            flmess.data=1;
            state_ac++;
            if (state_ac>=50) state_ac=50;
        }
        else {
            PIDdata.ex =0;PIDdata.ey=0;PIDdata.ez=0;PIDdata.time=0;PIDdata.integralx=0;PIDdata.integraly=0;
            state_ac=0;cont3=0; ac_angle=0; flag1=0;  //Para disminuir la altura en caso de que se pierda el tracking
            arm_pose.position.z -=0.02; //Disminucion progresiva
            if (arm_pose.position.z<=alturap) {arm_pose.position.z=alturap; } //limite inferior
            bool found_ik = kinematic_state->setFromIK(joint_model_group, arm_pose, 1, 1);
            std::vector<double> jv;
            control_msgs::FollowJointTrajectoryGoal goale;
            kinematic_state->copyJointGroupPositions(joint_model_group, jv);
            goale = arm.makeArmUpTrajectory(jv);
            arm.startTrajectory(goale);//Inicio de trayectoria en GAZEBO
            usleep(100000);flmess.data=0;
        }

      //  cout<<"Flag: "<<flag1<< " state: "<<state_ac<<endl;

        if (state_ac>=1){  //Cuando el Quad se haya detectado varias veces se continua al control del brazo

            cont3++;
            geometry_msgs::Pose currentArmPose=group.getCurrentPose().pose;
            if(flag1==0) PRM_Data.Point= currentArmPose;
            float angleM;
            marker_pose = uavPose_to_armPose(markpose, arm_pose, currentArmPose,cont3,oldPosem,op_ci,1.0, PIDdata);  //Transformacion de coordenadas y restricciones geometricas basicas 1.0 Para full scale, obtener posicion completa
            arm_pose    = uavPose_to_armPose(markpose, arm_pose, currentArmPose,cont3,oldPose,op_ci,0.55, PIDdata);  //0.2 para movimiento del brazo
            oldPosem = marker_pose;
            oldPose  = arm_pose;

            if (cont3>1) cont3=0; //Enviar orientacion cada 2 pasos implementacion dentro de uavPose_to_armPose

        //================================== ACUMULACION DE VALORES, CALCULO DE VARIACIONES EN X, Y ==================================================================
            vx=0.0;vy=0.0;

            for(int i=0;i<d_prv;i++)
            {
                acum_x[i]=acum_x[i+1];
                acum_y[i]=acum_y[i+1];
            }
            acum_x[d_prv]=marker_pose.position.x;  acum_y[d_prv]=marker_pose.position.y;

            if (vcnt != (d_prm+1))
                    vcnt++;
                else
                    vcnt=(d_prm+1);
            for(int i=(d_prv-d_prm+1);i<(d_prv);i++)
            {
                vx+=(acum_x[i]-acum_x[i-1]); //Acumulo todo el vector (diferencias)
                vy+=(acum_y[i]-acum_y[i-1]);
            }
            vx=1*vx/(vcnt-1); // Acumulacion sobre numero de datos　vx　es la variacion promedio en x
            vy=1*vy/(vcnt-1);
            //float maxdm=0.003;
          //  if (vy<=-maxdm) vy=-maxdm;if (vx<=-maxdm) vx=-maxdm;
           // if (vy>=maxdm) vy=maxdm;if (vy>=maxdm) vy=maxdm;old

           // cout<<"vx :: "<<vx<<"vy :: "<<vy<<endl;
        //=========================================================================================================================================================

        //===================================PREDICCION DE TRAYECTORIA=============================================================================================

        Delta_Prev Dv;
        Dv.dx=vx;
        Dv.dy=vy;
        Dv.dz=0.0;
        Point_c CurrentPoint;
        CurrentPoint.xvalc=marker_pose.position.x;  CurrentPoint.yvalc=marker_pose.position.y; CurrentPoint.zvalc=marker_pose.position.z;

        int tr_brk=0;
            Etraj tempTraj;
          tempTraj=Traj_Prediction( vcnt, d_prm, d_prv, prof_e, Dv, CurrentPoint,  acum_x, acum_y ,flag1,tr_brk,Tr_old,alturap);

           Tr_old =tempTraj;
            //cout<<"----------Prediccion Completa----------"<<endl;
        //=========================================================================================================================================================

        //=================================INICIALIZACION DE VECINDADES============================================================================================

         Initialize_VicinityRRT(vdr,prof_e,tempTraj);

         //cout<<"----------Vecindad Iniciada----------"<<endl;
         if (flag1==1)
             Node_Filter(nodes,vdr,prof_e,tr_brk);

         //ya ha entrado previamente
        // Initialize_Vicinity(VD,tr_brk, prof_e, tempTraj, kinematic_state);

        //=========================================================================================================================================================

        //==============================REORDENAMIENTO DE VECINDADES===============================================================================================

          Nodes_Reorder(nodes,vdr,prof_e,tr_brk,flag1);//a los primeros nodos, sweep a los puntos de trayectoria
          flag1=1;
          //Reorder_Vicinity(VD, prof_e, kinematic_state);
         //   cout<<"----------Nodos Reordenados----------"<<endl;

        //=========================================================================================================================================================

        //===============================PLANIFICACION=============================================================================================================

          RRT(nodes,vdr,prof_e,tr_brk, kinematic_state );
          //Planif_PRM(VD, PRM_Data);
      //   cout<<"----------RRT finalizado----------"<<endl;

        //=========================================================================================================================================================


          t = clock() - t;
          t1=t*1.0/CLOCKS_PER_SEC;//tiempo transcurrido en el codigo

          time =time-t1;//comparacion de tiempo de trajectoria time y tiempo de codigo t1
          if (time<=0.0) time=0.0;    //si el tiempo de codigo es mas grande que el de trajectoria
          usleep(1000000*(time+0.001));


         // bool found_ik = sendPose_ArmMov(PRM_Data.Point,arm,kinematic_state,time);
            bool found_ik = sendPose_ArmMov(arm_pose,arm,kinematic_state,time, joints_pub);  //sale el tiempo de trayectoria

          PIDdata.time=time+0.0;

          //std::cout<<"pausa"<<found_ik<<std::endl;
          //sleep(3);
          //arm_pose.position.y +=0.02;
           // found_ik = sendPose_ArmMov(arm_pose,joint_model_group,arm,kinematic_state);
         // std::cout<<"Salida"<<found_ik<<std::endl; const robot_state::JointModelGroup* joint_model_group, katana_tutorials::FollowJointTrajectoryClient &arm, robot_state::RobotStatePtr &kinematic_stateendl;

      }
        // Draw
        cv::circle( image, cv::Point( (marker_pose.position.x+maxsc)*scale,(marker_pose.position.y+maxsc)*scale ), 2, cv::Scalar( 00, 230, 50 ),  1, 8 );
        cv::circle( image, cv::Point( (arm_pose.position.x+maxsc)*scale,(arm_pose.position.y+maxsc)*scale ), 2, cv::Scalar( 00, 230, 50 ),  1, 8 );
        cv::imshow("Image",image); cv::imshow("Image1",image1);
        cv::waitKey(5);

          //std::cout<<"Orientaciones"<<arm_pose.orientation.w<<" x "<<arm_pose.orientation.x<<" y "<<arm_pose.orientation.y<<" z "<<arm_pose.orientation.z<<std::endl;

        loop_rate.sleep();
        ros::spinOnce();

     //  std::cout<<"==== TIEMPO DE SALIDA ======"<< t1<<"  clocks  "<<t<<"  TIEMPO  "<<CLOCKS_PER_SEC <<std::endl;
    }




    return 0;
}
