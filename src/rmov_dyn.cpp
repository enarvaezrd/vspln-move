//#include "armfcn.h"
#include "rrtfunctions.h"



int main(int argc, char** argv)
{
    // Init the ROS node

    ros::init(argc, argv, "follow_joint_trajectory_client");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);


    //=======================MOVEIT GROUP=====================================================================================================================
    moveit::planning_interface::MoveGroup group("pro_arm");
    group.setPlannerId("RRTConnectkConfigDefault");//PRMstarkConfigDefault---RRTConnectkConfigDefault--RRTkConfigDefault--PRMkConfigDefault--RRTstarkConfigDefault
    group.setGoalTolerance(0.005);//0.004
    group.setGoalOrientationTolerance(0.005);//0.008
    group.setPlanningTime(0.1);
    //=========================================================================================================================================================
    katana_tutorials::FollowJointTrajectoryClient arm; //Creacion de objeto de clase para mover el brazo

    //=======================PARA CINEMATICA INVERSA:=========================================================================================================

   robot_model_loader::RobotModelLoader robot_model_loadera("robot1/robot_description");   //--

   robot_model::RobotModelPtr kinematic_model = robot_model_loadera.getModel();   //--
   robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model)); //--

    kinematic_state->setToDefaultValues(); // --
   joint_model_group = kinematic_model->getJointModelGroup("pro_arm");
     //=========================================================================================================================================================

    //=======================NODE SUSCRIPTORS==================================================================================================================
   // ros::Subscriber submark = nh.subscribe("/tag_detections_pose", 1, Mark_Handler);  //Marker pose


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
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan); //Llamada de Planificacion en move_group
    std::cout<<success<<std::endl;
    if (success==1)
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
    float alturap=0.38;//0.21
    geometry_msgs::Pose currentPose=group.getCurrentPose().pose;
    geometry_msgs::Pose target_pose1=currentPose;
    std::cout<<"Orientaciones"<<target_pose1.orientation.w<<" x "<<target_pose1.orientation.x<<" y "<<target_pose1.orientation.y<<" z "<<target_pose1.orientation.z<<std::endl;
    std::cout<<"Posiciones"<<target_pose1.position.x<<" x "<<target_pose1.position.y<<" y "<<target_pose1.position.z<<" z "<<std::endl;



    target_pose1.position.x = 0.18;//0.1
    target_pose1.position.y = 0.0;
    target_pose1.position.z = alturap;

    bool fk = kinematic_state->setFromIK(joint_model_group, target_pose1, 1, 0.05);
   cout<<"fk paso"<<fk<<endl;
    std::vector<double> jv;
    control_msgs::FollowJointTrajectoryGoal goale;
    kinematic_state->copyJointGroupPositions(joint_model_group, jv);
    goale = arm.makeArmUpTrajectory(jv);
    arm.startTrajectory(goale);//Inicio de trayectoria en GAZEBO


    double time=0.0;long double t1;
    sleep(3.0);
    geometry_msgs::Pose arm_pose =  target_pose1,marker_pose;
    float xo=0.0,sx=0,sy=0, yo=0.0,x0=0, y0=0,x1=0,x2=0,x3=0,x4=0,y1=0,y2=0,y3=0,x5=0,y4=0,y5=0;


 while (ros::ok())
    {

    }




    return 0;
}
