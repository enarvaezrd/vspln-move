#include "armbase.h"
double PI=3.141592654;
cv::Mat image = cv::Mat::zeros( 400, 400, CV_8UC3 );
cv::Mat image1 = cv::Mat::zeros( 400, 400, CV_8UC3 );
namespace katana_tutorials
{

FollowJointTrajectoryClient::FollowJointTrajectoryClient() :
    traj_client_("/robot1/arm_controller/follow_joint_trajectory", true), got_joint_state_(false), spinner_(0)
{
    joint_names_.push_back("joint1");
    joint_names_.push_back("joint2");
    joint_names_.push_back("joint3");
    joint_names_.push_back("joint4");
    joint_names_.push_back("joint5");
    joint_names_.push_back("joint6");


  joint_state_sub_ = nh_.subscribe("/robot1/joint_states", 1, &FollowJointTrajectoryClient::jointStateCB, this);
   spinner_.start();

  // wait for action server to come up
  while (!traj_client_.waitForServer(ros::Duration(2.0)))
  {
    ROS_INFO("Waiting for the follow_joint_trajectory server, if this persist, something is wrong with the group inicialization ");
  }
}

FollowJointTrajectoryClient::~FollowJointTrajectoryClient()
{
}

void FollowJointTrajectoryClient::jointStateCB(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> ordered_js;

  ordered_js.resize(joint_names_.size());

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    bool found = false;
    for (size_t j = 0; j < msg->name.size(); ++j)
    {
      if (joint_names_[i] == msg->name[j])
      {
        ordered_js[i] = msg->position[j];
        found = true;
        break;
      }
    }
    if (!found)
      return;
  }

  ROS_INFO_ONCE("Got joint state!");
  current_joint_state_ = ordered_js;
  got_joint_state_ = true;
}

//! Sends the command to start a given trajectory
void FollowJointTrajectoryClient::startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
{
  // When to start the trajectory: 1s from now
  goal.trajectory.header.stamp = ros::Time::now();// + ros::Duration(0.1);

  //std::cout<<goal;
  traj_client_.sendGoal(goal);
}



void boundValue(double& val, double maxv,double minv)
{
    if (val>=maxv)
        val=maxv;
    else if(val<=minv)
        val=minv;
}



control_msgs::FollowJointTrajectoryGoal FollowJointTrajectoryClient::makeArmUpTrajectory(std::vector<double> joints_obj)
{
  const size_t NUM_TRAJ_POINTS = 2;
  const size_t NUM_JOINTS = 6;

  // positions after calibration
  std::vector<double> mid_positions(NUM_JOINTS);
  double maxval=2.35619449;
  //std::cout<<"before"<<joints_obj[1]<<std::endl;
  boundValue(joints_obj[1],maxval,-maxval );
  boundValue(joints_obj[2],maxval,-maxval );
  boundValue(joints_obj[4],maxval,-maxval );
  mid_positions[0] = joints_obj[0];
  mid_positions[1] = joints_obj[1];
  mid_positions[2] = joints_obj[2];
  mid_positions[3] = joints_obj[3];
  mid_positions[4] = joints_obj[4];
  mid_positions[5] = joints_obj[5];

  // arm pointing straight up
  std::vector<double> straight_up_positions(NUM_JOINTS);
  straight_up_positions[0] = joints_obj[0];
  straight_up_positions[1] = joints_obj[1];
  straight_up_positions[2] = joints_obj[2];
  straight_up_positions[3] = joints_obj[3];
  straight_up_positions[4] = joints_obj[4];
  straight_up_positions[5] = joints_obj[5];


  /*mid_positions[0] = 0;
  mid_positions[1] = -4*PI/6;
  mid_positions[2] = -4*PI/6;
  mid_positions[3] = 0;
  mid_positions[4] = 0;
  mid_positions[5] = 0;*/


 // std::cout<<joints_obj[0]<<" "<<joints_obj[1]<<" "<<joints_obj[2]<<" "<<joints_obj[3]<<" "<<joints_obj[4]<<" "<<joints_obj[5]<<" "<<std::endl;


  trajectory_msgs::JointTrajectory trajectory;

  for (ros::Rate r = ros::Rate(10); !got_joint_state_; r.sleep())
  {
    ROS_DEBUG("waiting for joint state...");

    if (!ros::ok())
      exit(-1);
  }
  //promDiff=(std::abs(joints_obj[0]-current_joint_state_[0])+std::abs(joints_obj[1]-current_joint_state_[1])+std::abs(joints_obj[2]-current_joint_state_[2])+(3*std::abs(joints_obj[5]-current_joint_state_[5])))/4;

  double tempdiff=-1000,tempdiff1=0; //diferencias de posicion entre links (temporales)
  int num_maxJoint=0;
  for (int i=0;i<6;i++){
     tempdiff1= std::abs(joints_obj[i]-current_joint_state_[i]);//distancia entre joints
     if(tempdiff1>tempdiff) { //escoger la diferencia mas grande y su indice joint
         num_maxJoint=i;
         tempdiff=tempdiff1;
     }
  }
  double Ttimer;
  float JVelA, JVelB;

  int mico =0;
  if (mico==1)
  {
    JVelA=((12.2*2*PI)/10);//joints 1 2 3 velocity per second K75+   9.4-12.2 rpm se usa lo maximo regresar a 60
    JVelB=((20.3*2*PI)/10);//joints 4 5 6 velocity per second K58   15.0-20.3 rpm =====REVISAR, CAMBIAR 10 por 60 y revisar velocidades
  }
  else
  {
      JVelA=((59*2*PI)/60);//joints 1 2 3 velocity per second 59 rpm se usa lo maximo regresar a 60
      JVelB=((59*2*PI)/60);//joints 4 5 6 velocity per second 59 rpm
  }
  if (num_maxJoint==0||num_maxJoint==1||num_maxJoint==2)
  {
      Ttimer=tempdiff*1/JVelA;
  }
  else
  {
      Ttimer=tempdiff*1/JVelB;
  }
  //std::cout<<"Tiempo de Trayectoria: "<<Ttimer<<std::endl;
    int ind=0;
  // First, the joint names, which apply to all waypoints
  trajectory.joint_names = joint_names_;

  trajectory.points.resize(NUM_TRAJ_POINTS);

  // trajectory point:
  trajectory.points[ind].time_from_start = ros::Duration(0.01*(ind+1));
  trajectory.points[ind].positions = current_joint_state_;
  // trajectory point:
  ind++;
  trajectory.points[ind].time_from_start = ros::Duration(Ttimer+0.05); //Ttimer quitar el 0.02
  trajectory.points[ind].positions.resize(NUM_JOINTS);
  trajectory.points[ind].positions = mid_positions;
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;

  return goal;
}

//! Returns the current state of the action
actionlib::SimpleClientGoalState FollowJointTrajectoryClient::getState()
{
  return traj_client_.getState();
}

} /* namespace katana_tutorials */


