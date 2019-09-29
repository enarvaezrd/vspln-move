#ifndef TRAJ_CLIENT_CODE
#define TRAJ_CLIENT_CODE
#include "trajectory_client.hpp"

namespace edArm_trajectory
{
FollowTrajectoryClient::FollowTrajectoryClient() : traj_client_("/robot1/arm_controller/follow_joint_trajectory", true), got_joint_state_(false), spinner_(0)
{
  joint_names_.push_back("joint1");
  joint_names_.push_back("joint2");
  joint_names_.push_back("joint3");
  joint_names_.push_back("joint4");
  joint_names_.push_back("joint5");
  joint_names_.push_back("joint6");

  joint_state_sub_ = nh_.subscribe("/robot1/joint_states", 1, &FollowTrajectoryClient::jointStateCB, this);
  pub_trajectory_arm_command = nh_trajectory_client.advertise<control_msgs::FollowJointTrajectoryGoal>("/robot2/arm_general/goal_command", 1); //commands for the arm

  spinner_.start();

  // wait for action server to come up
  while (!traj_client_.waitForServer(ros::Duration(2.0)))
  {
    ROS_INFO("Waiting for the follow_joint_trajectory server, if this persist, something is wrong with the group inicialization ");
  }
}

FollowTrajectoryClient::~FollowTrajectoryClient()
{
}

void FollowTrajectoryClient::jointStateCB(const sensor_msgs::JointState::ConstPtr &msg)
{

  std::vector<double> ordered_js;
  ordered_js.resize(NUM_JOINTS);

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
void FollowTrajectoryClient::startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
{ // When to start the trajectory: 1s from now
/*
  int j = 0;
  bool big_diff = false;
  for (auto pt : goal.trajectory.points[1].positions)
  {
    if (abs(pt - current_joint_state_[j]) > 0.8 &&j<5)
      big_diff = true;
    j++;
  }
  if (big_diff)
  {
    Print("CURRENT JOINTS", current_joint_state_[0],current_joint_state_[1],
         current_joint_state_[2],current_joint_state_[3],
          current_joint_state_[4], current_joint_state_[5]);

    Print("GOAL JOINTS", goal.trajectory.points[1].positions[0], goal.trajectory.points[1].positions[1],
          goal.trajectory.points[1].positions[2], goal.trajectory.points[1].positions[3],
          goal.trajectory.points[1].positions[4], goal.trajectory.points[1].positions[5]);
  }*/

  goal.trajectory.header.stamp = ros::Time::now(); // + ros::Duration(0.1);
  traj_client_.sendGoal(goal);
  pub_trajectory_arm_command.publish(goal);
}

void FollowTrajectoryClient::boundValue(double &val, double maxv, double minv)
{
  if (val >= maxv)
    val = maxv;
  else if (val <= minv)
    val = minv;
  return;
}

control_msgs::FollowJointTrajectoryGoal FollowTrajectoryClient::makeArmUpTrajectory(std::vector<double> joints_obj)
{
  const size_t NUM_TRAJ_POINTS = 1;
  std::vector<double> req_positions(NUM_JOINTS);
  //std::vector<double> mid_positions(NUM_JOINTS);
  double maxval = 2.35619449;
  /*boundValue(joints_obj[1],maxval,-maxval );
        boundValue(joints_obj[2],maxval,-maxval );
        boundValue(joints_obj[4],maxval,-maxval );*/

  req_positions[0] = joints_obj[0];
  req_positions[1] = joints_obj[1];
  req_positions[2] = joints_obj[2];
  req_positions[3] = joints_obj[3];
  req_positions[4] = joints_obj[4];
  req_positions[5] = joints_obj[5];

  /*mid_positions[0] = (joints_obj[0]+current_joint_state_[0])/2;
        mid_positions[1] = (joints_obj[1]+current_joint_state_[1])/2;
        mid_positions[2] = (joints_obj[2]+current_joint_state_[2])/2;
        mid_positions[3] = (joints_obj[3]+current_joint_state_[3])/2;
        mid_positions[4] = (joints_obj[4]+current_joint_state_[4])/2;
        mid_positions[5] = (joints_obj[5]+current_joint_state_[5])/2;*/

  for (ros::Rate r = ros::Rate(20); !got_joint_state_; r.sleep())
  {
    ROS_DEBUG("waiting for joint state...");
    if (!ros::ok())
      exit(-1);
  }
  got_joint_state_ = false;
  double Wait_Time = wait_time_calc(req_positions);

  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names = joint_names_;
  trajectory.points.resize(NUM_TRAJ_POINTS);
  // trajectory point:0
  trajectory.points[0].time_from_start = ros::Duration(Wait_Time + 0.03);
  trajectory.points[0].positions.resize(NUM_JOINTS);
  trajectory.points[0].positions = req_positions; //siempre poner en primera posicion, para controlador de motores dynamixel
  // trajectory point:1
  /* trajectory.points[1].time_from_start = ros::Duration(Wait_Time + 0.03); //Ttimer quitar el 0.005
  trajectory.points[1].positions.resize(NUM_JOINTS);
  trajectory.points[1].positions = req_positions;

  trajectory.points[2].time_from_start = ros::Duration(Wait_Time+0.04+0.001); //Ttimer quitar el 0.02
        trajectory.points[2].positions.resize(NUM_JOINTS);
        trajectory.points[2].positions = req_positions;*/

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  return goal;
}

double FollowTrajectoryClient::wait_time_calc(std::vector<double> joints)
{
  double tempdiff = -100000000.0, tempdiff1 = 0.0; //diferencias de posicion entre links (temporales)
  int num_maxJoint = 0;
  for (int i = 0; i < 6; i++)
  {
    tempdiff1 = std::abs(joints[i] - current_joint_state_[i]); //distancia entre joints
    if (tempdiff1 > tempdiff)
    { //escoger la diferencia mas grande y su indice joint
      num_maxJoint = i;
      tempdiff = tempdiff1;
    }
  }
  double Ttimer;
  float JVelA, JVelB;

  //JVelA = ((30 * 2 * PI) / 60); //joints 1 2 3 velocity per second 30 rpm se usa lo maximo regresar a 60
  JVelB = ((35.0 * 2.0 * PI) / 60.0); //joints 4 5 6 velocity per second 35 rpm

  Ttimer = tempdiff / JVelB;
  if(Ttimer<0.0){
    Ttimer=0.001;
  }
  else if(Ttimer>1.0)
  {
    Ttimer=1.0;
  }
  return Ttimer;
}

//! Returns the current state of the action
actionlib::SimpleClientGoalState FollowTrajectoryClient::getState()
{
  return traj_client_.getState();
}

} // namespace edArm_trajectory

#endif
