#ifndef ED_PMOV
#define ED_PMOV

#include "trajectory_client.hpp"

struct PositionResults
{
    VectorDbl Position;
    VectorDbl Position_Only_T;
    int region;
};
template <typename T>
class BlockingQueue
{
private:
    mutex mutex_;
    queue<T> queue_;

public:
    bool pop(T &t)
    {
        bool state = false;
        this->mutex_.lock();
        if (this->queue_.size() > 0)
        {
            t = this->queue_.front();
            this->queue_.pop();
            state = true;
        }
        this->mutex_.unlock();
        return state;
    }
    void push(T value)
    {
        this->mutex_.lock();
        this->queue_.push(value);
        this->mutex_.unlock();
    }
    int size()
    {
        this->mutex_.lock();
        int size = this->queue_.size();
        this->mutex_.unlock();
        return size;
    }
    bool empty()
    {
        this->mutex_.lock();
        bool check = this->queue_.empty();
        this->mutex_.unlock();
        return check;
    }
};
class Arm_Joint_State
{
public:
    Arm_Joint_State(string mx_joints_state_topic_, string pro_joints_state_topic_) : mx_joints_state_topic(mx_joints_state_topic_),
                                                                                     pro_joints_state_topic(pro_joints_state_topic_)
    {
        sub_mx_joints_state = nh_arm_joint_state.subscribe(mx_joints_state_topic, 1, &Arm_Joint_State::Mx_Joints_State_Handler, this);
        sub_pro_joints_state = nh_arm_joint_state.subscribe(pro_joints_state_topic, 1, &Arm_Joint_State::Pro_Joints_State_Handler, this);
        arm_joints_state.resize(6);
        for (int i = 0; i < 6; i++)
        {
            arm_joints_state[i] = 0.0;
        }
    }
    void Mx_Joints_State_Handler(const sensor_msgs::JointState &joints_state);
    void Pro_Joints_State_Handler(const sensor_msgs::JointState &joints_state);

    std::vector<double> GetCurrentArmJoints();

    sensor_msgs::JointState Mx_Joints_State;
    sensor_msgs::JointState Pro_Joints_State;
    ros::NodeHandle nh_arm_joint_state;
    string mx_joints_state_topic;
    string pro_joints_state_topic;
    ros::Subscriber sub_mx_joints_state;
    ros::Subscriber sub_pro_joints_state;
    std::vector<double> arm_joints_state;
    std::mutex arm_measurements_mtx;
};
class Ed_Pmov
{

public:
    edArm_trajectory::FollowTrajectoryClient arm;
    moveit::planning_interface::MoveGroupInterface group;

    Ed_Pmov(bool load_joint_states_sub_) : group("pro_arm"),
                                           robot_model_loader("robot1/robot_description"),
                                           load_joint_states_sub(load_joint_states_sub_)
    {
        
        num_IK_requests = 5;
        index_ks = 0;
        // group.setPlannerId("RRTConnectkConfigDefault"); //PRMstarkConfigDefault---RRTConnectkConfigDefault--RRTkConfigDefault--PRMkConfigDefault--RRTstarkConfigDefault
        group.setGoalTolerance(0.00001);            //0.004
        group.setGoalOrientationTolerance(0.003); //0.008
        group.setPlanningTime(0.02);
        group.setEndEffectorLink("link_motor_mx282");

        kinematic_models_.push_back(robot_model_loader.getModel()); //First model for basic operation
        for (int ith = 1; ith <= num_IK_requests; ith++)
        {
            std::string description_name = "robot1/robot_description_aux";
            description_name += std::to_string(ith);
            robot_model_loader_aux.push_back( RobotModelLoader(description_name.c_str()));
            kinematic_models_.push_back(robot_model_loader_aux[ith-1].getModel());
        }

        for (int i = 0; i <= num_IK_requests; i++)
        {
            robot_state::RobotStatePtr kA(new robot_state::RobotState(kinematic_models_[i]));
            kinematic_states_.push_back(kA);
        }

        for (int i = 0; i <= num_IK_requests; i++)
        {
            kinematic_states_[i]->setToDefaultValues();
        }
        // joint_model_group = kinematic_model->getJointModelGroup("pro_arm");
        joint_model_groups_.push_back(kinematic_models_[0]->getJointModelGroup("pro_arm"));

        for (int i = 1; i <= num_IK_requests; i++)
        {
            string group_name = "pro_arm_aux";
            group_name += std::to_string(i).c_str();
            joint_model_groups_.push_back(kinematic_models_[i]->getJointModelGroup(group_name));
        }
        for (int ith = 1; ith <= num_IK_requests; ith++)
        {
            ComputeThread_CollisionCheck(ith);
        }
        Print("Computing Threads Created", computing_thread_.size());
        // joints_pub = nh_.advertise< control_msgs::FollowJointTrajectoryGoal>("/joints_data", 1);    //uncomment if necessary
        if (load_joint_states_sub)
        {
            ArmJointsState = new Arm_Joint_State("/dynamixel_workbench_mx/joint_states", "/dynamixel_ed_pro_control/joint_states");
       }
       group.setEndEffectorLink("link_motor_mx282");
       eeffLink="link_motor_mx282";
    }

    geometry_msgs::Pose getCurrentPose();
    std::vector<double> ReadCurrentJoints();
    std::vector<double> getCurrentJoints(){
        return currentJoints;
    }

    void CheckandFixPoseRequest(geometry_msgs::Pose &pose_req);
    bool ReqMovement_byJointsValues(VectorDbl);
    bool SendMovement_byJointsValues(VectorDbl);
    bool ReqMovement_byPose(geometry_msgs::Pose);
    bool ReqMovement_byPose_FIx_Orientation(geometry_msgs::Pose pose_req);

    control_msgs::FollowJointTrajectoryGoal Req_Joints_byPose_FIx_Orientation(geometry_msgs::Pose pose_req);

    bool Request_Movement_byJointsTrajectory(control_msgs::FollowJointTrajectoryGoal goal);
    bool ReqMovement_byPose_Moveit(geometry_msgs::Pose pose_req);

    bool Check_Collision_Indx(VectorDbl Position, int index);
    bool Check_Collision_TypeB(VectorDbl Position);

    void FeedCollisionCheck_Queue(VectorDbl point, VectorDbl eeff_point_traslation, int region);
    void ComputeThread_CollisionCheck(int);
    std::pair<bool, PositionResults> RetrieveResults();

    void PrintCurrentPose(std::string);
    void PrintCurrentJoints(std::string);
    void PrintPose(std::string, geometry_msgs::Pose);
    std::chrono::microseconds getDelayTime() { return delay_time; }
    void Sleep(std::chrono::microseconds);
    void tic();
    std::chrono::microseconds toc(); //regresa el tiempo transcurrido
    std::chrono::microseconds toc(std::chrono::time_point<std::chrono::high_resolution_clock>);
    bool SendInitialPose();
    void PrintModelInfo()
    {

        Print("Model frame");
        Print(kinematic_models_[0]->getModelFrame().c_str());
        Print("Joint Group Name");
        Print(joint_model_groups_[0]->getName().c_str());
        Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
        Eigen::MatrixXd jacobian;
        kinematic_states_[0]->getJacobian(joint_model_groups_[0],
                                          kinematic_states_[0]->getLinkModel(joint_model_groups_[0]->getLinkModelNames().back()),
                                          reference_point_position, jacobian);

        Print("Jacobian", jacobian(0, 0), jacobian(0, 1), jacobian(0, 2), jacobian(0, 3), jacobian(0, 4), jacobian(0, 5));
        Print("        ", jacobian(1, 0), jacobian(1, 1), jacobian(1, 2), jacobian(1, 3), jacobian(1, 4), jacobian(1, 5));
        Print("        ", jacobian(2, 0), jacobian(2, 1), jacobian(2, 2), jacobian(2, 3), jacobian(2, 4), jacobian(2, 5));
        Print("        ", jacobian(3, 0), jacobian(3, 1), jacobian(3, 2), jacobian(3, 3), jacobian(3, 4), jacobian(3, 5));
        Print("        ", jacobian(4, 0), jacobian(4, 1), jacobian(4, 2), jacobian(4, 3), jacobian(4, 4), jacobian(4, 5));
        Print("        ", jacobian(5, 0), jacobian(5, 1), jacobian(5, 2), jacobian(5, 3), jacobian(5, 4), jacobian(5, 5));
    }

    typedef robot_model_loader::RobotModelLoader RobotModelLoader; //si se pasa a publico o antes de public: produce un error
    typedef moveit::planning_interface::MoveGroupInterface::Plan GroupPlan;
    typedef moveit::planning_interface::MoveItErrorCode ErrorCode;
    typedef moveit_msgs::MoveItErrorCodes MoveitCodes;

    RobotModelLoader robot_model_loader;
    std::vector<RobotModelLoader> robot_model_loader_aux;

    Printer Print;
    std::vector<robot_model::RobotModelPtr> kinematic_models_;
    std::vector<robot_state::RobotStatePtr> kinematic_states_;
    std::vector<const robot_state::JointModelGroup *> joint_model_groups_;

    BlockingQueue<PositionResults> eeff_positions_input_queue;
    BlockingQueue<std::pair<bool, PositionResults>> eeff_positions_results;
    Arm_Joint_State *ArmJointsState;

private:
    std::chrono::microseconds delay_time;
    geometry_msgs::Pose currentPose;
    std::vector<double> currentJoints;
    ros::Publisher joints_pub;
    ros::NodeHandle nh_;
    std::mutex positions_mtx, results_mtx, process_mtx;
    std::vector<std::thread> computing_thread_;
    int index_ks;
    int num_IK_requests;
    GroupPlan my_plan;
    bool load_joint_states_sub;
    std::chrono::time_point<std::chrono::high_resolution_clock> tic_clock_time;
    string eeffLink ;
};

#endif
