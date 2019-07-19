#ifndef ED_PMOV
#define ED_PMOV

#include <ros/ros.h>

#include <thread>
#include <mutex>
#include <queue>
#include <vector>
#include <math.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/Pose.h>
using namespace std;

typedef vector<double> VectorDbl;
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
        this->mutex_.lock();
        bool state = false;
        if (!this->queue_.empty())
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
        int size = queue_.size();
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

void runThread(robot_state::RobotStatePtr &kinematic_stateA, const robot_state::JointModelGroup *&joint_model_groupA,
               mutex &positions_mtx, mutex &results_mtx, BlockingQueue<geometry_msgs::Pose> &eeff_positions_queue,
               BlockingQueue<std::pair<bool, geometry_msgs::Pose>> &eeff_positions_results, std::thread &th, int indexA)
{
    th = std::thread([&]() {
        int index_th = indexA;
        cout << "THREAD CREATION" << index_th << endl;
        geometry_msgs::Pose eeff_position;
        while (true)
        {

            //  cout<<"input queue size: "<<queue_size<<endl;
            bool state = eeff_positions_queue.pop(eeff_position);
            //Print("Calculating", index_th);
            if (state)
            {
                bool found_ik = kinematic_stateA->setFromIK(joint_model_groupA, eeff_position, 1, 0.005);

                std::pair<bool, geometry_msgs::Pose> result_N = std::make_pair(found_ik, eeff_position);
                eeff_positions_results.push(result_N);
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
        }
    });
}
class IK_Solver
{
public:
    IK_Solver() : robot_model_loader("robot1/robot_description")
    {

        kinematic_models_.push_back(robot_model_loader.getModel());
        kinematic_models_.push_back(robot_model_loader.getModel());
        kinematic_models_.push_back(robot_model_loader.getModel());

        robot_state::RobotStatePtr kinematic_stateT(new robot_state::RobotState(kinematic_models_[0]));
        robot_state::RobotStatePtr kinematic_state1T(new robot_state::RobotState(kinematic_models_[1]));
        robot_state::RobotStatePtr kinematic_state2T(new robot_state::RobotState(kinematic_models_[2]));

        kinematic_states_.push_back(kinematic_stateT);
        kinematic_states_.push_back(kinematic_state1T);
        kinematic_states_.push_back(kinematic_state2T);

        for(int i=0; i<3;i++)
            kinematic_states_[i]->setToDefaultValues();
        
        joint_model_groups_.push_back(kinematic_models_[0]->getJointModelGroup("pro_arm_aux1"));
        joint_model_groups_.push_back(kinematic_models_[1]->getJointModelGroup("pro_arm_aux2"));
        joint_model_groups_.push_back(kinematic_models_[2]->getJointModelGroup("pro_arm_aux3"));
    }
    void FeedCollisionCheck_Queue(geometry_msgs::Pose position)
    {
        //Print("feeding values",eeff_point.size(),eeff_point_traslation.size(),region);
        eeff_positions_queue.push(position);
    }
    void ComputeThread_CollisionCheck(int index)
    {
        computing_thread_.push_back(std::thread([&]() {
            int index_th = index;
            cout << "THREAD CREATION" << index_th << endl;
            geometry_msgs::Pose eeff_position;
            while (true)
            {
                bool state = eeff_positions_queue.pop(eeff_position);
                //Print("Calculating", index_th);
                if (state)
                {
                    bool found_ik = kinematic_states_[index_th]->setFromIK(joint_model_groups_[index_th], eeff_position, 1, 0.005);
                    std::pair<bool, geometry_msgs::Pose> result_N = std::make_pair(found_ik, eeff_position);
                    eeff_positions_results.push(result_N);
                    cout << "results pushed" << index_th << "state " << found_ik << "\n";
                }
                else
                {
                    std::this_thread::sleep_for(std::chrono::microseconds(1));
                }
            }
        }));
        std::this_thread::sleep_for(std::chrono::milliseconds(75));
        return;
    }

    std::pair<bool, geometry_msgs::Pose> RetrieveResults()
    {
        bool result_retrieved = false;
        std::pair<bool, geometry_msgs::Pose> result;
        result.first = false;
        while (!result_retrieved)
        {

            bool state = eeff_positions_results.pop(result);
            if (state)
            {
                result_retrieved = true;
                return result;
            }
            else
            {
                result_retrieved = false;
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
        }
        return result;
    }


    robot_model_loader::RobotModelLoader robot_model_loader;
    std::mutex positions_mtx, results_mtx;
    BlockingQueue<geometry_msgs::Pose> eeff_positions_queue;
    BlockingQueue<std::pair<bool, geometry_msgs::Pose>> eeff_positions_results;

    std::vector<std::thread> computing_thread_;
    std::vector<robot_model::RobotModelPtr> kinematic_models_;

    std::vector<robot_state::RobotStatePtr> kinematic_states_;

    std::vector<const robot_state::JointModelGroup *> joint_model_groups_;
};
#endif
