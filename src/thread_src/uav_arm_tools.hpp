#ifndef UAV_ARM_TOOLS
#define UAV_ARM_TOOLS

#include "rrt_functions.hpp"

typedef geometry_msgs::Pose Pose_msg;
typedef apriltag_ros::AprilTagDetectionArray AprilTagPose;
namespace ua_ns
{

struct PIDarm
{
    double ex;
    double ey;
    double ez;
    double time; //time in seconds
    double integralx;
    double integraly;
    double Kp;
    double Ki;
    double Kd;
};
struct Positions2D
{
    std::vector<double> x, y;
};

class ControllerCommands
{
public:
    ControllerCommands(string controller_topic)
    {
        sub_control_msg = nh_controller.subscribe(controller_topic.c_str(), 1, &ControllerCommands::Controller_Handler, this);
        docking_process = false;
        tracking_process = false;
        storage_process = false;
    }

    void Controller_Handler(const sensor_msgs::Joy &controller_msg);
    Printer Print;
    bool docking_process;
    bool tracking_process;
    bool storage_process;

private:
    ros::NodeHandle nh_controller;
    ros::Subscriber sub_control_msg;
};

class uav_arm_tools
{
public:
    PIDarm PIDdata;
    ControllerCommands Controller_Commands;
    int counter;
    map<int, Positions2D> marker_offsets;
    uav_arm_tools(float rad_int_, float rad_ext_, double minArmAltitude_,
                  string cntrl_topic_, double Docking_Alt_Lim_, float DockingFactor_,
                  bool real_robots_, map<int, Positions2D> marker_offsets_) : rad_ext(rad_ext_),
                                                                              rad_int(rad_int_), minArmAltitude(minArmAltitude_),
                                                                              Controller_Commands(cntrl_topic_),
                                                                              Docking_Altitude_Limit(Docking_Alt_Lim_),
                                                                              DockingFactor(DockingFactor_), real_robots(real_robots_),
                                                                              marker_offsets(marker_offsets_)

    {
        tracking_state_delayed = 0;
        PIDdata.ex = 0;
        PIDdata.ey = 0;
        PIDdata.ez = 0;
        PIDdata.time = 0;
        PIDdata.integralx = 0;
        PIDdata.integraly = 0;
        PIDdata.Kp = 0.04;
        PIDdata.Kd = 0.00;
        PIDdata.Ki = 0.00;
        counter = 0;
        minArm_Altitude_Limit = minArmAltitude;
        docking_has_been_requested_ = false;
        DockingAltitude = minArm_Altitude_Limit;

        // rad_ext = 0.45;
        //rad_int = 0.24;

        sub_UAVmark = nh_ua.subscribe("/tag_detections", 1, &uav_arm_tools::Marker_Handler, this);

        oldPos_ci.x.resize(6);
        oldPos_ci.y.resize(6);
        oldPos_ciFull.x.resize(6);
        oldPos_ciFull.y.resize(6);
        empty_pose.position.x = 0.0;
        empty_pose.position.y = 0.0;
        empty_pose.position.z = 0.0;
        empty_pose.orientation.x = 0.0;
        empty_pose.orientation.y = 0.0;
        empty_pose.orientation.z = 0.0;
        empty_pose.orientation.w = 0.0;
        for (int i = 0; i < 6; i++)
        {
            oldPos_ci.x[i] = 0;
            oldPos_ci.y[i] = 0;
        }
        oldPos_ciFull = oldPos_ci; //initialize in zeros
        armDelay = 0.0001;         //.035
        state = 0;
        DockingIteration = 0;
#ifdef STREAMING
        Text_Stream_eeff_uav_relative = new TextStream("/home/edd/catkin_ws/src/ed_pmov/data_eeff_uavrelativepose_v1.txt");
        Text_Stream_eeff_uav_relative->write_Data("x");
        Text_Stream_eeff_uav_relative->write_Data("y");
        Text_Stream_eeff_uav_relative->write_Data("z");
        Text_Stream_eeff_uav_relative->write_Data("yaw");
        Text_Stream_eeff_uav_relative->write_Data("roll");
        Text_Stream_eeff_uav_relative->write_Data("pitch");
        Text_Stream_eeff_uav_relative->write_Data("type");
        Text_Stream_eeff_uav_relative->write_Data("time");
        Text_Stream_eeff_uav_relative->write_Data("ms");
        Text_Stream_eeff_uav_relative->write_Data("delimiter");
        Text_Stream_eeff_uav_relative->write_TimeStamp();
#endif
        tracking_ok = false;
    }

    void Marker_Handler(const AprilTagPose &apriltag_marker_detections);

    void counter_addOne();
    void setArmPoseReq(Pose_msg Pose) { ArmPoseReq = Pose; }
    void setMinArmAltitude(float minalt) { minArmAltitude = minalt; }
    float getMinArmAltitude() { return minArmAltitude; }
    Pose_msg getArmPoseReq()
    {
        m_uatools.lock();
        Pose_msg res = ArmPoseReq;
        m_uatools.unlock();
        return res;
    }
    Pose_msg getArmPoseReqFull()
    {
        m_uatools.lock();
        Pose_msg res = ArmPoseReqFull;
        m_uatools.unlock();
        return res;
    }
    void setAltitudeRequest(float altitude)
    {
        //minArmAltitude = minArm_Altitude_Limit;
        ArmPoseReq.position.z = altitude;
        ArmPoseReqFull.position.z = altitude;
    }
    void CalculateDockingAltitude();
    double getEEFFAltitude() { return DockingAltitude; }

    struct Quat ArmOrientReq_toQuaternion(double, Pose_msg);
    struct Quat Angles_toQuaternion(double, double, double);
    struct Angles ConvPosetoAngles(Pose_msg);

    void UpdateArmCurrentPose(Pose_msg);

    Pose_msg Calc_LocalUAVPose();          //Local pose from camera to manip coord
    Pose_msg uavPose_to_ArmPoseReq_full(); //=100%
    Pose_msg uavPose_to_ArmPoseReq_arm();  //<100%
    void ArmPoseReq_decreaseAlt(float);

    void PID_Calculation(double &a, double &b);
    Pose_msg ExternalCircle_Corrections(Pose_msg);
    Pose_msg InnerCircle_Corrections(Pose_msg, Pose_msg, int);

    void Load_PID_time(double time)
    {
        PID_data_mtx.lock();
        PIDdata.time = time;
        PID_data_mtx.unlock();
        return;
    }
    float getArmDelay() { return armDelay; }
    Pose_msg getMarkerPose() { return marker_pose; }
    int getTrackingState() { return state; }

    void PIDReset();

private:
    Pose_msg empty_pose;
    deque<Pose_msg> marker_poses_;
    bool real_robots;
    ros::Subscriber sub_UAVmark; //Marker pose
    Printer Print;
    TextStream *Text_Stream_eeff_uav_relative;
    int state;
    float minArmAltitude;
    Pose_msg marker_pose;
    Pose_msg ArmPoseReq;
    Pose_msg ArmPoseReqFull;
    Pose_msg CurrentArmPose;
    Pose_msg OldArmPoseReq;
    Pose_msg OldArmPoseReqFull;
    float rad_int, rad_ext;
    float armDelay;
    ros::NodeHandle nh_ua;
    mutex m_uatools;
    float minArm_Altitude_Limit;
    std::mutex PID_data_mtx;
    Positions2D oldPos_ci;
    Positions2D oldPos_ciFull;
    NumberCorrection num;
    double DockingAltitude;
    int DockingIteration;
    double Docking_Altitude_Limit;
    double DockingFactor;
    bool tracking_ok;
    int tracking_state_delayed;
    bool docking_has_been_requested_;
};

} // namespace ua_ns

#endif
