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

class uav_arm_tools
{
public:
    PIDarm PIDdata;
    int counter;

    uav_arm_tools(float rad_int_, float rad_ext_, double minArmAltitude_) : rad_ext(rad_ext_),
                                                                            rad_int(rad_int_), minArmAltitude(minArmAltitude_)

    {
        PIDdata.ex = 0;
        PIDdata.ey = 0;
        PIDdata.ez = 0;
        PIDdata.time = 0;
        PIDdata.integralx = 0;
        PIDdata.integraly = 0;
        PIDdata.Kp = 0.04;
        PIDdata.Kd = 0.001;
        PIDdata.Ki = 0.0000;
        counter = 0;
        minArm_Altitude_Limit = minArmAltitude;
        // rad_ext = 0.45;
        //rad_int = 0.24;

        sub_UAVmark = nh_ua.subscribe("/tag_detections", 1, &uav_arm_tools::Marker_Handler, this);

        oldPos_ci.x.resize(6);
        oldPos_ci.y.resize(6);
        oldPos_ciFull.x.resize(6);
        oldPos_ciFull.y.resize(6);
        for (int i = 0; i < 6; i++)
        {
            oldPos_ci.x[i] = 0;
            oldPos_ci.y[i] = 0;
        }
        oldPos_ciFull = oldPos_ci; //initialize in zeros
        armDelay = 0.02;           //.035
        state = 0;
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
        ArmPoseReq.position.z = altitude;
        ArmPoseReqFull.position.z = altitude;
    }

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

    float Load_PID_time(double time) { PIDdata.time = time; }
    float getArmDelay() { return armDelay; }
    Pose_msg getMarkerPose() { return marker_pose; }
    int getTrackingState() { return state; }

    void PIDReset();

private:
    ros::Subscriber sub_UAVmark; //Marker pose
    Printer Print;
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

    Positions2D oldPos_ci;
    Positions2D oldPos_ciFull;
    NumberCorrection num;
};
} // namespace ua_ns

#endif
