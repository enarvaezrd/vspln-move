#ifndef UAV_ARM_TOOLS
#define UAV_ARM_TOOLS

#include "rrt_functions.hpp"


typedef  geometry_msgs::Pose Pose_msg;
namespace ua_ns{
struct Quat {
    double x,y,z,w;
};
struct Angles {
    float yaw, roll, pitch;
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
struct Positions2D{
    std::vector<double> x,y;
};


class uav_arm_tools{
public:
    PIDarm PIDdata;
    int counter;

    uav_arm_tools()
    {
        PIDdata.ex = 0; PIDdata.ey = 0; PIDdata.ez = 0;
        PIDdata.time = 0;
        PIDdata.integralx = 0; PIDdata.integraly = 0;
        PIDdata.Kp = 0.25;
        PIDdata.Kd = 0.05;
        PIDdata.Ki = 0.04;
        counter = 0;
        rad_ext=0.4;
        rad_int=0.2;

        sub_UAVmark    = nh_ua.subscribe("/tag_detections_pose", 1, &uav_arm_tools::Marker_Handler, this);
        oldPos_ci.x.resize(6);
        oldPos_ci.y.resize(6);
        oldPos_ciFull.x.resize(6);
        oldPos_ciFull.y.resize(6);
        for (int i=0;i<6;i++)
        {
            oldPos_ci.x[i]=0;
            oldPos_ci.y[i]=0;
        }
        oldPos_ciFull = oldPos_ci;//initialize in zeros
        armDelay = 0.45;
        minArmAltitude = 0.42;
        state=0;
    }

    void Marker_Handler(const geometry_msgs::PoseArray& ma);

    void counter_addOne();
    void setArmPoseReq(Pose_msg Pose){ ArmPoseReq = Pose;}
    void setMinArmAltitude(float minalt){minArmAltitude=minalt;}
    float getMinArmAltitude(){return minArmAltitude;}
    Pose_msg getArmPoseReq(){m_uatools.lock(); Pose_msg res=ArmPoseReq;m_uatools.unlock(); return res;}
    Pose_msg getArmPoseReqFull(){m_uatools.lock(); Pose_msg res=ArmPoseReqFull;m_uatools.unlock(); return res;}
    void setAltitudeRequest(float altitude){ArmPoseReq.position.z=altitude; ArmPoseReqFull.position.z=altitude;}


    struct Quat ArmOrientReq_toQuaternion(double yaw_Mark, Pose_msg cpose);
    struct Quat Angles_toQuaternion(double pitch, double roll, double yaw);
    struct Angles ConvPosetoAngles(Pose_msg);

    void UpdateArmCurrentPose(Pose_msg);

    Pose_msg Calc_AbsoluteUAVPose();//Absolute pose from camera to manip coord
    Pose_msg uavPose_to_ArmPoseReq_full();//=100%
    Pose_msg uavPose_to_ArmPoseReq_arm();//<100%
    void ArmPoseReq_decreaseAlt(float dz_less);

    float getArmDelay(){return armDelay;}
    Pose_msg getMarkerPose(){return marker_pose;}
    int getTrackingState(){return state;}

    void PIDReset();
private:
    ros::Subscriber sub_UAVmark ;  //Marker pose
    Printer Print;
    int state;
    float minArmAltitude;
    Pose_msg marker_pose;
    Pose_msg ArmPoseReq;
    Pose_msg ArmPoseReqFull;
    Pose_msg CurrentArmPose;
    Pose_msg OldArmPoseReq;
    Pose_msg OldArmPoseReqFull;
    float rad_int,rad_ext;
    float armDelay;
    ros::NodeHandle nh_ua;
    mutex m_uatools;
    Positions2D oldPos_ci;
    Positions2D oldPos_ciFull;

};
}

#endif
