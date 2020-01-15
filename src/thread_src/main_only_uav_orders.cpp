#ifndef MAIN_APP
#define MAIN_APP
#define REAL_ROBOTS
#include "uav_arm_tools.hpp"

using namespace std;
int main(int argc, char **argv)
{
    //==============================CONFIGURATION PARAMETERS===========================

    int image_size = 600;

    int d_prv = 5;     // profundidad de datos previos disponibles para prediccion
    int d_pr_m = 3;    // datos previos a usar para calculo de mean values
    int prof_expl = 9; // Profundidad de exploracion  Esz=prof_f
    int Map_size = image_size;
    float max_dimm = 1.0;
    double rrt_extension = 0.1; //extension of each rrt step for regression 0.38 0.28
    int num_nodes_per_region = prof_expl + 15;
    double rad_int = 0.26;
    double rad_ext = 0.41;
    double armMinAltitude = 0.47;
    bool load_joints_state_sub, real_robots;
    std::string controller_topic = "/joy";
    double Docking_Alt_Lim_ = 0.83;
    double DockingFactor = 0.062;
    map<int, ua_ns::Positions2D> marker_offsets;
    ua_ns::Positions2D marker1;
    marker1.x = {0};
    marker1.y = {0};
    ua_ns::Positions2D marker2;
    marker2.x = {0};
    marker2.y = {0};
    marker_offsets.insert(std::pair<int, ua_ns::Positions2D>(0, marker1));
    marker_offsets.insert(std::pair<int, ua_ns::Positions2D>(2, marker2));
#ifdef REAL_ROBOTS
    load_joints_state_sub = true;
    string odom_str = "/robot1/odom"; ///robot1/robotnik_base_control/odom  ;   /robot1/odom
    string laser_topic = "/scan";     ///robot1/front_laser/scan SIMM ;  /scan REAL
    real_robots = true;
#else
    string odom_str = "/robot1/odom";                ///robot1/robotnik_base_control/odom  ;   /robot1/odom
    string laser_topic = "/robot1/front_laser/scan"; ///robot1/front_laser/scan SIMM ;  /scan REAL
    load_joints_state_sub = false;
    real_robots = false;
#endif

    double UAV_position_x = -0.18;
    double UAV_position_y = 0.22;
    //================================================================================================

    ros::init(argc, argv, "arm_program");
    ros::NodeHandle nodle_handle;
    ros::Rate loop_rate(30);
    Printer Print;

    RobotCommands Robot_Commands(odom_str, UAV_position_x, UAV_position_y);

    ua_ns::uav_arm_tools UavArm_tools(rad_int, rad_ext, armMinAltitude, controller_topic, Docking_Alt_Lim_, DockingFactor, real_robots, marker_offsets);
    sleep(3.0);
    auto image_MarkerARM_Pos = cv::Mat(image_size, image_size, CV_8UC3, cv::Scalar(255, 255, 255));
    int scale = floor(image_size / (2.0 * max_dimm));

    //==========================================CONTROL THREAD=========================================

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    ros::Rate loop_rateControl(30);
    double non_tracking_height_corr = 0.2;
    double y_correction = 0.0;
    auto timestamp = std::chrono::high_resolution_clock::now();
    while (ros::ok())
    {
        geometry_msgs::Pose LocalUAVPose;
        if (UavArm_tools.Controller_Commands.docking_process && UavArm_tools.Controller_Commands.tracking_process)
        {
            non_tracking_height_corr = -0.16;
        }
        if (UavArm_tools.getTrackingState() == 1 || UavArm_tools.getTrackingState() == 20)
        {
            LocalUAVPose = UavArm_tools.Calc_LocalUAVPose();
            Robot_Commands.Calculate_and_Send_Commands(LocalUAVPose, non_tracking_height_corr, y_correction);
            y_correction = 0.0;
        }
        auto toc_clock = std::chrono::high_resolution_clock::now();
        auto elapsed_c = std::chrono::duration_cast<std::chrono::microseconds>(toc_clock - timestamp);

        cout << "=====> SEQUENCE CONTROL TIME: " << elapsed_c.count() << endl;

        loop_rateControl.sleep();
        timestamp = std::chrono::high_resolution_clock::now();
    }
}
#endif
