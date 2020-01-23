#ifndef UAV_ARM_TOOLS_CODE
#define UAV_ARM_TOOLS_CODE
#include "uav_arm_tools.hpp"
using namespace ua_ns;

void uav_arm_tools::counter_addOne()
{
    if (counter > 2)
        counter = 0; //Enviar orientacion cada 2 pasos implementacion dentro de uavPose_to_armPose

    counter++;
}

void uav_arm_tools::Marker_Handler(const AprilTagPose &apriltag_marker_detections)
{
    state = apriltag_marker_detections.detections.size();
    Pose_msg Abs_pose;
    Abs_pose.position.x = 0.0;
    Abs_pose.position.y = 0.0;
    Pose_msg marker_pose_;
    if (state >= 1)
    {
        if (state > marker_offsets.size())
            ROS_ERROR("ERROR in size of marker_offsets uav_arm_tools.cpp %d", (int)marker_offsets.size());

        if (state > 1)
        {
            for (int i = 0; i < state; i++)
            {
                int pose_id = apriltag_marker_detections.detections[i].id[0];
                marker_pose_ = apriltag_marker_detections.detections[i].pose.pose.pose;
                marker_pose_.position.x += marker_offsets[pose_id].x[0];
                marker_pose_.position.y += marker_offsets[pose_id].y[0];

                Abs_pose.position.x += marker_pose_.position.x / (double)(state);
                Abs_pose.position.y += marker_pose_.position.y / (double)(state);
                //cout << "=====> MARKER POSE x: " << marker_pose_.position.x << ", y: " << marker_pose_.position.y << endl;
            }

            //cout << "=====> MARKERS POSE x: " << Abs_pose.position.x << ", y: " << Abs_pose.position.y << endl;
            marker_pose = marker_pose_;
            marker_pose.position.x = Abs_pose.position.x;
            marker_pose.position.y = Abs_pose.position.y;
        }
        else if (state == 1)
        {
            marker_pose = apriltag_marker_detections.detections[0].pose.pose.pose;
           // marker_pose.position.x /= 2;
           // marker_pose.position.y /= 2;
        }
    }
    else
    {
        state = 0;
    }
    return;
}

struct Quat uav_arm_tools::ArmOrientReq_toQuaternion(double yaw_Mark, geometry_msgs::Pose cpose) //convertir a quaternion con modificaciones de roll y pitch fijos
{
    Quat qc;
    qc.x = cpose.orientation.x;
    qc.y = cpose.orientation.y;
    qc.z = cpose.orientation.z;
    qc.w = cpose.orientation.w;
    double ysqrc = qc.y * qc.y;
    double t3c = +2.0 * (qc.w * qc.z + qc.x * qc.y);
    double t4c = +1.0 - 2.0 * (ysqrc + qc.z * qc.z);
    double yaw_eeff = std::atan2(t3c, t4c); //end effector
    double roll;
    double pitch;
    double diffyaw = 0;
    int mico = 1;
    if (mico == 1)
    {
        //yaw_eeff-=PI/2;
        /* if ( yaw_Mark >= 0.0 )  { yaw_Mark =  (yaw_Mark - PI);}//
    else{
        if ( yaw_Mark < 0.0 )    yaw_Mark =  (PI + yaw_Mark);
    }*/
        diffyaw = (yaw_Mark)*0.15;
        yaw_eeff += diffyaw;
    }

    if (mico == 2)
    {

        //cout<<"=======YawMarkOriginal==== "<<yaw_Mark;
        yaw_Mark = yaw_eeff + (yaw_Mark);
        //cout<<"=======YawMarkGLobal==== "<<yaw_Mark<<" =====YAW EEFF==== "<<yaw_eeff<<endl;
        if (yaw_Mark >= (17 * PI / 15))
        {
            yaw_Mark = yaw_Mark - (2 * PI);
        }
        else
        {
            if (yaw_Mark <= (-17 * PI / 15))
                yaw_Mark = yaw_Mark + (2 * PI);
        }
        if (yaw_Mark >= (2.99 * PI))
            yaw_Mark = (2.99 * PI);
        if (yaw_Mark <= (-2.99 * PI))
            yaw_Mark = -(2.99 * PI);
        double diffpe = yaw_Mark - yaw_eeff;
        double newyaw;
        if (abs(diffpe) > PI)
        {
            newyaw = yaw_eeff + (1.0 * diffpe);
        }
        else
        {
            newyaw = yaw_eeff + (0.2 * diffpe);
        }
        yaw_eeff = newyaw;

        // cout<<"=======YawMarkGlobalFIxed==== "<<yaw_Mark<<" =====YAW EEFF==== "<<yaw_eeff<<endl;
    }

    roll = -PI; //para mico 0, para ed_dual_arm -PI/2
    pitch = 0;  //para mico PI, para ed_dual_arm 0
    //yaw_eeff-=PI/2 ;//Hay un offset siempre en el end effector

    Quat q1;
    double t0 = std::cos(yaw_eeff * 0.5);
    double t1 = std::sin(yaw_eeff * 0.5);
    double t2 = std::cos(roll * 0.5);
    double t3 = std::sin(roll * 0.5);
    double t4 = std::cos(pitch * 0.5);
    double t5 = std::sin(pitch * 0.5);

    q1.w = t0 * t2 * t4 + t1 * t3 * t5;
    q1.x = t0 * t3 * t4 - t1 * t2 * t5;
    q1.y = t0 * t2 * t5 + t1 * t3 * t4;
    q1.z = t1 * t2 * t4 - t0 * t3 * t5;
    return q1;
}

struct Quat uav_arm_tools::Angles_toQuaternion(double pitch, double roll, double yaw)
{
    Quat q1;
    double t0 = std::cos(yaw * 0.5);
    double t1 = std::sin(yaw * 0.5);
    double t2 = std::cos(roll * 0.5);
    double t3 = std::sin(roll * 0.5);
    double t4 = std::cos(pitch * 0.5);
    double t5 = std::sin(pitch * 0.5);
    q1.w = t0 * t2 * t4 + t1 * t3 * t5;
    q1.x = t0 * t3 * t4 - t1 * t2 * t5;
    q1.y = t0 * t2 * t5 + t1 * t3 * t4;
    q1.z = t1 * t2 * t4 - t0 * t3 * t5;
    return q1;
}

struct Angles uav_arm_tools::ConvPosetoAngles(geometry_msgs::Pose pose) //Convertir quaternion de pose en angulos
{
    Angles angles;
    Quat q;
    q.x = pose.orientation.x;
    q.y = pose.orientation.y;
    q.z = pose.orientation.z;
    q.w = pose.orientation.w;
    double ysqr = q.y * q.y;
    // roll (x-axis rotation)
    double t00 = +2.0 * (q.w * q.x + q.y * q.z);
    double t11 = +1.0 - 2.0 * (q.x * q.x + ysqr);
    double roll = std::atan2(t00, t11);

    // pitch (y-axis rotation)
    double t22 = +2.0 * (q.w * q.y - q.z * q.x);
    t22 = t22 > 1.0 ? 1.0 : t22;
    t22 = t22 < -1.0 ? -1.0 : t22;
    double pitch = std::asin(t22);

    // yaw (z-axis rotation)
    double t33 = +2.0 * (q.w * q.z + q.x * q.y);
    double t44 = +1.0 - 2.0 * (ysqr + q.z * q.z);
    double yaw = std::atan2(t33, t44);
    angles.roll = roll;
    angles.pitch = pitch;
    angles.yaw = yaw;
    return angles;
}
geometry_msgs::Pose uav_arm_tools::uavPose_to_ArmPoseReq_full()
//Funcion de convertir 100% pose del UAV a movimiento del Brazo, contiene restricciones basicas de circulos interno y externo
{
    Angles IAngleMark = ConvPosetoAngles(marker_pose);
    // Positions2D opT=oldPos_ciFull;
    Quat quaternion = ArmOrientReq_toQuaternion(IAngleMark.yaw, CurrentArmPose);
    Angles AnglesCurrent = ConvPosetoAngles(CurrentArmPose);
    AnglesCurrent.yaw -= PI / 2; //por el offset hay que hacer creer al sistema que la orientacion es esta
    //Transformacion en rotacion================================================================================
    float xc1 = (marker_pose.position.x) * sin(AnglesCurrent.yaw) + (marker_pose.position.y) * cos(AnglesCurrent.yaw);
    float yc1 = (marker_pose.position.x) * cos(AnglesCurrent.yaw) - (marker_pose.position.y) * sin(AnglesCurrent.yaw);
    //==========================================================================================================

    if (counter > 1)
    { //Envio de orientaciones cada determinado numero de iteraciones

        ArmPoseReqFull.orientation.x = quaternion.x;
        ArmPoseReqFull.orientation.y = quaternion.y;
        ArmPoseReqFull.orientation.z = quaternion.z;
        ArmPoseReqFull.orientation.w = quaternion.w;
        //std::cout << "ORIENT REQ x "<<ArmPoseReqFull.orientation.x<<" y "<<ArmPoseReqFull.orientation.y<<" z "<<ArmPoseReqFull.orientation.z<<" w "<<ArmPoseReqFull.orientation.w<<std::endl;
    }

    float cx, cy, corg;
    corg = 0.005; //0.0065

    cx = xc1;
    cy = yc1;

    num.MinMax_Correction(cx, corg);
    num.MinMax_Correction(cy, corg);

    ArmPoseReqFull.position.x += cx;
    ArmPoseReqFull.position.y -= cy;

    float max_rectangle = 0.4;
    num.MinMax_Correction(ArmPoseReqFull.position.x, max_rectangle); //limitaciones rectangulares
    num.MinMax_Correction(ArmPoseReqFull.position.y, max_rectangle);

    double cat1, cat2, offx, offy;
    offx = 0.0;
    offy = 0.0;
    cat1 = ArmPoseReqFull.position.x - offx;
    cat2 = ArmPoseReqFull.position.y - offy;
    double rad = sqrt((cat1 * cat1) + (cat2 * cat2));

    double theta = 0;
    if (rad >= rad_ext)
    { //Circulo externo
        if (ArmPoseReqFull.position.x < 0)
        {
            theta = PI + atan(ArmPoseReqFull.position.y / ArmPoseReqFull.position.x);
        }
        else
        {
            theta = atan(ArmPoseReqFull.position.y / ArmPoseReqFull.position.x);
        }
        ArmPoseReqFull.position.y = rad_ext * sin(theta);
        ArmPoseReqFull.position.x = rad_ext * cos(theta);
    }

    if (rad <= rad_int)
    { //Circulo interno
        float sx = 0, sy = 0;
        float xf = ArmPoseReqFull.position.x, yf = ArmPoseReqFull.position.y;
        float xo = OldArmPoseReqFull.position.x, yo = OldArmPoseReqFull.position.y;

        float dx = 1.5 * (xf - xo) / 1;
        float dy = 1.5 * (yf - yo) / 1, maxC = 1.0;

        if ((xf >= 0) && (-dx > maxC * xf))
        {
            dx = -maxC * xf / 1;
        }
        else
        {

            if ((xf < 0) && (-dx < maxC * xf))
            {
                dx = -maxC * xf / 1;
            }
        }

        if ((yf >= 0) && (-dy > maxC * yf))
        {
            dy = -maxC * yf / 1;
        }
        else
        {
            if ((yf < 0) && (-dy < maxC * yf))
            {
                dy = -maxC * yf / 1;
            }
        }

        for (int i = 0; i < 5; i++)
        {
            oldPos_ciFull.x[i] = oldPos_ciFull.x[i + 1];
            oldPos_ciFull.y[i] = oldPos_ciFull.y[i + 1];
        }

        oldPos_ciFull.x[5] = dx;
        oldPos_ciFull.y[5] = dy;
        double dxa = (oldPos_ciFull.x[0] + oldPos_ciFull.x[1] + oldPos_ciFull.x[2] + oldPos_ciFull.x[3] + oldPos_ciFull.x[4] + oldPos_ciFull.x[5]) / 6;
        double dya = (oldPos_ciFull.y[0] + oldPos_ciFull.y[1] + oldPos_ciFull.y[2] + oldPos_ciFull.y[3] + oldPos_ciFull.y[4] + oldPos_ciFull.y[5]) / 6;
        dx = dxa;
        dy = dya;

        if ((xo > 0) && (xf < 0) || (xo < 0) && (xf > 0))
        {
            sx = xf - offx + (dx / 30);
        }
        else
        {
            sx = xf - offx + dx;
        }
        if ((yo > 0) && (yf < 0) || (yo < 0) && (yf > 0))
        {
            sy = yf - offy + (dy / 30);
        }
        else
        {
            sy = yf - offy + dy;
        }

        theta = std::atan2(sy, sx);

        double xf3 = rad_int * cos(theta) + offx;
        double yf3 = rad_int * sin(theta) + offy;

        double corrx = 0.1 * (xf3 - CurrentArmPose.position.x);
        double corry = 0.1 * (yf3 - CurrentArmPose.position.y);
        float corr_factor = 4.0;
        if (abs(dx) > 3 * abs(dy))
        {
            //corrx /= corr_factor;
            corry *= corr_factor;
        }
        else if (abs(dy) > 3 * abs(dx))
        {
            //corry /= corr_factor;
            corrx *= corr_factor;
        }

        if (abs(dy) + abs(dx) < corg / 3)
        {
            corrx = 0.0;
            corry = 0.0;
        }
        if (corrx > corg)
            corrx = corg;
        if (corrx < -corg)
            corrx = -corg;
        if (corry > corg)
            corry = corg;
        if (corry < -corg)
            corry = -corg;
        ArmPoseReqFull.position.x += corrx;
        ArmPoseReqFull.position.y += corry;
    }
    //oldPos_ciFull=opT;
    OldArmPoseReqFull = ArmPoseReqFull; //ya que se hace con el 100%

    return ArmPoseReqFull;
}
geometry_msgs::Pose uav_arm_tools::uavPose_to_ArmPoseReq_arm()
//Funcion de convertir pose del UAV a movimiento del Brazo, contiene restricciones basicas de circulos interno y externo
{
    Angles IAngleMark = ConvPosetoAngles(marker_pose);
    //Positions2D opT=oldPos_ci;
    //Quat quaternion = ArmOrientReq_toQuaternion(IAngleMark.yaw, CurrentArmPose);

    Angles AnglesCurrent = ConvPosetoAngles(CurrentArmPose);
    Quat quaternion = Angles_toQuaternion(0, -PI, AnglesCurrent.yaw + IAngleMark.yaw);
    //Print("|||| current arm angle, mark ", AnglesCurrent.yaw, IAngleMark.yaw);
    if (!real_robots)
        AnglesCurrent.yaw += PI / 2.0; //por el offset hay que hacer creer al sistema que la orientacion es esta
    //Transformacion en rotacion================================================================================
    double x_correction = (marker_pose.position.x) * sin(AnglesCurrent.yaw) + (marker_pose.position.y) * cos(AnglesCurrent.yaw);
    double y_correction = (marker_pose.position.x) * cos(AnglesCurrent.yaw) - (marker_pose.position.y) * sin(AnglesCurrent.yaw);
    //==========================================================================================================

    if (counter > 1)
    { //Envio de orientaciones cada determinado numero de iteraciones
        ArmPoseReq.orientation.x = quaternion.x;
        ArmPoseReq.orientation.y = quaternion.y;
        ArmPoseReq.orientation.z = quaternion.z;
        ArmPoseReq.orientation.w = quaternion.w;
        counter = 0;
    }

    float corg;
    corg = 0.008; //0.0065

    //PID implementation
    //Error calculation
    //corg=0.15;
    //num.MinMax_Correction(x_correction, 0.01);
    //num.MinMax_Correction(y_correction, 0.01);
    // cout << "Previous PID x: " << x_correction << ", y" << y_correction << endl;
    if (!Controller_Commands.tracking_process)
    {
        PIDReset();
    }
    else
    {
        PID_Calculation(x_correction, y_correction);
    }
    // cout << "After PID x: " << x_correction << ", y" << y_correction << endl;

 if (Controller_Commands.docking_process)
    {
        //Print("Docking ACTIVATED");
        num.MinMax_Correction(x_correction, 0.0011); //as we dont want large corrections in docking
        num.MinMax_Correction(y_correction, 0.0009);
    }
    else
    {
        num.MinMax_Correction(x_correction, 0.002); //small but larger for common tracking
        num.MinMax_Correction(y_correction, 0.002);
    }

    Pose_msg PID_ArmReq = ArmPoseReq;
    if (real_robots)
    {
        PID_ArmReq.position.x += x_correction;
        PID_ArmReq.position.y -= y_correction;
    }
    else
    {
        PID_ArmReq.position.x -= x_correction;
        PID_ArmReq.position.y += y_correction;
    }
    // cout << "PID x: " << PID_ArmReq.position.x << ", y" << PID_ArmReq.position.y << endl;

    //cout << "PID cor x: " << x_correction << ", y" << y_correction << endl;

    double cat1, cat2, offx(0.0), offy(0.0);
    cat1 = PID_ArmReq.position.x - offx;
    cat2 = PID_ArmReq.position.y - offy;
    double rad = sqrt((cat1 * cat1) + (cat2 * cat2));

    // double difference_x = ArmPoseReq.position.x - PID_ArmReq.position.x;
    //double difference_y = ArmPoseReq.position.y - PID_ArmReq.position.y;

    //------agregar flag de contacto-------

    if (rad <= (rad_int + 0.025))
    {
        minArmAltitude += 0.0055;
        num.MinMax_Correction(minArmAltitude, minArm_Altitude_Limit + 0.13);
        Print("ENter in Correction Z, rad. minalt",rad,minArmAltitude);
    }
    else
    {
        minArmAltitude -= 0.001;
        if (minArmAltitude <= minArm_Altitude_Limit)
            minArmAltitude = minArm_Altitude_Limit;
        num.MinMax_Correction(minArmAltitude, minArm_Altitude_Limit);
    }

    /* 
    if (rad >= (rad_ext - 0.01))
    {
        minArmAltitude -= 0.001;
        MinMax_Correction(minArmAltitude, 0.69);
    }
    else
    {
        minArmAltitude += 0.001;
        MinMax_Correction(minArmAltitude, 0.72);
    }
*/
    //-------------------------------------

    if (rad >= rad_ext)
    { //Circulo externo
        PID_ArmReq = ExternalCircle_Corrections(PID_ArmReq);
    }

    if (rad <= rad_int)
    { //Circulo interno
        PID_ArmReq = InnerCircle_Corrections(PID_ArmReq, OldArmPoseReq, corg);
    }
    float max_rectangle = 0.45;
    num.MinMax_Correction(PID_ArmReq.position.x, max_rectangle); //limitaciones rectangulares
    num.MinMax_Correction(PID_ArmReq.position.y, max_rectangle);
    //// difference_x = ArmPoseReq.position.x - PID_ArmReq.position.x;
    //difference_y = ArmPoseReq.position.y - PID_ArmReq.position.y;
    ArmPoseReq = PID_ArmReq;
    ArmPoseReqFull = ArmPoseReq;
    //  float factor = 0.005;
    // double full_x_correction = (ArmPoseReq.position.x - OldArmPoseReq.position.x);
    // double full_y_correction = (ArmPoseReq.position.y - OldArmPoseReq.position.y);
    // MinMax_Correction(full_x_correction, factor);
    //MinMax_Correction(full_y_correction, factor);

    //ArmPoseReq.position.x = OldArmPoseReq.position.x + full_x_correction;
    //ArmPoseReq.position.y = OldArmPoseReq.position.y + full_y_correction;

    OldArmPoseReq = ArmPoseReq;

    return ArmPoseReq;
}
void uav_arm_tools::PID_Calculation(double &x_correction, double &y_correction)
{

    double errorx = x_correction; //PID on correction
    double errory = y_correction;
    //Proportional terms

    PID_data_mtx.lock();
    double Poutx = PIDdata.Kp * errorx;
    double Pouty = PIDdata.Kp * errory;

    //Derivative terms
    double derx = 0.0, dery = 0.0;
    if (PIDdata.time != 0.0)
    {
        derx = (errorx - PIDdata.ex) / PIDdata.time;
        dery = (errory - PIDdata.ey) / PIDdata.time;
    }

    double Doutx = PIDdata.Kd * derx;
    double Douty = PIDdata.Kd * dery;

    //INTEGRAL terms
    PIDdata.integralx += errorx * PIDdata.time;
    PIDdata.integraly += errory * PIDdata.time;
    double Ioutx = PIDdata.Ki * PIDdata.integralx;
    double Iouty = PIDdata.Ki * PIDdata.integraly;
    //  num.MinMax_Correction(Ioutx, 0.05); //as we dont want large corrections
    //  num.MinMax_Correction(Iouty, 0.05);
    double pid_outputx = Poutx + Ioutx + Doutx;
    double pid_outputy = Pouty + Iouty + Douty;
    // Restrict to max/min
   
    PIDdata.ex = errorx; //old values for next iteration
    PIDdata.ey = errory;

    PID_data_mtx.unlock();
    x_correction = pid_outputx;
    y_correction = pid_outputy;
    return;
}

Pose_msg uav_arm_tools::ExternalCircle_Corrections(Pose_msg CurrentPoseReq)
{
    double theta = 0;
    if (CurrentPoseReq.position.x < 0.0)
    {
        theta = PI + atan(CurrentPoseReq.position.y / CurrentPoseReq.position.x);
    }
    else
    {
        theta = atan(CurrentPoseReq.position.y / CurrentPoseReq.position.x);
    }
    Pose_msg Result_PoseReq = CurrentPoseReq;
    Result_PoseReq.position.y = rad_ext * sin(theta);
    Result_PoseReq.position.x = rad_ext * cos(theta);
    return Result_PoseReq;
}
Pose_msg uav_arm_tools::InnerCircle_Corrections(Pose_msg CurrentPoseReq, Pose_msg OldPoseReq, int max_correction)
{
    double offx = 0.0, offy = 0.0;
    double theta = 0;
    float sx = 0, sy = 0, maxC = 1.0;
    float xf = CurrentPoseReq.position.x, yf = CurrentPoseReq.position.y;
    float xo = OldPoseReq.position.x, yo = OldPoseReq.position.y;
    float dx = 1.5 * (xf - xo);
    float dy = 1.5 * (yf - yo);

    if ((xf >= 0) && (-dx > maxC * xf))
    {
        dx = -maxC * xf;
    }
    else
    {

        if ((xf < 0) && (-dx < maxC * xf))
        {
            dx = -maxC * xf;
        }
    }

    if ((yf >= 0) && (-dy > maxC * yf))
    {
        dy = -maxC * yf;
    }
    else
    {
        if ((yf < 0) && (-dy < maxC * yf))
        {
            dy = -maxC * yf;
        }
    }

    for (int i = 0; i < 5; i++)
    {
        oldPos_ci.x[i] = oldPos_ci.x[i + 1];
        oldPos_ci.y[i] = oldPos_ci.y[i + 1];
    }

    oldPos_ci.x[5] = dx;
    oldPos_ci.y[5] = dy;
    double dxa = (oldPos_ci.x[0] + oldPos_ci.x[1] + oldPos_ci.x[2] + oldPos_ci.x[3] + oldPos_ci.x[4] + oldPos_ci.x[5]) / 6.0;
    double dya = (oldPos_ci.y[0] + oldPos_ci.y[1] + oldPos_ci.y[2] + oldPos_ci.y[3] + oldPos_ci.y[4] + oldPos_ci.y[5]) / 6.0;
    dx = dxa;
    dy = dya;

    if ((xo > 0) && (xf < 0) || (xo < 0) && (xf > 0))
    {
        sx = xf - offx + (dx / 30.0);
    }
    else
    {
        sx = xf - offx + dx;
    }
    if ((yo > 0) && (yf < 0) || (yo < 0) && (yf > 0))
    {
        sy = yf - offy + (dy / 30.0);
    }
    else
    {
        sy = yf - offy + dy;
    }

    theta = std::atan2(sy, sx);

    double xf3 = rad_int * cos(theta) + offx; //ideal
    double yf3 = rad_int * sin(theta) + offy;

    double corrx = 0.3 * (xf3 - CurrentPoseReq.position.x);
    double corry = 0.3 * (yf3 - CurrentPoseReq.position.y);

    float corr_factor = 4.0;
    if (abs(dx) > 3.0 * abs(dy))
    {
        //corrx /= corr_factor;
        corry *= corr_factor;
    }
    else if (abs(dy) > 3.0 * abs(dx))
    {
        //corry /= corr_factor;
        corrx *= corr_factor;
    }
    if (abs(dy) + abs(dx) < max_correction / 3.0)
    {
        corrx = 0.0;
        corry = 0.0;
    }
    //if (dx>0&&corrx<0) corrx=0.0;  //cual es la logica?
    // if (dx<0&&corrx>0) corrx=0.0;
    // if (dy>0&&corry<0) corry=0.0;
    //if (dy<0&&corry>0) corry=0.0;

    num.MinMax_Correction(corrx, max_correction);
    num.MinMax_Correction(corry, max_correction);

    Pose_msg Result_poseReq = CurrentPoseReq;
    Result_poseReq.position.x += corrx;
    Result_poseReq.position.y += corry;
    return Result_poseReq;
}
geometry_msgs::Pose uav_arm_tools::Calc_LocalUAVPose()
{
    geometry_msgs::Pose quad_pose;

    //Absolute pose from camera to manip coord mantaining the same z value or altitude
    Angles IAngleMark1 = ConvPosetoAngles(marker_pose); //UAV angles
                                                        // Quat quaternion1 = ArmOrientReq_toQuaternion(IAngleMark1.yaw, CurrentArmPose); //end effector + uav yaw

    //std::cout<<"x"<<currentPose.orientation.x<<" y "<<currentPose.orientation.y<<" z "<<currentPose.orientation.z<<" w "<<currentPose.orientation.w<<std::endl;
    Angles AnglesCurrent = ConvPosetoAngles(CurrentArmPose); //end effector angle
#ifdef SREAMING
    Text_Stream_eeff_uav_relative->write_Data(CurrentArmPose.position.x);
    Text_Stream_eeff_uav_relative->write_Data(CurrentArmPose.position.y);
    Text_Stream_eeff_uav_relative->write_Data(CurrentArmPose.position.z);
    Text_Stream_eeff_uav_relative->write_Data(AnglesCurrent.yaw);
    Text_Stream_eeff_uav_relative->write_Data(AnglesCurrent.roll);
    Text_Stream_eeff_uav_relative->write_Data(AnglesCurrent.pitch);
    Text_Stream_eeff_uav_relative->write_Data(1);
    Text_Stream_eeff_uav_relative->write_TimeStamp();
#endif
    Quat quaternion_angles = Angles_toQuaternion(0, -PI, AnglesCurrent.yaw + IAngleMark1.yaw);

    if (!real_robots)
        AnglesCurrent.yaw += PI / 2;
    float xc11 = (marker_pose.position.x) * sin(AnglesCurrent.yaw) + (marker_pose.position.y) * cos(AnglesCurrent.yaw);
    float yc11 = (marker_pose.position.x) * cos(AnglesCurrent.yaw) - (marker_pose.position.y) * sin(AnglesCurrent.yaw);

    quad_pose = CurrentArmPose;
    quad_pose.orientation.x = quaternion_angles.x;
    quad_pose.orientation.y = quaternion_angles.y;
    quad_pose.orientation.z = quaternion_angles.z;
    quad_pose.orientation.w = quaternion_angles.w;
    if (real_robots)
    {
        quad_pose.position.x += xc11;
        quad_pose.position.y -= yc11;
    }
    else
    {
        quad_pose.position.x -= xc11;
        quad_pose.position.y += yc11;
    }
#ifdef SREAMING
    Text_Stream_eeff_uav_relative->write_Data(quad_pose.position.x);
    Text_Stream_eeff_uav_relative->write_Data(quad_pose.position.y);
    Text_Stream_eeff_uav_relative->write_Data(quad_pose.position.z);
    Text_Stream_eeff_uav_relative->write_Data(AnglesCurrent.yaw + IAngleMark1.yaw);
    Text_Stream_eeff_uav_relative->write_Data(IAngleMark1.roll);
    Text_Stream_eeff_uav_relative->write_Data(IAngleMark1.pitch);
    Text_Stream_eeff_uav_relative->write_Data(2);
    Text_Stream_eeff_uav_relative->write_TimeStamp();
#endif
    //  cout << "ANGLES: uav: " << IAngleMark1.yaw << ", EEFF" << AnglesCurrent.yaw << " corrx: " << xc11 << ", corry: " << yc11 << endl;
    //   cout << "QUAD POSE: x" << quad_pose.position.x << ", y" << quad_pose.position.y << endl;
    //  cout << "EEFF POSE: x" << CurrentArmPose.position.x << ", y" << CurrentArmPose.position.y << endl;

    return quad_pose;
}
void uav_arm_tools::PIDReset()
{

    PID_data_mtx.lock();
    PIDdata.ex = 0.0;
    PIDdata.ey = 0.0;
    PIDdata.ez = 0.0;
   // PIDdata.time = 0.001;
    PIDdata.integralx = 0.0;
    PIDdata.integraly = 0.0;
    PID_data_mtx.unlock();
    return;
}

void uav_arm_tools::ArmPoseReq_decreaseAlt(float dz_less)
{
    ArmPoseReq.position.z -= dz_less;
    if (ArmPoseReq.position.z < minArmAltitude)
        ArmPoseReq.position.z = minArmAltitude;
    return;
}

void uav_arm_tools::UpdateArmCurrentPose(geometry_msgs::Pose currentpose)
{

    CurrentArmPose = currentpose;
    return;
}
void uav_arm_tools::CalculateDockingAltitude()
{
    if (Controller_Commands.docking_process && state >= 1)
    {
        DockingIteration += 1;

        DockingAltitude = minArm_Altitude_Limit + log2(1.0 + double(DockingIteration) / 25.0) * DockingFactor;

        if (DockingAltitude >= Docking_Altitude_Limit)
            DockingAltitude = Docking_Altitude_Limit;
        minArmAltitude = DockingAltitude;
        tracking_state_delayed = 0;
    }
    else
    {
        tracking_state_delayed++;
        if (tracking_state_delayed > 15)
        {
            Controller_Commands.docking_process = false;
            DockingIteration = 0;
            minArmAltitude -= 0.001;
            if (minArmAltitude <= minArm_Altitude_Limit)
                minArmAltitude = minArm_Altitude_Limit;
            num.MinMax_Correction(minArmAltitude, minArm_Altitude_Limit);

            DockingAltitude = minArmAltitude; //the one calculated in uavPose_to_ArmPoseReq_arm
        }
    }
    return;
}
void ControllerCommands::Controller_Handler(const sensor_msgs::Joy &Controller_Msg)
{
    sensor_msgs::Joy controller_msg = Controller_Msg;
#ifndef WIRELESS_CONTROLLER
    controller_msg.buttons[4] = controller_msg.buttons[3];
#endif
    if (controller_msg.buttons[4] == 1.0)
    {
        if (docking_process)
        {
            docking_process = false;
        }
        else
        {
            docking_process = true;
        }
    }
//cout<<"CONTROL"<<controller_msg.buttons[0]<<endl;
    if (controller_msg.buttons[0] == 1.0)
    {
        if (tracking_process)
        {
            tracking_process = false;
        }
        else
        {
            tracking_process = true;
        }
    }
}

#endif
