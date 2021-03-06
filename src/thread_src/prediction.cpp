#ifndef PREDICITON_CODE
#define PREDICTION_CODE

#include "prediction.hpp"

using namespace PredNs;

void Prediction::Trajectory_Prediction(geometry_msgs::Pose Marker_Abs_Pose, geometry_msgs::Pose CurrentEEFF_Pose, bool docking_process_flag)
{
    //1. vicinities_init - conteo de acumulaciones,
    //2. d_pr_m - profundidad de datos previos para acumulaciones,
    //3. d_prv -  profundidad de datos previos para prediccion
    //4. prof_expl - profundidad de valores a predecir,
    //5. Valores de Delta de distancias previas,
    //6. Current end effector point,
    //7.8. acumulaciones de Valores X Y de la trajectoria previa
    //9. flag1 - Bandera de primera entrada, indica creacion de primeras vecinadades y almacenamiento de anterior trayectoria
    //10. tr_brk - Trajectory break, punto de quiebre de trayectorias, indica en donde termina la anterior y empieza la nueva trayectoria. q_tr en matlab
    //11. tr_old - Anterior trayectoria almacenada, para comparar con la nueva tr, y realizar composicion de trayectoria

    int advance_prediction = 3;
    float delta_x_prev = 0.0;
    float delta_y_prev = 0.0;
    VectorDbl deltas_x_;
    VectorDbl deltas_y_;

    VectorDbl accum_x_copy = acum_x;
    VectorDbl accum_y_copy = acum_y;
    double num_points=4.0;
    acum_x[0] = CurrentEEFF_Pose.position.x + (1.0 * (Marker_Abs_Pose.position.x - CurrentEEFF_Pose.position.x) / num_points);
    acum_y[0] = CurrentEEFF_Pose.position.y + (1.0 * (Marker_Abs_Pose.position.y - CurrentEEFF_Pose.position.y) / num_points);
    acum_x[1] = CurrentEEFF_Pose.position.x + (2.0 * (Marker_Abs_Pose.position.x - CurrentEEFF_Pose.position.x) / num_points);
    acum_y[1] = CurrentEEFF_Pose.position.y + (2.0 * (Marker_Abs_Pose.position.y - CurrentEEFF_Pose.position.y) / num_points);
    acum_x[2] = CurrentEEFF_Pose.position.x + (3.0 * (Marker_Abs_Pose.position.x - CurrentEEFF_Pose.position.x) / num_points);
    acum_y[2] = CurrentEEFF_Pose.position.y + (3.0 * (Marker_Abs_Pose.position.y - CurrentEEFF_Pose.position.y) / num_points);

    double x_pre = acum_x[d_prv - 1];
    double y_pre = acum_y[d_prv - 1];
    int n = 2;

    /*for (double i = 1.0; i < d_prv - 1.0; i += 1.0)
    {
        acum_x[i] = CurrentEEFF_Pose.position.x + (i * (Marker_Abs_Pose.position.x - CurrentEEFF_Pose.position.x) / 4.0);
        acum_y[i] = CurrentEEFF_Pose.position.y + (i * (Marker_Abs_Pose.position.y - CurrentEEFF_Pose.position.x) / 4.0);
    }*/
    // acum_x[d_prv - 1] = (acum_x[d_prv - 1] + x_pre) / 2.0;
    //  acum_y[d_prv - 1] = (acum_y[d_prv - 1] + y_pre) / 2.0;
#ifdef OPENCV_DRAW

    for (int i = 0; i < d_prv; i++)
    {
        cv::circle(image_Ptraj, cv::Point(round((acum_x[i] + maxsc) * scale), round((acum_y[i] + maxsc) * scale)), 12, cv::Scalar(0, 0, 250), -1, 8);
    }
    cv::circle(image_Ptraj, cv::Point(round((Marker_Abs_Pose.position.x + maxsc) * scale), round((Marker_Abs_Pose.position.y + maxsc) * scale)), 4, cv::Scalar(0, 250, 250), -1, 8);
    // cv::circle(image_Ptraj, cv::Point(round((CurrentEEFF_Pose.position.x + maxsc) * scale), round((CurrentEEFF_Pose.position.y + maxsc) * scale)), 4, cv::Scalar(0, 250, 250), -1, 8);

#endif
    if (acum_values != (d_pr_m + 1))
        acum_values++;
    else
        acum_values = (d_pr_m + 1);

    //for(int i = (d_prv-d_pr_m+1);i<(d_prv);i++)
    for (int i = d_prv; i > (d_prv - d_pr_m); i--)
    {
        mean.vx += (acum_x[i] - acum_x[i - 1]); //Acumulo todo el vector (diferencias)
        mean.vy += (acum_y[i] - acum_y[i - 1]);
    }
    mean.vx /= (acum_values - 1); // Acumulacion sobre numero de datos　vx　es la variacion promedio en x
    mean.vy /= (acum_values - 1);

    for (int i = 0; i < acum_x.size() - 1; i++)
    {

        deltas_x_.push_back(acum_x[i + 1] - acum_x[i]);
        deltas_y_.push_back(acum_y[i + 1] - acum_y[i]);
    }
    deltas_x_.push_back(CurrentEEFF_Pose.position.x - acum_x.back());
    deltas_y_.push_back(CurrentEEFF_Pose.position.y - acum_y.back());
    for (int i = 0; i < deltas_x_.size() - 1; i++)
    {
        delta_x_prev += abs(abs(deltas_x_[i + 1]) - abs(deltas_x_[i]));
        delta_y_prev += abs(abs(deltas_y_[i + 1]) - abs(deltas_y_[i]));
    }
    if (delta_x_prev < 0.0015 || delta_y_prev < 0.0015 || abs(delta_y_prev - delta_y_prev) < 0.001)
    {
        n = 1;
        Print("Small deltas x y", delta_x_prev, delta_y_prev);
    }
    // Print("Deltas x y", delta_x_prev, delta_y_prev);
    Position CurrentPoint;
    CurrentPoint.xval = CurrentEEFF_Pose.position.x; //   CurrentEEFF_Pose    Marker_Abs_Pose
    CurrentPoint.yval = CurrentEEFF_Pose.position.y;
    CurrentPoint.zval = CurrentEEFF_Pose.position.z;

    tr_brk = 0;
    int prof_expl_adv = prof_expl + adv;
    std::vector<double> coeffs(n + 1);
    Etraj traj;
    traj.xval.resize(prof_expl_adv + 1);
    traj.yval.resize(prof_expl_adv + 1);
    traj.zval.resize(prof_expl_adv + 1);

    double fixed_dist = rrt_extension; //seria la distancia fija a la que se extiende la prediccion
    double zvalue = eeff_min_height;

    TP_Mtx.lock();
    if (acum_values < d_pr_m) //acum_values come from XYMean_Calculation
    {
        if (acum_values == 1)
        {
            for (int i = 0; i < prof_expl_adv; i++)
            {
                traj.xval[i] = CurrentPoint.xval;
                traj.yval[i] = CurrentPoint.yval; //asignar un solo punto xyz a toda la trayectoria, constante
                traj.zval[i] = zvalue;
            }
        }
        else
        {
            for (int i = 0; i < prof_expl_adv; i++)
            {
                traj.xval[i] = CurrentPoint.xval + (i * mean.vx / 2);
                traj.yval[i] = CurrentPoint.yval + (i * mean.vy / 2); //Prediccion totalmente lineal, no hay suficientes datos para calcular regresion
                traj.zval[i] = zvalue;                                //cambiar segun convenga habra que ingresar valor desde fuera de la funcion
            }
        }
        Tr = traj;
        tr_brk = prof_expl_adv;
    }
    else
    { //Cuando ya se pueda calcular regresion, es decir cuando ya se hayan acumulado muchos valores para mean.vx mean.vy

        double vx = mean.vx, vy = mean.vy;
        double vxtm = 0.0003, vytm = 0.0003;
        float maxdm = 0.001, pnd = 1.0;
        const int stepc = 400;
        std::vector<double> xvala(stepc), yvala(stepc);
        //xvala.resize(stepc); si compila estas lineas no son necesarias
        //yvala.resize(stepc);

        if (vy <= -maxdm)
            vy = -maxdm;
        if (vx <= -maxdm)
            vx = -maxdm;
        if (vy >= maxdm)
            vy = maxdm;
        if (vy >= maxdm)
            vy = maxdm;

        for (int i = 0; i < prof_expl_adv; i++)
        {
            //  traj.xval[i]=CurrentPoint.xvalc+(i*pnd*vx);  //creacion de estructuras basicas, para luego escoger que eje sera abcisa...
            // traj.yval[i]=CurrentPoint.yvalc+(i*pnd*vy);
            traj.zval[i] = zvalue;
        }
        double sationary_step_dist = 0.0016; //0.0016
        flagMtx.lock();
        // Print("Differences", abs(accum_x_copy[d_prv] - accum_x_copy[d_prv - 1]), abs(accum_y_copy[d_prv] - accum_y_copy[d_prv - 1]));
        if ((abs(accum_x_copy[d_prv] - accum_x_copy[d_prv - 1]) <= sationary_step_dist && abs(accum_y_copy[d_prv] - accum_y_copy[d_prv - 1]) <= sationary_step_dist) || docking_process_flag)
        {
            fixed_dist = 0.001; //0.001
            Stop_RRT_flag = true;
            Print("RRT stopped dist", fixed_dist);
        }
        else
        {
            fixed_dist = rrt_extension;
            Stop_RRT_flag = false;
        }
        flagMtx.unlock();
        //=======================================================================================================================================
        if (abs(mean.vx) > abs(mean.vy)) //Seleccion de modo, que eje es absisa y que eje es ordenadas , se escoge el que tenga mayou informacion, pasos mas grandes
        {
            Regression(acum_x, acum_y, d_prv, 1, n, coeffs);

            double signx = acum_x[d_prv] - acum_x[d_prv - 1];
            if (signx >= 0)
                vxtm = vxtm;
            else
                vxtm = -vxtm;

            for (int i = 0; i < stepc; i++)
                xvala[i] = CurrentPoint.xval + (i * vxtm); //disponer de muchos puntos

            int indxj = 1;
            for (int i = 0; i < stepc; i++) //Calculo de distancia junto con la regresion
            {
                yvala[i] = 0.0;
                for (int j = 0; j <= n; j++)
                    yvala[i] += (coeffs[j] * (pow(xvala[i], j)));

                double hipx = sqrt((pow((CurrentPoint.yval - yvala[i]), 2)) + (pow((CurrentPoint.xval - xvala[i]), 2)));
                if (hipx > fixed_dist)
                {
                    indxj = i;
                    break;
                }
            }
            double stepx = (vxtm * indxj / prof_expl_adv); //paso resultante, para distancia fija
            for (int i = 0; i < prof_expl_adv; i++)
            {
                traj.xval[i] = CurrentPoint.xval + ((i + advance_prediction) * stepx);
                //cout<<"==XVAl "<<traj.xval[i]<<endl;
            }

            for (int i = 0; i < prof_expl_adv; i++) //Calculo, ahora si, de los datos de trayectoria
            {
                traj.yval[i] = 0.0;
                for (int j = 0; j <= n; j++)
                    traj.yval[i] += (coeffs[j] * (pow(traj.xval[i], j)));
            }
        }
        else
        {
            Regression(acum_y, acum_x, d_prv, 1, n, coeffs); //lado contrario, ejes cambiados
            double signy = acum_y[d_prv] - acum_y[d_prv - 1];
            if (signy >= 0)
                vytm = vytm;
            else
                vytm = -vytm;
            for (int i = 0; i < stepc; i++)
                yvala[i] = CurrentPoint.yval + (i * vytm);

            int indyj = 1;
            for (int i = 0; i < stepc; i++) //Calculo de distancia junto con la regresion-
            {
                xvala[i] = 0.0;
                for (int j = 0; j <= n; j++)
                    xvala[i] += (coeffs[j] * (pow(yvala[i], j)));

                double hipy = sqrt((pow((CurrentPoint.yval - yvala[i]), 2)) + (pow((CurrentPoint.xval - xvala[i]), 2)));
                if (hipy > fixed_dist)
                {
                    indyj = i;
                    break;
                }
            }
            double stepy = (vytm * indyj / prof_expl_adv);
            for (int i = 0; i < prof_expl_adv; i++)
                traj.yval[i] = CurrentPoint.yval + ((i + advance_prediction) * stepy);

            for (int i = 0; i < prof_expl_adv; i++)
            {
                traj.xval[i] = 0.0;
                for (int j = 0; j <= n; j++)
                    traj.xval[i] += (coeffs[j] * (pow(traj.yval[i], j)));
            }
        }
        Tr = traj;
        //=========================================================================================================================================
        //=========================================COMPOSICION DE TRAYECTORIA======================================================================

        if (first_tr)
        {
            /*double dvxy = abs((mean.vx + mean.vy) / 2);
            tr_brk = prof_expl_adv - 1; //Primero suponer que toda la trayectoria debe reemplazarse
            for (int j = 0; j < prof_expl_adv - 1; j++)
            {
                //cv::circle( image, cv::Point( ( traj.xval[j]+maxsc1)*scale1,(traj.yval[j]+maxsc1)*scale1 ), 1, cv::Scalar( 240, 0, 0 ),  2, 8 );
                double d_tr = sqrt((traj.xval[j] - Tr_old.xval[j + 1]) * (traj.xval[j] - Tr_old.xval[j + 1]) + (traj.yval[j] - Tr_old.yval[j + 1]) * (traj.yval[j] - Tr_old.yval[j + 1]));
                if (d_tr > 2 * dvxy)
                {
                    tr_brk = j;
                    break;
                }
            }*/
            for (int j = 0; j < prof_expl_adv; j++)
            {
                /*  if (j < tr_brk)
                {
                    Tr.xval[j] = Tr_old.xval[j + 1]; //Asignar trayectoria antigua antes del punto de quiebre
                    Tr.yval[j] = Tr_old.yval[j + 1];
                    Tr.zval[j] = zvalue;
                }
                else
                {*/
                Tr.xval[j] = traj.xval[j]; //Asignar trayectoria nueva despues del punto de quiebre
                Tr.yval[j] = traj.yval[j];
                Tr.zval[j] = zvalue;
                //}
                //                 cv::circle( image_Ptraj, cv::Point( ( Tr.xval[j]+maxsc1)*scale1, (Tr.yval[j]+maxsc1)*scale1 ), 1, cv::Scalar( 240, 0, 0 ),  2, 8 );
            }
            CheckandFix_Boundaries(Tr.xval, Tr.yval, prof_expl_adv);
        }
        fixed_dist = rrt_extension;
    }
    Tr_Original = Tr;

    /*Check_Recover_Trajectory(false);
    //Average_OldTrajectory();
    SmoothTrajectory_Average(7, 2);
    Check_Recover_Trajectory(false);
    SmoothTrajectory_Average(5, 2);
    //Average_OldTrajectory();
    for (int k = 0; k < 6; k++)
    {
        //Check_Recover_Trajectory(false);
        SmoothTrajectory_Average(3, 2);
        // Check_Recover_Trajectory(false);
    }
*/
    //SmoothTrajectory_Average(3, 1);

    //SmoothTrajectory();
    TP_Mtx.unlock();
    for (int i = 0; i < Tr.xval.size() - 1; i++)
    {
        //Recovered TRajectory
        cv::circle(image_Ptraj, cv::Point(round((Tr.xval[i] + maxsc) * scale), round((Tr.yval[i] + maxsc) * scale)), 3, cv::Scalar(204, 0, 102), -1, 8);
    }
    first_tr = true;
    Tr_old = Tr;
    acum_x = accum_x_copy;
    acum_y = accum_y_copy;
    return;
}

void Prediction::Check_Recover_Trajectory_tendency() //should be executed in sequence
{
    //CHECK transformations cells <-> Real word xy coords
    Etraj Tr_Cells = Tr_to_Cells(Tr); //convert to cell coords
    Etraj Final_Tr = Tr;
    int max_iter = 200;
    std::string tendency_v("none");
    std::string tendency_h("none");
    int overshot = 7;
    double global_tr_correction_x = 0.0;
    double global_tr_correction_y = 0.0;
    //for (int i = Tr_Cells.xval.size() - 1; i >= 0; i--)
    for (int i = 0; i < Tr_Cells.xval.size(); i++)
    {
        bool horiz = false;
        if (i > 0)
        {
            double diffx = abs(Tr_Cells.xval[i] - Tr_Cells.xval[i - 1]);
            double diffy = abs(Tr_Cells.yval[i] - Tr_Cells.yval[i - 1]);
            if (diffx > diffy)
                horiz = true;
        }

        int x = Tr_Cells.xval[i];
        int y = Tr_Cells.yval[i];
        if (ObstacleMap[x][y] > 0)
        {
            int inc = 0;
            float ch = 1;
            bool found_better = false;
            int xchk, ychk;
            for (int j = 0; j < max_iter; j++)
            {
                if (!horiz)
                {
                    if (Check_Map_Coord(x + inc, y) && tendency_h != "left")
                    {
                        xchk = x + inc + overshot;
                        ychk = y; //right
                        tendency_h = "right";
                        found_better = true;
                        break;
                    }

                    if (Check_Map_Coord(x - inc, y) && tendency_h != "right")
                    {
                        xchk = x - inc - overshot;
                        ychk = y; //left
                        tendency_h = "left";
                        found_better = true;
                        break;
                    }
                }
                else
                {

                    if (Check_Map_Coord(x, y + inc) && tendency_v != "bottom")
                    {
                        xchk = x;
                        ychk = y + inc + overshot; //top
                        tendency_v = "top";
                        found_better = true;
                        break;
                    }

                    if (Check_Map_Coord(x, y - inc) && tendency_v != "top")
                    {
                        xchk = x;
                        ychk = y - inc - overshot; //bottom
                        tendency_v = "bottom";
                        found_better = true;
                        break;
                    }
                }

                if (Check_Map_Coord(x + inc, y + inc) && tendency_v != "bottom" && tendency_h != "left")
                {
                    xchk = x + inc + overshot;
                    ychk = y + inc + overshot; //top-right
                    tendency_h = "right";
                    tendency_v = "top";

                    found_better = true;
                    break;
                }

                if (Check_Map_Coord(x - inc, y - inc) && tendency_v != "bottom" && tendency_h != "right")
                {
                    xchk = x - inc - overshot;
                    ychk = y - inc - overshot; //top-left
                    tendency_h = "left";
                    tendency_v = "top";
                    found_better = true;
                    break;
                }

                if (Check_Map_Coord(x + inc, y - inc) && tendency_v != "top" && tendency_h != "left")
                {
                    xchk = x + inc + overshot;
                    ychk = y - inc - overshot; //bottom-right
                    tendency_h = "right";
                    tendency_v = "bottom";
                    found_better = true;
                    break;
                }

                if (Check_Map_Coord(x - inc, y - inc) && tendency_v != "top" && tendency_h != "right")
                {
                    xchk = x - inc - overshot;
                    ychk = y - inc - overshot; //bottom-left
                    tendency_h = "left";
                    tendency_v = "bottom";
                    found_better = true;
                    break;
                }
                inc++;
            }
            if (found_better)
            {
                double point_correction_x = (xchk - ((MapSize - 1) / 2)) / MapResolution;
                double point_correction_y = (ychk - ((MapSize - 1) / 2)) / MapResolution;
                Tr.xval[i] = point_correction_x + 3 * global_tr_correction_x / Tr_Cells.xval.size();
                Tr.yval[i] = point_correction_y + 3 * global_tr_correction_y / Tr_Cells.xval.size();

                //  global_tr_correction_x += point_correction_x;
                // global_tr_correction_y += point_correction_y;
            }
        }
    }
    Tr_old = Tr;

    return;
}

void Prediction::Check_Recover_Trajectory(bool add_global) //should be executed in sequence
{
    //CHECK transformations cells <-> Real word xy coords
    Etraj Tr_Cells = Tr_to_Cells(Tr); //convert to cell coords
    Etraj Final_Tr = Tr;
    int max_iter = 250;
    std::string tendency_v("none");
    std::string tendency_h("none");
    int overshot = 10;

    double global_tr_correction_x = 0.0;
    double global_tr_correction_y = 0.0;
    //  for (int i = Tr_Cells.xval.size() - 1; i >= 0; i--)
    int size_ordered_indexes = tr_order_indexes.size();
    for (int k = 0; k < size_ordered_indexes; k++)
    {
        int i = k; //tr_order_indexes[k];

        bool horiz = false;

        if (i > 0)
        {
            double diffx = abs(Tr_Cells.xval[i] - Tr_Cells.xval[i - 1]);
            double diffy = abs(Tr_Cells.yval[i] - Tr_Cells.yval[i - 1]);
            if (diffx > diffy)
                horiz = true;
        }
        else
        {
            double diffx = abs(Tr_Cells.xval[i + 1] - Tr_Cells.xval[i]);
            double diffy = abs(Tr_Cells.yval[i + 1] - Tr_Cells.yval[i]);
            if (diffx > diffy)
                horiz = true;
        }

        int x = Tr_Cells.xval[i];
        int y = Tr_Cells.yval[i];

        if (ObstacleMap[x][y] > 0)
        {
            int inc = 0;
            float ch = 1;
            bool found_better = false;
            int xchk, ychk;
            for (int j = 0; j < max_iter; j++)
            {

                /* if (Check_Map_Coord(x + inc, y + inc) && tendency_v != "bottom" && tendency_h != "left")
                {
                    xchk = x + inc + overshot;
                    ychk = y + inc + overshot; //top-right
                    tendency_h = "right";
                    tendency_v = "top";

                    found_better = true;
                    break;
                }

                if (Check_Map_Coord(x - inc, y - inc) && tendency_v != "bottom" && tendency_h != "right")
                {
                    xchk = x - inc - overshot;
                    ychk = y - inc - overshot; //top-left
                    tendency_h = "left";
                    tendency_v = "top";
                    found_better = true;
                    break;
                }

                if (Check_Map_Coord(x + inc, y - inc) && tendency_v != "top" && tendency_h != "left")
                {
                    xchk = x + inc + overshot;
                    ychk = y - inc - overshot; //bottom-right
                    tendency_h = "right";
                    tendency_v = "bottom";
                    found_better = true;
                    break;
                }

                if (Check_Map_Coord(x - inc, y - inc) && tendency_v != "top" && tendency_h != "right")
                {
                    xchk = x - inc - overshot;
                    ychk = y - inc - overshot; //bottom-left
                    tendency_h = "left";
                    tendency_v = "bottom";
                    found_better = true;
                    break;
                }*/
                if (!horiz)
                {
                    if (Check_Map_Coord(x + inc, y) && tendency_h != "left")
                    {
                        xchk = x + inc + overshot;
                        ychk = y; //right
                        tendency_h = "right";
                        found_better = true;
                        break;
                    }

                    if (Check_Map_Coord(x - inc, y) && tendency_h != "right")
                    {
                        xchk = x - inc - overshot;
                        ychk = y; //left
                        tendency_h = "left";
                        found_better = true;
                        break;
                    }
                }
                else
                {

                    if (Check_Map_Coord(x, y + inc) && tendency_v != "bottom")
                    {
                        xchk = x;
                        ychk = y + inc + overshot; //top
                        tendency_v = "top";
                        found_better = true;
                        break;
                    }

                    if (Check_Map_Coord(x, y - inc) && tendency_v != "top")
                    {
                        xchk = x;
                        ychk = y - inc - overshot; //bottom
                        tendency_v = "bottom";
                        found_better = true;
                        break;
                    }
                }

                inc++;
                if (i < 1 && i > 9)
                {
                    tendency_v = "none"; //override results for extremes
                    tendency_h = "none";
                }
            }
            if (found_better)
            {
                Tr.xval[i] = (xchk - ((MapSize - 1) / 2)) / MapResolution;
                Tr.yval[i] = (ychk - ((MapSize - 1) / 2)) / MapResolution;
                double point_correction_x = (xchk - x - ((MapSize - 1) / 2)) / MapResolution;
                double point_correction_y = (ychk - y - ((MapSize - 1) / 2)) / MapResolution;
                global_tr_correction_x += point_correction_x;
                global_tr_correction_y += point_correction_y;
            }
        }
        if (add_global)
        {
            Tr.xval[i] += global_tr_correction_x / (10 * Tr.xval.size());
            Tr.yval[i] += global_tr_correction_y / (10 * Tr.xval.size());
        }
    }
    Tr_old = Tr;
    return;
}

void Prediction::Average_OldTrajectory()
{
    for (int i = 0; i < Tr.xval.size(); i++)
    {
        double x_acum = 0.0;
        double y_acum = 0.0;
        x_acum = Tr.xval[i] + Tr_old.xval[i];
        y_acum = Tr.yval[i] + Tr_old.yval[i];

        x_acum /= 2;
        y_acum /= 2;

        Tr.xval[i] = x_acum;
        Tr.yval[i] = y_acum;
    }
    return;
}

void Prediction::SmoothTrajectory_Average(int neighbours, int iterations)
{
    int NO_OF_NEIGHBOURS = neighbours; //odd number
    int sides_neighbours = (NO_OF_NEIGHBOURS - 1) / 2;

    for (int k = 0; k < iterations; k++)
    {
        for (int i = 1; i < Tr.xval.size() - 2; i++)
        {
            double x_acum = 0.0;
            double y_acum = 0.0;
            if (i < sides_neighbours)
            {
                int down_spaces = i;
                for (int j = -down_spaces; j <= down_spaces; j++) //NO_OF_NEIGHBOURS - i
                {

                    x_acum += Tr.xval[i + j];
                    y_acum += Tr.yval[i + j];
                }
                x_acum /= 2 * down_spaces + 1;
                y_acum /= 2 * down_spaces + 1;
            }
            else if (i >= (Tr.xval.size() - sides_neighbours))
            {
                int up_spaces = Tr.xval.size() - i - 2;
                for (int j = -up_spaces; j <= up_spaces; j++) //-NO_OF_NEIGHBOURS + up_spaces + 1
                {
                    x_acum += Tr.xval[i + j];
                    y_acum += Tr.yval[i + j];
                }
                x_acum /= (2 * up_spaces) + 1;
                y_acum /= (2 * up_spaces) + 1;
            }
            else
            {
                for (int j = -sides_neighbours; j <= sides_neighbours; j++)
                {
                    x_acum += Tr.xval[i + j];
                    y_acum += Tr.yval[i + j];
                }
                x_acum /= NO_OF_NEIGHBOURS;
                y_acum /= NO_OF_NEIGHBOURS;
            }
            Tr.xval[i] = x_acum;
            Tr.yval[i] = y_acum;
        }
    }
    return;
}

void Prediction::SmoothTrajectory()
{
    double diffx_global = 0.0, diffy_global = 0.0;
    for (int i = 0; i < Tr.xval.size() - 1; i++)
    {
        diffx_global += abs(Tr.xval[i + 1] - Tr.xval[i]);
        diffy_global += abs(Tr.yval[i + 1] - Tr.yval[i]);
    }
    diffx_global /= (Tr.xval.size() - 1);
    diffy_global /= (Tr.yval.size() - 1);

    for (int j = 0; j < 5; j++)
    {
        double accum_x_error = 0.0;
        double accum_y_error = 0.0;
        for (int i = 0; i < Tr.xval.size() - 1; i++)
        {
            if (i != 0 && i != Tr.xval.size() - 1)
            {
                double diffx = abs(Tr.xval[i] - Tr.xval[i - 1]);
                double diffy = abs(Tr.yval[i] - Tr.yval[i - 1]);
                double meanx;
                double meany;
                if (diffx > 0.002 && diffy_global > (3 * diffx_global))
                {

                    if (i == Tr.xval.size() - 2)
                    {
                        meanx = (Tr.xval[i] + Tr.xval[i - 1]) / 2;
                    }
                    else
                    {
                        meanx = (Tr.xval[i] + Tr.xval[i + 1] + Tr.xval[i - 1]) / 3;
                    }
                    Tr.xval[i] = meanx + accum_x_error / 2;
                }
                else if (diffy > 0.002 && diffx_global > 3 * diffy_global)
                {
                    if (i == Tr.xval.size() - 2)
                    {
                        meany = (Tr.yval[i] + Tr.yval[i - 1]) / 2;
                    }
                    else
                    {
                        meany = (Tr.yval[i] + Tr.yval[i + 1] + Tr.yval[i - 1]) / 3;
                    }

                    Tr.yval[i] = meany + accum_y_error / 2;
                }
                accum_x_error += meanx;
                accum_y_error += meany;
            }
        }
    }
}

Etraj Prediction::Tr_to_Cells(Etraj tr)
{
    Etraj Traj_Cells = tr;
    for (int i = 0; i < tr.xval.size(); i++)
    {
        num.MinMax_Correction(tr.xval[i], max_dimm);
        num.MinMax_Correction(tr.yval[i], max_dimm);

        // cout<<Traj_Cells.xval[i] <<", "<<tr.xval[i] <<", "<<MapResolution<<", "<<HalfMapSize +  round(tr.xval[i]*MapResolution)<<endl;
        Traj_Cells.xval[i] = HalfMapSize + round(tr.xval[i] * MapResolution); //now between 0 and MapSize
        Traj_Cells.yval[i] = HalfMapSize + round(tr.yval[i] * MapResolution);
        Traj_Cells.zval[i] = HalfMapSize + round(tr.zval[i] * MapResolution);
    }
    return Traj_Cells;
}
Position_ Prediction::RealPosition_to_Cells(Position_ Pos)
{
    Position_ Position_Cells;

    num.MinMax_Correction(Pos.x, max_dimm);
    num.MinMax_Correction(Pos.y, max_dimm);

    Position_Cells.x = HalfMapSize + round(Pos.x * MapResolution); //now between 0 and MapSize
    Position_Cells.y = HalfMapSize + round(Pos.y * MapResolution);
    Position_Cells.z = HalfMapSize + round(Pos.z * MapResolution);

    return Position_Cells;
}
double Prediction::Cell_to_Real(int point_cell)
{
    double RealPoint = ((point_cell - (MapSize - 1) / 2.0) / MapResolution);
    return RealPoint;
}

bool Prediction::Check_Map_Coord(int x, int y)
{

    if (x >= 0 && x < MapSize && y >= 0 && y < MapSize)
    {
        if (ObstacleMap[x][y] == 0)
            return true;
        else
            return false;
    }
    else
        return false;
}

struct rrtns::MeanValues Prediction::XYMean_Calculation(geometry_msgs::Pose Marker_Abs_Pose)
{
    //  image_Ptraj = White_Imag.clone();//.setTo(cv::Scalar(255,255,255));

    mean.vx = 0.0;
    mean.vy = 0.0;
    for (int i = 0; i < d_prv; i++)
    {
        acum_x[i] = acum_x[i + 1]; //- ugv_state.velocity_linear.dx * ugv_state_factor;
        acum_y[i] = acum_y[i + 1]; //+ ugv_state.velocity_linear.dy * ugv_state_factor;
    }

    acum_x[d_prv] = Marker_Abs_Pose.position.x;
    acum_y[d_prv] = Marker_Abs_Pose.position.y;

    /*  double UAV_Velocity_temp = 0.0;
    for (int i = 0; i < d_prv-1; i++)
    {
        double x_velocity = acum_x[i + 1] - acum_x[i];
        double y_velocity = acum_y[i + 1] - acum_y[i];

        UAV_Velocity_temp += sqrt((x_velocity * x_velocity) + (y_velocity * y_velocity));
    }
    Velocity_mtx.lock();
    UAV_Velocity = UAV_Velocity_temp ;//   / (double)(d_prv-1);
    Velocity_mtx.unlock();
*/

    //Marker_Pose_Manipulator_Coords = Marker_Abs_Pose;

    return mean;
}

void Prediction::Regression(std::vector<double> x, std::vector<double> y, int ndatos, int it, int order, std::vector<double> &coeffs)
{
    //x-Valores de X
    //y-Valores de Y
    //ndatos - Cantidad de datos para realizar regresion
    //prof - Cuantos datos se van a predecir en futuro
    //it - numero de iteracion en la que inicia la regresion
    //order - Orden de regresion
    int n = order;
    const int nt = n;
    int tn = 2 * n + 1;
    std::vector<double> SX(tn), SY(tn);
    std::vector<std::vector<double>> B(n + 1, std::vector<double>(n + 2, 0));
    std::vector<double> a(order + 1);
    // std::vector<double> coeffs(order+2);

    Etraj traj;
    traj.xval.resize(ndatos);
    traj.yval.resize(ndatos);
    traj.zval.resize(ndatos);
    int cnt = 0;
    double tmp;
    for (int i = 0; i < ndatos; i++)
    {
        traj.xval[cnt] = x[i];
        traj.yval[cnt] = y[i];
        cnt++;
    }

    for (int m = 0; m < (tn); ++m)
    {
        SX[m] = 0.0;
        for (int j = 0; j < ndatos; ++j)
            SX[m] += pow(traj.xval[j], m);
    }
    for (int m = 0; m <= n; ++m)
    {
        SY[m] = 0.0;
        for (int j = 0; j < ndatos; ++j)
            SY[m] += pow(traj.xval[j], m) * traj.yval[j];
    }
    for (int i = 0; i <= n; ++i)
        for (int j = 0; j <= n; ++j)
            B[i][j] = SX[i + j];

    for (int i = 0; i <= n; ++i)
        B[i][n + 1] = SY[i];
    n += 1;
    int nm1 = n - 1;

    // Pivotisation of the B matrix.
    for (int i = 0; i < n; ++i)
        for (int k = i + 1; k < n; ++k)
            if (B[i][i] < B[k][i])
                for (int j = 0; j <= n; ++j)
                {
                    tmp = B[i][j];
                    B[i][j] = B[k][j];
                    B[k][j] = tmp;
                }
    // Performs the Gaussian elimination.
    // (1) Make all elements below the pivot equals to zero
    //     or eliminate the variable.
    for (int i = 0; i < nm1; ++i)
        for (int k = i + 1; k < n; ++k)
        {
            double t = B[k][i] / B[i][i];
            for (int j = 0; j <= n; ++j)
                B[k][j] -= t * B[i][j]; // (1)
        }
    // Back substitution.
    // (1) Set the variable as the rhs of last equation
    // (2) Subtract all lhs values except the target coefficient.
    // (3) Divide rhs by coefficient of variable being calculated.
    for (int i = nm1; i >= 0; --i)
    {
        a[i] = B[i][n]; // (1)
        for (int j = 0; j < n; ++j)
            if (j != i)
                a[i] -= B[i][j] * a[j]; // (2)
        a[i] /= B[i][i];                // (3)
    }
    //coeffs.resize(a.size());
    int NN = a.size();

    for (int i = 0; i < NN; ++i)
    {
        coeffs[i] = a[i]; //cout<<"Coeficientes"<<a[i]<<endl;
    }
    return;
}

void Prediction::CheckandFix_Boundaries(std::vector<double> &x, std::vector<double> &y, int &prof_e)
{
    double cat1, cat2, offx, offy;
    offx = 0.0;
    offy = 0.0;
    double theta = 0;
    double rad;
    float sx = 0, sy = 0, corg = 0.0011;
    double diff_prev = 0;
    float xf, yf, xo, yo;
    int cndf = 1, colorred, sc;
    double diffxa = 0, diffya = 0, sqdiffs;
    int div = 1;

    Positions op;
    op.xval.resize(6);
    op.yval.resize(6);
    for (int i = 0; i < 6; i++)
    {
        op.xval[i] = 0;
        op.yval[i] = 0;
    }
    for (int i = 0; i < prof_e; i++)
    {
        cat1 = x[i] - offx;
        cat2 = y[i] - offy;
        rad = sqrt((cat1 * cat1) + (cat2 * cat2));
        if (rad >= rad_ext)
        { //Circulo externo
            if (x[i] < 0)
            {
                theta = PI + atan(y[i] / x[i]);
            }
            else
            {
                theta = atan(y[i] / x[i]);
            }
            y[i] = rad_ext * sin(theta);
            x[i] = rad_ext * cos(theta);
        }
        sx = 0;
        sy = 0;
        if (rad <= rad_int)
        { //Circulo interno
            xf = x[i], yf = y[i];
            if (i == 0)
            {
                xo = x[i], yo = y[i];
            }
            else
            {
                xo = x[i - 1], yo = y[i - 1];
            }

            float dx = 1.5 * (xf - xo) / 1;
            float dy = 1.5 * (yf - yo) / 1, maxC = 1.0;

            if ((xf >= 0) && (-dx > maxC * xf))
            {
                dx = -maxC * xf;
            }
            //     or eliminate the variable.
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

            for (int j = 0; j < 5; j++)
            {
                op.xval[j] = op.xval[j + 1];
                op.yval[j] = op.yval[j + 1];
            }
            op.xval[5] = dx;
            op.yval[5] = dy;

            div = i + 1;
            if (i > 5)
                div = 6;

            double dxa = (op.xval[0] + op.xval[1] + op.xval[2] + op.xval[3] + op.xval[4] + op.xval[5]) / div;
            double dya = (op.yval[0] + op.yval[1] + op.yval[2] + op.yval[3] + op.yval[4] + op.yval[5]) / div;
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

            double corrx = 0.05 * (xf3 - xo);
            double corry = 0.05 * (yf3 - yo);

            if (dx > 0 && corrx < 0)
                corrx = 0;
            if (dx < 0 && corrx > 0)
                corrx = 0;
            if (dy > 0 && corry < 0)
                corry = 0;
            if (dy < 0 && corry > 0)
                corry = 0;
            if (corrx > corg)
                corrx = corg;
            if (corrx < -corg)
                corrx = -corg;
            if (corry > corg)
                corry = corg;
            if (corry < -corg)
                corry = -corg;
            x[i] += corrx;
            y[i] += corry;
        }

        xf = x[i], yf = y[i];
        if (i == 0)
        {
            xo = x[i], yo = y[i];
        }
        else
        {
            xo = x[i - 1], yo = y[i - 1];
        }
        colorred = 220, sc = 1;
        diff_prev = sqrt((xf - xo) * (xf - xo) + (yf - yo) * (yf - yo));
        if (diff_prev > 0.1)
        {
            cndf++;
            if (diff_prev > 0.15)
                sc = 100;

            x[i] = xo + ((diffxa) / (cndf * sc));
            y[i] = yo + ((diffya) / (cndf * sc));
            diffxa = x[i] - xo;
            diffya = y[i] - yo;
            sqdiffs = sqrt((diffxa * diffxa) + (diffya * diffya));
            if (sqdiffs > 0.6)
            {
                prof_e = i;
                break;
            }
            colorred = 160;
        }
#ifdef OPENCV_DRAW
        //mtxA.lock();
        //CHECKED ORIGINAL TRAJECTORY
        //cv::Vec3b &color = image_Ptraj.at<cv::Vec3b>(round((y[i] + maxsc) * scale), round((x[i] + maxsc) * scale));
        //    color[0] = 0;
        //    color[1] = 0;
        //    color[2] = 0;
        cv::circle(image_Ptraj, cv::Point(round((x[i] + maxsc) * scale), round((y[i] + maxsc) * scale)), 1, cv::Scalar(0, 0, 0), -1, 8);
//mtxA.unlock();
#endif
    }
    return;
}

void Prediction::RRT_Path_Generation()
{
    if (NodesCharged && !Stop_RRT_flag)
    {
#ifdef OPENCV_DRAW
        for (int j = 0; j < nodes.N; j++)
        {
            int x_point = (nodes.coord[j][0] + maxsc) * scale;
            int y_point = (nodes.coord[j][1] + maxsc) * scale;
            cv::Vec3b &color = image_Ptraj.at<cv::Vec3b>(y_point, x_point);
            color[0] = 0;
            color[1] = 96;
            color[2] = 220;
        }
#endif
        int d = 0;
        // Charge_Nodes(); //the copied nodes are now in nodes
        double DFactor = 0.0;
        double d_traj_adv = 0.0;

        double speed = 10 * sqrt((mean.vx * mean.vx) + (mean.vy * mean.vy));
        double max_cost = -100;
        int max_cost_indx;
        for (int i = 0; i < nodes.N; i++)
        {
            VectorDbl coordT = {Tr.xval[prof_expl], Tr.yval[prof_expl], Tr.zval[prof_expl]};
            double cost = nodes.cost[i] - Distance(nodes.coord[i], coordT) / 10.0;
            if (max_cost < cost)
            {
                max_cost = cost;
                max_cost_indx = i;
            }
        }

        int RRTVS_Indx, VS_Node_Indx;
        VectorDbl TR{Tr.xval[adv], Tr.yval[adv], Tr.zval[adv]}; //VS position
        double D_traj_avd = Distance(nodes.coord[0], TR);       //as VS is faster than RRT,its needed to find a point near the current VS position

        /* 
        DFactor = max_cost;// / (1.0 + speed) ;  //look for a node with this cost
        double Min_RRTVS_Dist=1000000.0,Min_VS_Node_Dist=10000.0;
        for(int i=1;i<nodes.N;i++)
        {
            double DistC = abs(nodes.cost[i] - D_traj_avd);//dist real of each node
            double NodeDist = abs(DFactor-DistC); // difference with Dfactor
            //Find node to a certain distance from VS position (indicated by DFactor)
            if(NodeDist < Min_RRTVS_Dist)  //store the one with a cost close to Dfactor
            {
                Min_RRTVS_Dist = NodeDist;
                RRTVS_Indx = i;
            }
            //Find a the closest node to VS point
            if(DistC < Min_VS_Node_Dist)
            {
                Min_VS_Node_Dist = DistC;
                VS_Node_Indx = i; //cause, vs point is progressing faster than rrt
            }
        }*/
        RRTVS_Indx = max_cost_indx;
        VS_Node_Indx = 0;
        bool goal_found = false;
        VectorInt road_indexes;
        road_indexes.push_back(RRTVS_Indx);
        int road_index_T = RRTVS_Indx;
        VectorInt RoadIndexes;
        RoadIndexes.push_back(RRTVS_Indx);
        if (VS_Node_Indx != RRTVS_Indx)
        {
            while (!goal_found)
            {
                int parent_T = nodes.parent[road_index_T];
                // if (parent_T>=0&&road_index_T>=0&&false){
                // cv::line( image_Ptraj, cv::Point((nodes.coord[road_index_T][0]+maxsc)*scale,(nodes.coord[road_index_T][1]+maxsc)*scale ),
                //cv::Point((nodes.coord[parent_T][0]+maxsc)*scale,(nodes.coord[parent_T][1]+maxsc)*scale ),
                //   cv::Scalar( 00, 230, 50 ),  2, 8 );}
                road_index_T = parent_T;
                road_indexes.push_back(road_index_T);
                if (road_index_T == VS_Node_Indx || nodes.parent[road_index_T] == -1 || road_index_T < 0)
                    goal_found = true;
                else
                    VS_Node_Indx = road_index_T;

                // if (nodes.cost[road_index_T] <= D_traj_avd)
                //{

                // }
                RoadIndexes.push_back(road_index_T);
            }
            PathPlanning_Indexes = RoadIndexes;
        }
        else
        {
            Print("fail finding nodes", nodes.N);
        }

        PathPlanning_Available = true;
        New_Nodes_from_RRT = false;
        NodesAvailable = false;

        //#ifdef STREAMING

        //#endif

#ifdef OPENCV_DRAW
        cv::circle(image_Ptraj, cv::Point((nodes.coord[VS_Node_Indx][0] + maxsc) * scale, (nodes.coord[VS_Node_Indx][1] + maxsc) * scale), 6, Colors[0], CV_FILLED, 3, 8);
        cv::circle(image_Ptraj, cv::Point((nodes.coord[RRTVS_Indx][0] + maxsc) * scale, (nodes.coord[RRTVS_Indx][1] + maxsc) * scale), 6, Colors[1], CV_FILLED, 3, 8);

#endif

        // cv::circle(image_Ptraj, cv::Point((nodes.coord[VS_Node_Indx][0] + maxsc) * scale, (nodes.coord[VS_Node_Indx][1] + maxsc) * scale), 6, Colors[0], CV_FILLED, 3, 8);
#ifdef STREAMING
        Text_Stream_RRTData->write_TimeStamp();
        int Tr_size = Tr.xval.size();
        for (int i = 0; i < Tr_size; i++)
        {
            Text_Stream_RRTData->write_Data(Tr.xval[i]);
            Text_Stream_RRTData->write_Data(Tr.yval[i]);
            Text_Stream_RRTData->write_Data(Tr.zval[i]);
            Text_Stream_RRTData->write_Data(i);
            Text_Stream_RRTData->write_Data(1);
            Text_Stream_RRTData->jump_line();
        }
        for (int i = 0; i < Tr_Original.xval.size(); i++)
        {
            Text_Stream_RRTData->write_Data(Tr_Original.xval[i]);
            Text_Stream_RRTData->write_Data(Tr_Original.yval[i]);
            Text_Stream_RRTData->write_Data(Tr_Original.zval[i]);
            Text_Stream_RRTData->write_Data(i);
            Text_Stream_RRTData->write_Data(2);
            Text_Stream_RRTData->jump_line();
        }

        //Draw vicinity here
#endif
        for (int j = 0; j < rrt_vicinity.R.size(); j++)
        {
            cv::ellipse(image_Ptraj, cv::Point(Img(rrt_vicinity.TP[j][0]), Img(rrt_vicinity.TP[j][1])),
                        cv::Size(scale * rrt_vicinity.R[j][0], scale * rrt_vicinity.R[j][1]),
                        rad_to_deg(rrt_vicinity.angles[j][0]), 0, 360, Colors[j], 1, 8);
        }

        int pplan_indx = PathPlanning_Indexes.size();
#ifdef SREAMING

        Text_Stream_RRTData->write_Data(nodes.coord[RoadIndexes[0]]);
        Text_Stream_RRTData->write_Data(0);
        Text_Stream_RRTData->write_Data(3);
        Text_Stream_RRTData->jump_line();
#endif
        for (int i = 0; i < pplan_indx - 1; i++)
        {
            int iR = PathPlanning_Indexes[i];
            int iRN = PathPlanning_Indexes[i + 1];

#ifdef OPENCV_DRAW
            if (i == pplan_indx - 2)
                cv::circle(image_Ptraj, cv::Point((nodes.coord[iR][0] + maxsc) * scale, (nodes.coord[iR][1] + maxsc) * scale), 5, Colors[5], CV_FILLED, 3, 8);
            cv::line(image_Ptraj, cv::Point((nodes.coord[iR][0] + maxsc) * scale, (nodes.coord[iR][1] + maxsc) * scale),
                     cv::Point((nodes.coord[iRN][0] + maxsc) * scale, (nodes.coord[iRN][1] + maxsc) * scale),
                     cv::Scalar(00, 230, 50), 2, 8);

#endif

#ifdef STREAMING
            Text_Stream_RRTData->write_Data(nodes.coord[iRN]);
            Text_Stream_RRTData->write_Data(iRN);
            Text_Stream_RRTData->write_Data(3);
            Text_Stream_RRTData->jump_line();
#endif
        }
    }
    return;
}

geometry_msgs::Pose Prediction::Selection_Function(float trust_index)
{
    int Final_VSPP_Index;
    geometry_msgs::Pose NextRobotRequest;
    if (PathPlanning_Available && !Stop_RRT_flag)
    {
        VectorDbl Visual_Servoing_position{Tr.xval[adv], Tr.yval[adv], Tr.zval[adv]};
        double minDistance = 1000000.0;
        int VS_Index_in_PPath = PathPlanning_Indexes.size() - 1;
        int Road_VS_Result_Indx = 0;
        //Find path planning road point closest to VS point
        for (int i = 0; i < PathPlanning_Indexes.size(); i++)
        {
            int PPlan_Indx = PathPlanning_Indexes[i];
            double distance = Distance(nodes.coord[PPlan_Indx], Visual_Servoing_position);
            if (distance < minDistance)
            {
                minDistance = distance;
                VS_Index_in_PPath = PPlan_Indx;
                Road_VS_Result_Indx = i;
            }
        }
        int original_indx = Road_VS_Result_Indx;
        Velocity_mtx.lock();
        double UAV_vel = UAV_Velocity;
        Velocity_mtx.unlock();

        if (trust_index != 0.0)
        {
            trust_index += (ugv_state.velocity_linear.dx);

            if (trust_index > 0.9)
            {
                Road_VS_Result_Indx -= 2;
            }
            if (trust_index > 0.8)
            {
                Road_VS_Result_Indx -= 2;
            }
            if (trust_index > 0.7)
            {
                Road_VS_Result_Indx -= 2;
            }
            if (trust_index > 0.6)
            {
                Road_VS_Result_Indx -= 2;
            }
            if (trust_index > 0.5)
            {
                Road_VS_Result_Indx -= 2;
            }
            if (trust_index > 0.4)
            {
                Road_VS_Result_Indx -= 2;
            }
            if (trust_index > 0.3)
            {
                Road_VS_Result_Indx -= 2;
            }

            Road_VS_Result_Indx -= PathPlanningAdvancing_Index;

            if (Road_VS_Result_Indx < 0)
                Road_VS_Result_Indx = 0;
        }
        Print("TRUST indexes", trust_index, Road_VS_Result_Indx, original_indx);
        // Road_VS_Result_Indx=original_indx;   // must delete
        Final_VSPP_Index = PathPlanning_Indexes[Road_VS_Result_Indx];

        //Add more cases here

        NextRobotRequest = Marker_Pose_Manipulator_Coords; //copying orientation

        NextRobotRequest.position.x = nodes.coord[Final_VSPP_Index][0]; //Then copy position
        NextRobotRequest.position.y = nodes.coord[Final_VSPP_Index][1];
        NextRobotRequest.position.z = nodes.coord[Final_VSPP_Index][2];
        Print("Poses NEXT", NextRobotRequest.position.x, NextRobotRequest.position.y, NextRobotRequest.position.z);
        Print("Poses MARKERREQ", Marker_Pose_Manipulator_Coords.position.x, Marker_Pose_Manipulator_Coords.position.y, Marker_Pose_Manipulator_Coords.position.z);
        //NextRobotRequest = Marker_Pose_Manipulator_Coords; //must delete

#ifdef OPENCV_DRAW
        cv::circle(image_Ptraj, cv::Point((nodes.coord[Final_VSPP_Index][0] + maxsc) * scale, (nodes.coord[Final_VSPP_Index][1] + maxsc) * scale), 7, cv::Scalar(200, 200, 0), -1, 8);
#endif

        if (UAV_vel > 0.01)
        {
            PathPlanningAdvancing_Index++;
        }
    }
    else
    {
        NextRobotRequest = Marker_Pose_Manipulator_Coords; //copying orientation
#ifdef OPENCV_DRAW
        cv::circle(image_Ptraj, cv::Point((Tr.xval[adv] + maxsc) * scale, (Tr.yval[adv] + maxsc) * scale), 6, cv::Scalar(200, 0, 0), -1, 8);
#endif
        /*NextRobotRequest.position.x = Tr.xval[adv]; //Then copy position
        NextRobotRequest.position.y = Tr.yval[adv];
        NextRobotRequest.position.z = Tr.zval[adv];*/
        Final_VSPP_Index = adv;
    }
#ifdef SREAMING

    Text_Stream_RRTData->write_Data(NextRobotRequest.position.x);
    Text_Stream_RRTData->write_Data(NextRobotRequest.position.y);
    Text_Stream_RRTData->write_Data(NextRobotRequest.position.z);
    Text_Stream_RRTData->write_Data(Final_VSPP_Index);
    Text_Stream_RRTData->write_Data(4);
    Text_Stream_RRTData->jump_line();
#endif
    return NextRobotRequest;
}
void Prediction::Draw_Map()
{
#ifdef OPENCV_DRAW
    int obs_thick_size = Obstacle_Points_thick.size();
    // int obs_size = Obstacle_Points.size();
    // int major_size = obs_thick_size;
    /* if (obs_size >= major_size)
    {
        major_size = obs_size;
    }*/

    for (int i = 0; i < obs_thick_size; i++)
    {
        /* if (i < obs_thick_size)
        {*/
        image_Ptraj.at<cv::Vec3b>(image_size - Obstacle_Points_thick[i].xval, image_size - Obstacle_Points_thick[i].yval) = {200, 200, 200};
        //color[0] = 200;
        //color[1] = 200;
        //color[2] = 200;
        /* }*/
        /* if (i < obs_size)
        {
            color = image_Ptraj.at<cv::Vec3b>(1+image_size-Obstacle_Points[i].yval, Obstacle_Points[i].xval);
            color[0] = 70;
            color[1] = 70;
            color[2] = 70;
        }*/
    }

#endif
    return;
}

geometry_msgs::Pose Prediction::NoTarget_Sequence(geometry_msgs::Pose Marker_Abs_Pose) //No quad
{
    geometry_msgs::Pose Final_EEFF_Pose = Marker_Abs_Pose;
    int max_iter = 200;
    Position_ CurrentRealPoint;
    CurrentRealPoint.x = Marker_Abs_Pose.position.x;
    CurrentRealPoint.y = Marker_Abs_Pose.position.y;
    CurrentRealPoint.z = Marker_Abs_Pose.position.z;
    Position_ CurrentPoint_Cells = RealPosition_to_Cells(CurrentRealPoint);

    bool horiz = false;

    int x = CurrentPoint_Cells.x;
    int y = CurrentPoint_Cells.y;

    if (ObstacleMap[x][y] > 0)
    {
        int inc = 0;
        float ch = 1;
        bool found_better = false;
        int xchk, ychk;
        for (int j = 0; j < max_iter; j++)
        {
            if (Check_Map_Coord(x + inc, y))
            {
                xchk = x + inc;
                ychk = y; //right
                found_better = true;
                break;
            }

            if (Check_Map_Coord(x - inc, y))
            {
                xchk = x - inc;
                ychk = y; //left
                found_better = true;
                break;
            }

            if (Check_Map_Coord(x, y + inc))
            {
                xchk = x;
                ychk = y + inc; //top
                found_better = true;
                break;
            }

            if (Check_Map_Coord(x, y - inc))
            {
                xchk = x;
                ychk = y - inc; //bottom
                found_better = true;
                break;
            }
            inc++;
        }

        if (found_better)
        {
            Final_EEFF_Pose.position.x = (xchk - ((MapSize - 1) / 2)) / MapResolution;
            Final_EEFF_Pose.position.y = (ychk - ((MapSize - 1) / 2)) / MapResolution;
        }
    }
    double cat1, cat2;
    cat1 = Final_EEFF_Pose.position.x;
    cat2 = Final_EEFF_Pose.position.y;
    double rad = sqrt((cat1 * cat1) + (cat2 * cat2));

    if (rad >= rad_ext - 0.005)
    {
        double theta = 0;
        if (Final_EEFF_Pose.position.x < 0.0)
        {
            theta = PI + atan(Final_EEFF_Pose.position.y / Final_EEFF_Pose.position.x);
        }
        else
        {
            theta = atan(Final_EEFF_Pose.position.y / Final_EEFF_Pose.position.x);
        }

        Final_EEFF_Pose.position.y = (rad_ext - 0.01) * sin(theta);
        Final_EEFF_Pose.position.x = (rad_ext - 0.01) * cos(theta);
    }

#ifdef OPENCV_DRAW

    //  cv::circle(image_Ptraj, cv::Point((int)((Final_EEFF_Pose.position.x + maxsc) * scale), (int)((Final_EEFF_Pose.position.y + maxsc) * scale)), 6, Colors[0], CV_FILLED, 3, 8);

    int x_point = (Final_EEFF_Pose.position.x + maxsc) * scale;
    int y_point = (Final_EEFF_Pose.position.y + maxsc) * scale;
    cv::Vec3b &color = image_Ptraj.at<cv::Vec3b>(y_point, x_point);
    color[0] = 0;
    color[1] = 9;
    color[2] = 20;
#endif
    return Final_EEFF_Pose;
}

void Prediction::Planif_SequenceA(geometry_msgs::Pose Marker_Abs_Pose, geometry_msgs::Pose CurrentArmPose, geometry_msgs::Pose NextRequestPartialPose, bool docking_process_flag) //extraer vecindad
{

    //tic();
    XYMean_Calculation(Marker_Abs_Pose);
    //Print("tiempo RRT XY Mean Calc",toc());
    //tic();
    // Print("-------RRt2 TrajPredict------------");
    eeff_min_height = CurrentArmPose.position.z; //this is the z value for the entire rrt, modify here to contact phase
    Marker_Pose_Manipulator_Coords = NextRequestPartialPose;
    Trajectory_Prediction(Marker_Abs_Pose, CurrentArmPose, docking_process_flag);
    // Print("AAA tiempo Traj Predict",toc());

    return;
}
void Prediction::ClearImage_Ptraj()
{
#ifdef OPENCV_DRAW
    White_Imag.copyTo(image_Ptraj);
#endif
}
void Prediction::InsertElement_in_Vector(VectorDbl &vector, int Position, double value)
{
    VectorDbl::iterator it = vector.begin();
    vector.insert(it + Position, value);
    return;
}
double Prediction::Distance(VectorDbl P0, VectorDbl P1)
{
    return sqrt(((P1[0] - P0[0]) * (P1[0] - P0[0])) + ((P1[1] - P0[1]) * (P1[1] - P0[1])) + ((P1[2] - P0[2]) * (P1[2] - P0[2])));
}
inline int Prediction::Img(double point)
{
    point += maxsc;
    point *= scale;
    return int(point);
}
inline double Prediction::rad_to_deg(double rad)
{
    double deg = rad * 180 / PI;
    return deg;
}
#endif //PREDICTION
