#ifndef RRT_PLANIF_CODE
#define RRT_PLANIF_CODE
#include "rrt_functions.hpp"
using namespace rrt_planif;

void RRT::Initialize_VicinityRRT()
{
    std::vector<double> angles(3);
    double dnprv, dnxt, dm;
    adv = 0;
    for (int j = 0; j < prof_expl; j++)
    {
        vdr.TP[j][0] = Tr.xval[j + adv]; //Se carga la trayectoria predicha en esta iteracion, a los valores de trayectoria nuevos
        vdr.TP[j][1] = Tr.yval[j + adv];
        vdr.TP[j][2] = Tr.zval[j + adv];
        // mtxA.lock();
#ifdef OPENCV_DRAW
        cv::circle(image_Ptraj, cv::Point((Tr.xval[j + adv] + maxsc) * scale, (Tr.yval[j + adv] + maxsc) * scale), 4, Colors[0], 3, 8);
#endif
        //mtxA.unlock();
    }
    for (int j = 0; j < 4; j++)
    {
        Old_Positions.xval[j] = Old_Positions.xval[j + 1];
        Old_Positions.yval[j] = Old_Positions.yval[j + 1];
        Old_Positions.zval[j] = Old_Positions.zval[j + 1];
    }
    Old_Positions.xval[4] = Tr.xval[0];
    Old_Positions.yval[4] = Tr.yval[0];
    Old_Positions.zval[4] = Tr.zval[0];
    double UAV_vel = 0.0;
    for (int j = 0; j < 4; j++)
    {
        UAV_vel += sqrt((Old_Positions.xval[j] * Old_Positions.xval[j]) + (Old_Positions.xval[j + 1] * Old_Positions.xval[j + 1]));
    }
    UAV_Velocity = UAV_vel;

    for (int j = 0; j < prof_expl; j++) //desde tr_brk hasta el ultmo valor de prof_e (7 que es el octavo valor), ultimo valor de trajectoria predicha
    {
        if (j == 0) //Si es el primer paso
        {
            angles = Angles_Calculation(vdr.TP[j], vdr.TP[j + 1]); //calcular inicial y siguiente
            dnxt = Distance(vdr.TP[j], vdr.TP[j + 1]);
            dnprv = 0; //no habria distancia anterior
        }
        else
        {
            if (j == (prof_expl - 1)) //Si es el ultimo paso
            {
                angles = Angles_Calculation(vdr.TP[j - 1], vdr.TP[j]); //Calculo final y anterior
                dnxt = 0;                                              //No habria distancia siguiente
            }
            else
            {
                angles = Angles_Calculation(vdr.TP[j - 1], vdr.TP[j], vdr.TP[j + 1]);
                dnxt = Distance(vdr.TP[j + 1], vdr.TP[j]);
            }
            dnprv = Distance(vdr.TP[j], vdr.TP[j - 1]);
        }
        vdr.angles[j] = angles;
        //  cout<<"Angulos: "<<angles[0]<<" "<<angles[1]<<" "<<angles[2]<<endl;
        if (j == prof_expl - 1 || j == 0)
            dm = 1.0 * (dnxt + dnprv);
        else
            dm = 1.0 * (dnxt + dnprv) / 2;

        if (j == 0)
            dm /= 2;

        if (dm >= 0.05)
            dm = 0.05;
        //if (dm<=0.001) dm=0.001;
        //VD.R[j][1]  Es el radio de apertura creciente
        vdr.R[j][2] = 0.002;  //valor de radio  z
        vdr.R[j][0] = 2 * dm; //dm, distancia entre puntos

        double acDist = 1.1;
        for (int k = 0; k <= j; k++)
        {
            acDist += 0.9 * vdr.R[k][0];
        }
        if (acDist == 0)
            acDist = 0.01; //Quitar o revisar valor
        double factorA = ((double(j) * double(j)) / (30 * double(prof_expl) * double(prof_expl)));
        vdr.R[j][1] = 0.01 + factorA + ((acDist * acDist) - 1.14) / 20; //+((j*j*1.0)/5000)
        if (vdr.R[j][1] <= 0.0002)
            vdr.R[j][1] = 0.0002;

        double static_UAV_vel=0.5;
        if (j < prof_expl / 2 && UAV_Velocity < static_UAV_vel)
        {
            vdr.R[j][1] *= 1.1;
        }
        if (j < prof_expl / 3 && UAV_Velocity < static_UAV_vel)
        {
            vdr.R[j][1] *= 1.3;
        }
        if (j < 3 && UAV_Velocity < static_UAV_vel)
        {
            vdr.R[j][1] = 0.04;
        }

        // mtxA.lock();
        // cv::circle(image_Ptraj, cv::Point((vdr.TP[j][0] + maxsc) * scale, (vdr.TP[j][1] + maxsc) * scale), 4, Colors[0], CV_FILLED, 3, 8);
        //  mtxA.unlock();
    }

    //Print("UAV VELOCITY", UAV_Velocity);
    vdr.L = prof_expl;
    return;
}
void RRT::Node_Filter()
{
    if (nodes_reordered)
    {
        OldNodes = EmptyNodes;

        int allowed = 0;
        int cn_Old;
        std::vector<int> del_List;
        double xo, yo, zo, rx, ry, rz, tm;
        /* for(int i = 0; i < prof_expl; i++)
        {
            mtxA.lock();
            cv::circle( image_Ptraj, cv::Point( (nodes.coord[i][0] +maxsc)*scale,(nodes.coord[i][1]+maxsc)*scale ),3, Colors[nodes.region[i]],CV_FILLED,  1, 8 );
            mtxA.unlock();
        }*/
        int allowednodes = 0;

        VectorDbl T1;
        vector<VectorDbl> Rpitch, Rroll, Ryaw;
        for (int i = 2; i < nodes.N; i++) //from prof_expl
        {
            cn_Old = 0;
            /*if (nodes.parent[i]<0 )
            {
                allowed=0;
                Print("node pointing  ", i,nodes.parent[i]);
            }*/
            //  else
            // {
            for (int k = prof_expl - 1; k >= 1; k--)
            {
                Initialize_Inv_Transf_Matrices(Rpitch, Rroll, Ryaw, k);
                xo = nodes.coord[i][0] - vdr.TP[k][0];
                yo = nodes.coord[i][1] - vdr.TP[k][1];
                zo = nodes.coord[i][2] - vdr.TP[k][2];
                VectorDbl pointT{xo, yo, zo};
                T1 = Rotation(pointT, Rpitch, Rroll, Ryaw);
                rx = vdr.R[k][0];
                ry = vdr.R[k][1];
                rz = vdr.R[k][2];
                tm = ((T1[0] / rx) * (T1[0] / rx)) + ((T1[1] / ry) * (T1[1] / ry)) + ((T1[2] / rz) * (T1[2] / rz));

                if (tm <= 1.0)
                {
                    allowed = 1; //Si es permitido, pasar a analizar otro punto
                    allowednodes++;
                    //Print("Adding to oldnode",i);
                    Push_Nodes_Elem_in_Nodes(OldNodes, i);

                    // mtxA.lock();
                    //cv::ellipse(image_Ptraj,cv::Point(image_size/2,image_size/2),cv::Size( scale*rx,  scale*ry),rad_to_deg(0),0,360,Colors[k],1,8);
                    //cv::circle( image_Ptraj, cv::Point( (T1[0] +maxsc)*scale,(T1[1]+maxsc)*scale ), 1, Colors[k],CV_FILLED,  1, 8 );
                    // cv::circle( image_Ptraj, cv::Point( (nodes.coord[i][0] +maxsc)*scale,(nodes.coord[i][1]+maxsc)*scale ), 1, Colors[k],CV_FILLED,  1, 8 );
                    //mtxA.unlock();
                    break;
                }
                else
                {
                    allowed = 0;
                }
                // }
            }
            if (allowed == 0) //Si es un punto rechazado
            {
                del_List.push_back(i);
            }
        }
        // Print("Nodos antiguos", OldNodes.N);
        // Print("allowed, existent",allowednodes,nodes.N,nodes.coord.size());
        //Print("Old Nodes Size First", OldNodes.N);
        //filtrado por trayectoria, puntos no necesarios de la antigua trayectoria
        /*for (int i = TrajNodesIncluded; i < prof_expl; i++)
        {
            del_List.push_back(i);  //se elimina puntos de trayectoria y sus branches
        }*/
        //auto temp_tic = Clock::now();
        //Ahora eliminar los puntos en la lista y sus respectivas ramas
        // for (int i = 0; i < del_List.size(); i++)
        // {
        //     delete_branch(del_List[i]);
        // }

        //Print("Nodes size", nodes.N);
        //Draw_RRT();
        //Print("TIEMPO FILTER A1",toc(temp_tic).count());
    }
    return;
}

inline void RRT::delete_branch(int indx)
{
    std::vector<int> parents;
    parents.push_back(indx); //all the branch with this parent is eliminated
    std::vector<int> indxlist;
    Nodes nodestemp1 = nodes;
    Nodes Fin_Nodes;

    int ln = nodes.N;
    int k = 0;
    bool found = false;
    std::vector<int> valid_parents;
    valid_parents.resize(ln);
    while (k < ln)
    {
        found = false;
        for (int j = 0; j < parents.size(); j++)
        {
            if (parents[j] == nodestemp1.parent[k] && nodestemp1.parent[k] >= -1)
            {
                parents.push_back(k);
                //Push_Nodes_Elem_in_Nodes(OldNodes, nodes.id[k]);
                nodestemp1.parent[k] = -100; //borro el valor de parent para que no vuelva a caer aqui
                found = true;
            }
        }
        if (found == true)
        {
            k = -1;
        } //reinicio desde cero el bucle para revisar todos los nodos otra vez4
        k++;
    }
    //Ahora quitar los nodos invalidos, y dejar los nodos permitidos unicamente.

    const int maxnodes = ln;
    Fin_Nodes.N = 0;

    int fcn = 0;
    int badfound = 0;

    for (int i = 0; i < ln; i++)
        valid_parents[i] = i;

    //erase parents elements from valid parents vector
    for (int i = 0; i < parents.size(); i++)
        valid_parents.erase(std::remove(valid_parents.begin(), valid_parents.end(), parents[i]), valid_parents.end());

    int i;
    for (int j = 0; j < valid_parents.size(); j++)
    {
        i = valid_parents[j];

        Fin_Nodes.coord.push_back(nodes.coord[i]);
        Fin_Nodes.coordT.push_back(nodes.coordT[i]);
        Fin_Nodes.cost.push_back(nodes.cost[i]);
        Fin_Nodes.costParent.push_back(nodes.costParent[i]);
        Fin_Nodes.parent.push_back(nodes.parent[i]); //nodos permitidos van ordenados en Fin_Nodes
        Fin_Nodes.region.push_back(nodes.region[i]);
        Fin_Nodes.id.push_back(fcn);
        Fin_Nodes.N = fcn + 1;

        indxlist.push_back(i); //orden en la lista seria el indice nuevo y valor es el indice antiguo
        fcn++;
    }
    //Ahora se corrige los parents de los nodos finales, para que apunten al nodo correcto
    //es decir, buscar si hay cambios en el indice de un nodo, y si los hay, buscar nodos hijos y corregirles el parent.
    for (int i = 0; i < fcn; i++)
    {
        if (i != indxlist[i])
        {
            for (int j = 0; j < fcn; j++)
            {
                if (Fin_Nodes.parent[j] == indxlist[i])
                {
                    Fin_Nodes.parent[j] = i;
                }
            }
        }
    }
    //Print(" ----DB 4---- #nodos al entrar y salir de deletebranch: ",ln,Fin_Nodes.N);
    nodes = Fin_Nodes; //Fin_nodes seria el arreglo de nodos filtrado
    return;
}

void RRT::Nodes_Reorder()
{
    //NUEVOS NODOS DE TRAYECTORIA

    for (int j = 0; j < TrajNodesIncluded; j++) // tr_brk son los nuevos, requieren inicializar hijos, solo para los puntos de trayectoria
    {
        //Inicializacion
        nodes.coord[j] = vdr.TP[j];
        nodes.coordT[j] = vdr.TP[j];

        nodes.id[j] = j;
        if (j == 0)
        {
            nodes.parent[j] = -1;
            nodes.cost[j] = 0;
            nodes.costParent[j] = 0;
        }
        else
        {
            //Print("reorder init j no 00",0);
            nodes.parent[j] = j - 1;
            //Print("nodes reorder cost",double(j), nodes.cost[j-1]);
            nodes.costParent[j] = Distance(vdr.TP[j - 1], vdr.TP[j]);
            nodes.cost[j] = nodes.costParent[j] + nodes.cost[j - 1]; //Aqui tiene que calcularse en funcion de la pose actual del eeff.tambien se puede calcular acumulando paso a paso
        }
        nodes.region[j] = j;
    }
    //ANTIGUOS NODOS DE TRAYECTORIA, REQUIEREN SWEEP Y ACTUALIZACION DE PADRES

    /*for (int j = 0; j < 0; j++) //  tr_brk   son los nuevos, requieren inicializar hijos, solo para los puntos de trayectoria
    {
        if (nodes_reordered == 1) //Revisar restriciones&& tr_brk < prof_expl-1
        {

            nodes.coord[j] = vdr.TP[j];
            nodes.cost[j] = nodes.cost[j + 1];
            nodes.costParent[j] = nodes.costParent[j + 1];
            nodes.parent[j] = nodes.parent[j + 1];
            nodes.id[j] = j;

            if (j == 0)
            {
                nodes.parent[j] = -1;
                nodes.cost[j] = 0;
                nodes.costParent[j] = 0;
            }
            else
            {
                nodes.parent[j] = j - 1;
                nodes.costParent[j] = Distance(vdr.TP[j - 1], vdr.TP[j]) + nodes.cost[j - 1];
                nodes.cost[j] = nodes.costParent[j] + nodes.cost[j - 1];
            }
            nodes.region[j] = j;
        }
    }*/
    if (nodes.N <= TrajNodesIncluded)
        nodes.N = TrajNodesIncluded;
    //if (nodes.N < prof_expl-2) nodes.N = prof_expl;//ya que los nodos estan inicializados

    nodes_reordered = 1;
    return;
}
//===============================================RRT======================================================================
//========================================================================================================================
inline void RRT::RRT_Generation()
{
    int oldSize = nodes.N;
    int NumNodesToAdd_reduced = NumNodesToAdd;
    //Print("********//Nodes size Start", nodes.N);
    /* if (oldSize > 350)
    {
        NumNodesToAdd_reduced /= 5;
        if (oldSize > 400)
        {
            NumNodesToAdd_reduced /= 7;
        }
        if (oldSize > 500)
        {
            NumNodesToAdd_reduced = 0; //Control of number of nodes after filtering and before rrt generation
        }
    }*/
    //Text_Stream->write_TimeStamp();
    int num_requests = 0;
    auto ticA = std::chrono::high_resolution_clock::now();
    double num_nodes_to_add = double(NumNodesToAdd_reduced);
    if (NumNodesToAdd_reduced != 0)
    {
        for (int j = 0; j < prof_expl; j++) //(int j=prof_expl-1;j >= 0 ;j--)
        {
#ifdef OPENCV_DRAW
            cv::ellipse(image_Ptraj, cv::Point(Img(vdr.TP[j][0]), Img(vdr.TP[j][1])), cv::Size(scale * vdr.R[j][0], scale * vdr.R[j][1]), rad_to_deg(vdr.angles[j][0]), 0, 360, Colors[j], 1, 8);
#endif
            num_nodes_to_add = double(NumNodesToAdd_reduced);
            if (j >= 0 && j <= 4)
                num_nodes_to_add *= 2.0;

            num_requests += Add_Node(j, num_nodes_to_add); //agrega N nodos cada vez
        }
    }
    // Print("Nodes feeding time", ArmModel.toc(ticA).count());
    // auto tic = std::chrono::high_resolution_clock::now();
    RetrieveNodes(num_requests);
    //Print("Retrieving time", ArmModel.toc(tic).count());

    //  Print("NodesOld size, new size", oldSize, nodes.N);
    //Stretch_the_Cord();
    Draw_RRT();

    return;
}
//===================================================================================================================================
//===================================================================================================================================

int RRT::Add_Node(int It, int num_nodes)
{
    //Print("ADD nodes region", It);
    double rx = vdr.R[It][0]; //revisar
    double ry = vdr.R[It][1];
    double rz = vdr.R[It][2];
    VectorDbl rnTemp1(3), rnTemp_T(3); //poque despues en chequeo de colisiones se usa unicamente los 3 valores de posicion
    bool found_ik = 0, allwd = 0;
    VectorDbl q_rand(3);
    //====================== Creacion de puntos random centrados en cero con sus respectivos radios =========================================================
    std::random_device rdx;
    std::random_device rdy;
    std::random_device rdz;
    std::mt19937 genx(rdx());
    std::mt19937 geny(rdy());
    std::mt19937 genz(rdz());
    int try_count = 0;
    double tm = 100;
    bool ran_int = true;
    int prcsd = 20000000, prcs = (int)(prcsd / 2);
    /*
    int dnx=round(rx*prcsd);
    int dny=round(ry*prcsd);
    int dnz=round(rz*prcsd);
        std::uniform_int_distribution<> distxr(1,dnx);//uniform_real_distribution
        std::uniform_int_distribution<> distyr(1,dny);
        std::uniform_int_distribution<> distzr(1,dnz);
    
    */

    std::uniform_real_distribution<double> distxr(-rx * prcs, rx * prcs); //uniform_real_distribution
    std::uniform_real_distribution<double> distyr(-ry * prcs, ry * prcs);
    std::uniform_real_distribution<double> distzr(-rz * prcs, rz * prcs);
    /*    
    std::normal_distribution<double> distxr(-rx, rx); //uniform_real_distribution
    std::normal_distribution<double> distyr(-ry, ry);
    std::normal_distribution<double> distzr(-rz, rz);
*/
    //Print("Radios",rx,ry,rz);
    //Print("Maximum " , xmax,ymax,zmax);
    int max_tries = 5;
    int max_rnd_tries = 5;
    double rnx, rny, rnz;
    int request_counter = 0;
    int limit_requests = num_nodes + max_tries - 1;

    while (request_counter < limit_requests)
    {
        if (try_count > max_tries)
        {
            break;
        }
        tm = 100;
        bool rnd_point_found = false;
        int rnd_point_counts = 0;
        while (tm > 1)
        {
            //rnd_point_counts++;
            if (rnd_point_counts > max_rnd_tries)
            {
                rnd_point_found = false;
                Print("fail, too many rand tries");
                break;
            }

            rnx = distxr(genx);
            rny = distyr(geny);
            rnz = distzr(genz);
            //Print("rnx",rnx,rny,rnz);
            //rnx -= rx*prcs-1;
            rnx /= prcs;
            // rny -= ry*prcs-1;
            rny /= prcs;
            //rnz -= rz*prcs-1;
            rnz /= prcs;
            /*
            rnx = distxr(genx);
            rny = distyr(geny);
            rnz = distzr(genz);
 */
            q_rand[0] = rnx;
            q_rand[1] = rny;
            q_rand[2] = rnz;
            tm = ((rnx / rx) * (rnx / rx)) + ((rny / ry) * (rny / ry)) + ((rnz / rz) * (rnz / rz));
        }

        //======================================================================================================================================================
        //Transformaciones, rotacion y traslacion
        bool found_ik_tmp = false;
        vector<std::vector<double>> Rpitch, Rroll, Ryaw;
        Initialize_Transf_Matrices(Rpitch, Rroll, Ryaw, It);
        rnTemp1 = Transform(q_rand, It, Rpitch, Rroll, Ryaw); //First rotate then translate
        rnTemp_T = Translation(q_rand, It);                   //Only trtaslation
        allwd = Check_Boundaries(rnTemp1);

        if (allwd == 1)
        {
            //======Chequeo de colisiones===========================================
            std::vector<double> tempPosit(3);
            tempPosit[0] = rnTemp1[0];
            tempPosit[1] = rnTemp1[1];
            tempPosit[2] = rnTemp1[2];
            //Print("Feed, try count, IT",try_count, It,request_counter);

            ArmModel.FeedCollisionCheck_Queue(tempPosit, rnTemp_T, It);
            request_counter++;
        }
        else
        {
            try_count++;
        }
    }
    if (request_counter == 0 && try_count >= max_tries)
    {
        Print("==ERROR, no random point found around this position");
    }

    return request_counter;
}

void RRT::RetrieveNodes(int request_counter)
{
    std::vector<PositionResults> results_;
    if (request_counter > 0)
    {
        for (int ic = 0; ic < request_counter; ic++)
        {
            // Print("Retrieving",i,request_counter);
            std::pair<bool, PositionResults> result = ArmModel.RetrieveResults();
            if (result.first)
            {
                results_.push_back(result.second);
                RRT_AddValidCoord(result.second.Position, result.second.Position_Only_T, result.second.region);
            }
        }
        Print("Total Requests Sent vs Results Received", request_counter, results_.size());
        //Print("Input Output sizes", ArmModel.eeff_positions_input_queue.size(), ArmModel.eeff_positions_results.size());
        if (results_.size() == 0)
            Print("ERROR No points found by IK for this region");
        //  Print("Input Queue Size",ArmModel.eeff_positions_input_queue.size(),request_counter,results_.size() );
        //  Print("Output Queue Size",ArmModel.eeff_positions_results.size());

        //Print("Difference ",request_counter-results_.size());
        //Print("ADDING RESULTS",results_.size());
    }
    return;
}

void RRT::RRT_AddOldCoords()
{
    double tm;
    bool allowed;
    VectorDbl ON_B(3);
    double rx, ry, rz, ON_x, ON_y, ON_z;
    int region;
    int old = 0;
    Print("add old nodes");
    for (int i = 0; i < prof_expl; i++)
        Old_Nodes_Added_Reg[i] = 0;

    for (int on = 0; on < OldNodes.N; on++)
    {
        //if (OldNodes.id[on]>prof_expl)
        //{
        tm = 100.0;
        allowed = false;
        ON_B[0] = OldNodes.coord[on][0];
        ON_B[1] = OldNodes.coord[on][1];
        ON_B[2] = OldNodes.coord[on][2];

        for (int It = 1; It < prof_expl; It++)
        {
            vector<VectorDbl> Rpitch, Rroll, Ryaw;
            Initialize_Inv_Transf_Matrices(Rpitch, Rroll, Ryaw, It);
            rx = vdr.R[It][0]; //revisar
            ry = vdr.R[It][1];
            rz = vdr.R[It][2];
            ON_x = ON_B[0] - vdr.TP[It][0];
            ON_y = ON_B[1] - vdr.TP[It][1];
            ON_z = ON_B[2] - vdr.TP[It][2];
            VectorDbl pointT{ON_x, ON_y, ON_z};
            VectorDbl Temp1 = Rotation(pointT, Rpitch, Rroll, Ryaw);
            tm = ((Temp1[0] / rx) * (Temp1[0] / rx)) + ((Temp1[1] / ry) * (Temp1[1] / ry)) + ((Temp1[2] / rz) * (Temp1[2] / rz));
            // cv::circle( image_Ptraj, cv::Point( (Temp1[0] +maxsc)*scale,(Temp1[1]+maxsc)*scale ), 1, Colors[0],CV_FILLED,  4, 8 );
            // Print("OLD ALLOWED",ON_B[0],ON_B[1],tm,vdr.TP[It][0],vdr.TP[It][1],ON_x,ON_y);
            // Print("OLD ALLOWED",tm);
            if (tm <= 1)
            {
                Old_Nodes_Added_Reg[It]++;
                //cv::circle( image_Ptraj, cv::Point( (ON_B[0] +maxsc)*scale,(ON_B[1]+maxsc)*scale ), 1, Colors[0],CV_FILLED,  4, 8 );
                allowed = true;
                region = It;
                break;
            }
        }
        Print("lets add", on, allowed, region);
        if (allowed)
        {
            //Print("test2");
            if (Old_Nodes_Added_Reg[region] <= MaxOldNodesReg * 2)
            {
                RRT_AddValidCoord(OldNodes.coord[on], OldNodes.coordT[on], region);
                old++;
            }
        }
        //}
    }
    Print("old nodes added", old);
    return;
}
inline void RRT::RRT_AddValidCoord(VectorDbl q_rand_TR, VectorDbl q_randA_T, int It)
{
    // AQUI EMPIEZA RRT
    double tmp_dist;
    //Hallar el minimo
    std::vector<double> temp_coords(3);
    //Search
    double min_ndist = 1000000.0;
    int index_near = It;
    if (index_near >= TrajNodesIncluded)
        index_near = TrajNodesIncluded - 1;
    int co = 0;
    for (int k = 0; k < nodes.N; k++)
    {
        if (true || nodes.region[k] >= It) // && k<2 && k>=prof_expl)//&&nodes.region[k]<=It+1)
        {
            tmp_dist = Distance(q_rand_TR, nodes.coord[k]);
            if (tmp_dist <= min_ndist)
            {
                min_ndist = tmp_dist;
                index_near = k;
            }
        }
    }
    Node q_near, q_new;
    Extract_Node_from_Nodes(q_near, nodes, index_near); //almacenar en q_near el nodo mas cercano al punto q_new
    //Funcion steer
    q_new.coord = steer(q_rand_TR, q_near.coord, min_ndist, EPS);
    q_new.cost = Distance(q_new.coord, q_near.coord); // + q_near.cost;
    //Buscar dentro del radio r los nodos mas cercanos con costo menor al punto q_new, para que se convierta en su nuevo padre
    Node q_min = q_near;
    double C_min = q_new.cost;
    int q_min_Indx = index_near;
    for (int j = 0; j < nodes.N; j++)
    {
        //Print("region",nodes.region[j]);
        double Dist_node_to_qnew = Distance(nodes.coord[j], q_new.coord);
        if (Dist_node_to_qnew <= r) //Si esta dentro del circulo de radio r, se considera un nearest
        {
            double Cost_q_nearest = Dist_node_to_qnew + nodes.cost[j];
            if (Cost_q_nearest < C_min)
            {
                Extract_Node_from_Nodes(q_min, nodes, j); //q_min seria el que tiene menor costo, es decir el nuevo padre de q_new
                C_min = Cost_q_nearest;
                q_min_Indx = j;
                break;
            }
        }
    }
    //Print("DIST TIME",toc(ttic).count(),nodes.N,index_near,q_min_Indx);
    //Print("Point to add or not",q_rand[0],q_rand[1]);
    //Aqui q_min es el nodo mas optimo, que forma el camino mas corto
    //Update parent to least cost-from node.
    q_new.parent = q_min_Indx;
    //Ahora hacer steer otra vez para cumplir con las restricciones de dist max de movimiento
    double val = Distance(q_new.coord, q_min.coord);
    Node q_new_f;
    q_new_f.coord = q_new.coord; //steer(q_new.coord,q_min.coord,val,EPS);
    q_new_f.coordT = q_randA_T;
    q_new_f.costParent = Distance(q_new_f.coord, q_min.coord);
    q_new_f.cost = q_new_f.costParent + q_min.cost;
    q_new_f.parent = q_min_Indx;
    q_new_f.id = nodes.N;
    q_new_f.region = It;
    Text_Stream->write_Data(q_new_f.coord);
    Text_Stream->jump_line();
    Insert_Node_in_Nodes(nodes, nodes.N + 1, q_new_f); //Insertar nodo al final de la lista nodes, internamente se aumenta el valor de nodes.N
                                                       //Print("Node added",q_new_f.coord[0],q_new_f.coord[1],q_new_f.coord[2]);
#ifdef OPENCV_DRAW
    // mtxA.lock();
    //cv::line( image_Ptraj, cv::Point((q_new_f.coord[0]+maxsc)*scale,(q_new_f.coord[1]+maxsc)*scale ),cv::Point((q_min.coord[0]+maxsc)*scale,(q_min.coord[1]+maxsc)*scale ),  cv::Scalar( 00, 230, 50 ),  1, 8 );
    //cv::circle( image_Ptraj, cv::Point( (q_new_f.coord[0] +maxsc)*scale,(q_new_f.coord[1]+maxsc)*scale ), 1, Colors[It],CV_FILLED,  1, 8 );
    //int stw=3;
    //cv::line( image, cv::Point((q_new_f.coord[0] -vdr.TP[It][0] +maxsc/stw)*stw*scale,(q_new_f.coord[1]-vdr.TP[It][1]+maxsc/stw)*stw*scale ),cv::Point((q_min.coord[0]-vdr.TP[It][0]+maxsc/stw)*stw*scale,(q_min.coord[1]-vdr.TP[It][1]+maxsc/stw)*stw*scale ),  cv::Scalar( 00, 230, 50 ),  1, 8 );

    //cv::circle( image, cv::Point( (q_new_f.coord[0] -vdr.TP[It][0] +maxsc/stw)*stw*scale,(q_new_f.coord[1]-vdr.TP[It][1]+maxsc/stw)*stw*scale ), 2, Colors[It],CV_FILLED,  1,8 );

    // cv::imshow("ImagepTraj",image_Ptraj);
    //cv::waitKey(1);
    // mtxA.unlock();
#endif
    return;
}

void RRT::Extract_Node_from_Nodes(Node &node, Nodes nodes, int nIndx)
{
    //Extrae un nodo del arreglo de nodos, para almacenarlo en una estructura Nodo
    node.coord.resize(3);
    node.coordT.resize(3);
    int i = 0;
    node.coord = nodes.coord[nIndx];
    node.coordT = nodes.coordT[nIndx];
    node.cost = nodes.cost[nIndx];
    node.costParent = nodes.costParent[nIndx];
    node.parent = nodes.parent[nIndx];
    node.id = nodes.id[nIndx];
    node.region = nodes.region[nIndx];

    return;
}
void RRT::Insert_Node_in_Nodes(Nodes &nodes, int nIndx, Node node)
{
    //Agregar un nodo al vector de nodos en la posicion nIndx
    if (nIndx > nodes.N)
    {
        nodes.coord.resize(nIndx);
        nodes.coordT.resize(nIndx);
        nodes.cost.resize(nIndx);
        nodes.costParent.resize(nIndx);
        nodes.id.resize(nIndx);
        nodes.parent.resize(nIndx);
        nodes.region.resize(nIndx);
        nodes.N = nodes.N + 1;
    }
    nodes.coord[nIndx - 1] = node.coord;
    nodes.coordT[nIndx - 1] = node.coordT;
    nodes.cost[nIndx - 1] = node.cost;
    nodes.costParent[nIndx - 1] = node.costParent;
    nodes.id[nIndx - 1] = node.id;
    nodes.parent[nIndx - 1] = node.parent;
    nodes.region[nIndx - 1] = node.region;
    return;
}
void RRT::Push_Nodes_Elem_in_Nodes(Nodes &nodesR, int indxG)
{
    //if(indxG>=0 )
    nodesR.coord.push_back(nodes.coord[indxG]);
    nodesR.coordT.push_back(nodes.coordT[indxG]);
    nodesR.cost.push_back(nodes.cost[indxG]);
    nodesR.costParent.push_back(nodes.costParent[indxG]);
    nodesR.parent.push_back(nodes.parent[indxG]);
    nodesR.id.push_back(nodes.id[indxG]);
    nodesR.region.push_back(nodes.region[indxG]);
    nodesR.N++;
    return;
}

VectorDbl RRT::steer(VectorDbl qr, VectorDbl qn, double min_ndist, double EPS)
{
    VectorDbl A(3);
    if (min_ndist >= EPS) //||min_ndist<=EPS/10)
    {
        A[0] = qn[0] + (((qr[0] - qn[0]) * EPS) / min_ndist);
        A[1] = qn[1] + (((qr[1] - qn[1]) * EPS) / min_ndist);
        A[2] = qn[2] + (((qr[2] - qn[2]) * EPS) / min_ndist);
        //Print("nodes unsteered",qn[0],qn[1],qn[2]);
        //Print("nodes old      ",qr[0],qr[1],qr[2]);
        //Print("nodes steered  ",A[0],A[1],A[2]);
    }
    else
    {
        A[0] = qr[0];
        A[1] = qr[1];
        A[2] = qr[2];
    }
    return A;
}

control_msgs::FollowJointTrajectoryGoal RRT::SteerJoints(control_msgs::FollowJointTrajectoryGoal goal)
{
    if (goal.trajectory.points.size() > 0)
    {
        double JointEPS =  80*EPS;
        auto joints = ArmModel.getCurrentJoints();
        int ReqJointsSize = goal.trajectory.points[0].positions.size();
        int CurrentJointsSize = joints.size();
        if (CurrentJointsSize != ReqJointsSize)
            Print("Joints sizes are not the same RRT functions line 773");
        else
        {
            int joint_cn = 0;
            double diff;
            for (auto req_joint : goal.trajectory.points[0].positions)
            {
                auto req_jointT = req_joint;
                diff = req_joint - joints[joint_cn];
                if (diff < -JointEPS)
                    diff = -JointEPS;
                if (diff > JointEPS)
                    diff = JointEPS;
                req_joint = joints[joint_cn] + diff;
                goal.trajectory.points[0].positions[joint_cn]=req_joint;
              //  Print("Joint", joints[joint_cn],req_jointT,req_joint );
                joint_cn++;
            }
        }
    }
    return goal;
}

void RRT::Draw_RRT()
{
    //Print("size",nodes.coord.size(),nodes.N);
#ifdef OPENCV_DRAW
    for (int i = 0; i < nodes.N; i++)
    {
        const int parent = nodes.parent[i];
        // Print("parent",parent, i);
        //  mtxA.lock();
        if (parent != -1)
            cv::line(image_Ptraj, cv::Point((nodes.coord[i][0] + maxsc) * scale, (nodes.coord[i][1] + maxsc) * scale), cv::Point((nodes.coord[parent][0] + maxsc) * scale, (nodes.coord[parent][1] + maxsc) * scale), cv::Scalar(00, 230, 50), 1, 8);
        cv::circle(image_Ptraj, cv::Point((nodes.coord[i][0] + maxsc) * scale, (nodes.coord[i][1] + maxsc) * scale), 1, Colors[nodes.region[i]], CV_FILLED, 1, 8);

        // mtxA.unlock();
        // cv::imshow("ImagepTraj",image_Ptraj);
        //       cv::waitKey(1);
        /* VectorDbl Temp1{nodes.coord[i][0] ,nodes.coord[i][1] };
       int It=nodes.region[i];
        int rx = vdr.R[It][0]; //revisar
                int ry = vdr.R[It][1];
                int rz = vdr.R[It][2];
                       double  tm = ((Temp1[0]/rx)*(Temp1[0]/rx))+((Temp1[1]/ry)*(Temp1[1]/ry));
                       if (tm>1.0) Print("parent",parent);*/
    }
#endif
}

void RRT::Initialize_Transf_Matrices(vector<VectorDbl> &Rpitch, vector<VectorDbl> &Rroll, vector<VectorDbl> &Ryaw, int &It)
{
    Rpitch.resize(3);
    Rroll.resize(3);
    Ryaw.resize(3);
    for (int i = 0; i < 3; i++)
    {
        Rpitch[i].resize(3);
        Rroll[i].resize(3);
        Ryaw[i].resize(3);
        for (int j = 0; j < 3; j++)
        {
            Rpitch[i][j] = 0;
            Rroll[i][j] = 0;
            Ryaw[i][j] = 0;
        }
    }
    //yaw,pitch,roll, ordenados
    Rpitch[0][0] = 1.0; /*,  0                                ,            0   */
    /*0, */ Rpitch[1][1] = cos(vdr.angles[It][1]);
    Rpitch[1][2] = -sin(vdr.angles[It][1]); //X rotation
    /*0, */ Rpitch[2][1] = sin(vdr.angles[It][1]);
    Rpitch[2][2] = cos(vdr.angles[It][1]);

    Rroll[0][0] = cos(vdr.angles[It][2]);                     /*,      0            ,*/
    Rroll[0][2] = sin(vdr.angles[It][2]);                     //Y rotation
    /*    0                            ,*/ Rroll[1][1] = 1.0; /* ,                0   */
    Rroll[2][0] = -sin(vdr.angles[It][2]);                    /*,      0            ,*/
    Rroll[2][2] = cos(vdr.angles[It][2]);

    Ryaw[0][0] = cos(vdr.angles[It][0]);
    Ryaw[0][1] = -sin(vdr.angles[It][0]); /*,       0*/ //Z rotation
    Ryaw[1][0] = sin(vdr.angles[It][0]);
    Ryaw[1][1] = cos(vdr.angles[It][0]); /*,       0*/
    /* ,         0                   ,                0                     ,*/ Ryaw[2][2] = 1.0;
    /*vector<std::vector<double> > R=Rroll;
    cout<<" Matriz "<<R[0][0]<<" "<<R[0][1]<<" "<<R[0][2]<<endl;
    cout<<" Matriz "<<R[1][0]<<" "<<R[1][1]<<" "<<R[1][2]<<endl;
    cout<<" Matriz "<<R[2][0]<<" "<<R[2][1]<<" "<<R[2][2]<<endl;*/
}
void RRT::Initialize_Inv_Transf_Matrices(vector<VectorDbl> &Rpitch, vector<VectorDbl> &Rroll, vector<VectorDbl> &Ryaw, int &It)
{
    Rpitch = emptyMatrix;
    Rroll = emptyMatrix;
    Ryaw = emptyMatrix;

    double InvPitch = 0.0 - vdr.angles[It][1];
    double InvRoll = 0.0 - vdr.angles[It][2];
    double InvYaw = 0.0 - vdr.angles[It][0];

    //yaw,pitch,roll, ordenados
    Rpitch[0][0] = 1.0; /*,  0                       ,            0                  */
    /*0, */ Rpitch[1][1] = cos(InvPitch);
    Rpitch[1][2] = -sin(InvPitch); //X rotation
    /*0, */ Rpitch[2][1] = sin(InvPitch);
    Rpitch[2][2] = cos(InvPitch);

    Rroll[0][0] = cos(InvRoll);                        /*,         0            ,*/
    Rroll[0][2] = sin(InvRoll);                        //Y rotation
    /*    0                     ,*/ Rroll[1][1] = 1.0; /* ,                0         */
    Rroll[2][0] = -sin(InvRoll);                       /*,           0          ,*/
    Rroll[2][2] = cos(InvRoll);

    Ryaw[0][0] = cos(InvYaw);
    Ryaw[0][1] = -sin(InvYaw); /*,       0*/ //Z rotation
    Ryaw[1][0] = sin(InvYaw);
    Ryaw[1][1] = cos(InvYaw); /*,       0*/
    /* ,         0        ,           0           ,*/ Ryaw[2][2] = 1.0;
    return;
}

VectorDbl RRT::Transform(VectorDbl Point, int It, vector<VectorDbl> &Rpitch, vector<VectorDbl> &Rroll, vector<VectorDbl> &Ryaw)
{
    VectorDbl temp(3);
    //Inicializacion de Matrices
    temp = Matrix_Vector_MultiplyA(Rpitch, Point);
    temp = Matrix_Vector_MultiplyA(Rroll, temp);
    temp = Matrix_Vector_MultiplyA(Ryaw, temp);
    temp[0] += vdr.TP[It][0];
    temp[1] += vdr.TP[It][1];
    temp[2] += vdr.TP[It][2];

    //     or eliminate the variable.
    return temp;
}

inline VectorDbl RRT::Translation(VectorDbl Point, int It)
{
    VectorDbl temp(3);
    temp = Point;
    temp[0] += vdr.TP[It][0];
    temp[1] += vdr.TP[It][1];
    temp[2] += vdr.TP[It][2];

    return temp;
}
inline VectorDbl RRT::Rotation(VectorDbl Point, vector<VectorDbl> Rpitch, vector<VectorDbl> Rroll, vector<VectorDbl> Ryaw)
{
    VectorDbl temp(3);
    //Inicializacion de Matrices
    temp = Matrix_Vector_MultiplyA(Rpitch, Point);
    temp = Matrix_Vector_MultiplyA(Rroll, temp);
    temp = Matrix_Vector_MultiplyA(Ryaw, temp);

    //     or eliminate the variable.
    return temp;
}
inline VectorDbl RRT::Matrix_Vector_MultiplyA(vector<VectorDbl> Matrix, VectorDbl Vector)
{
    VectorDbl mult(3);
    for (int i = 0; i < 3; ++i)
    {
        mult[i] = 0.0;
        for (int k = 0; k < 3; ++k)
        {
            mult[i] += (Matrix[i][k] * Vector[k]);
        }
    }
    return mult;
}

inline bool RRT::Check_Boundaries(VectorDbl Point)
{
    double cat1, cat2, offx, offy;
    offx = 0.0; //just in case the arm is not centered in the origin
    offy = 0.0;
    cat1 = Point[0] - offx;
    cat2 = Point[1] - offy;
    double rad = sqrt((cat1 * cat1) + (cat2 * cat2));

    if (rad < r_exterior && rad > r_interior)
    { //dentro del area permitida Circulo externo-circulo interno
        return true;
    }
    else
    {
        return false;
    }
}

VectorDbl RRT::Angles_Calculation(VectorDbl P0, VectorDbl P1)
{
    VectorDbl angles;
    double dx1 = P1[0] - P0[0];
    double dy1 = P1[1] - P0[1];
    double dz1 = P1[2] - P0[2];

    double dxy1 = sqrt((dx1 * dx1) + (dy1 * dy1));
    double yaw1 = atan2(dy1, dx1); //+M_PI_2;
    double pitch1 = atan2(dz1, dxy1);
    double roll = 0;
    angles.push_back(yaw1);
    angles.push_back(pitch1);
    angles.push_back(roll);
    return angles;
}
inline VectorDbl RRT::Angles_Calculation(VectorDbl P0, VectorDbl P1, VectorDbl P2) //Overloaded
{
    std::vector<double> angles;
    double dx1 = P1[0] - P0[0];
    double dy1 = P1[1] - P0[1];
    double dz1 = P1[2] - P0[2];
    double dxy1 = sqrt((dx1 * dx1) + (dy1 * dy1));
    double yaw1 = atan2(dy1, dx1);    //+M_PI_2;
    double pitch1 = atan2(dz1, dxy1); //(dxy1,dz1);
    double roll1 = 0;

    double dx2 = P2[0] - P1[0];
    double dy2 = P2[1] - P1[1];
    double dz2 = P2[2] - P1[2];

    double dxy2 = sqrt((dx2 * dx2) + (dy2 * dy2));
    double yaw2 = atan2(dy2, dx2); //+M_PI_2;
    double pitch2 = atan2(dz2, dxy1);
    double roll2 = 0;

    double yaw = (yaw1 + yaw2) / 2; //mean values
    double pitch = (pitch1 + pitch2) / 2;
    double roll = (roll1 + roll2) / 2;
    angles.push_back(yaw);
    angles.push_back(pitch);
    angles.push_back(roll);
    return angles;
}
inline double RRT::Distance(VectorDbl P0, VectorDbl P1)
{
    return sqrt(((P1[0] - P0[0]) * (P1[0] - P0[0])) + ((P1[1] - P0[1]) * (P1[1] - P0[1])) + ((P1[2] - P0[2]) * (P1[2] - P0[2])));
    //return  sqrt(((P1[0]-P0[0])*(P1[0]-P0[0]))+ ((P1[1]-P0[1])*(P1[1]-P0[1])));
}

void RRT::RRT_SequenceB() //extraer vecindad
{                         //tic();

    finish = false;
    nodes = EmptyNodes;
    //Print("//-------RRt3 InitVicinity-----------");
    //auto tic = std::chrono::high_resolution_clock::now();

    Initialize_VicinityRRT();
    //Print("Initialize Vcnty time", ArmModel.toc(tic).count());
    //tic();
    //Print("//-------RRt4 NodelFilter------------");
    //Node_Filter();
    //tic = std::chrono::high_resolution_clock::now();
    //Print("BBBtiempo Node Filter",toc().count());
    //tic();
    //Print("//-------RRt5 NodesReorder-----------");
    Nodes_Reorder();
    // Print("Nodes Reorder    time", ArmModel.toc(tic).count());
    auto tic = std::chrono::high_resolution_clock::now();
    //Print("BBtiempo Nodes Reorder",toc().count());
    //tic();
    //Print("//-----RRt6 RRTGEN-----------------");
    RRT_Generation();
    //  Print("RT Generation    time", ArmModel.toc(tic).count());

    //Print("//-------RRt7 Finish-----------------");
    //Print("BBtiempo RRT Gen",toc().count());
    finish = true;
    return;
}

void RRT::loop_start()
{
    // std::unique_lock<std::mutex> lck(mtx);
    sequence_loop = true;
    //cv.notify_all();
    return;
}
void RRT::loop_end()
{ //std::unique_lock<std::mutex> lck(mtx);
    sequence_loop = false;
    // cv.notify_all();
}

inline int RRT::Img(double point)
{
    point += maxsc;
    point *= scale;
    return int(point);
}
inline double RRT::rad_to_deg(double rad)
{
    double deg = rad * 180 / PI;
    return deg;
}

#endif
