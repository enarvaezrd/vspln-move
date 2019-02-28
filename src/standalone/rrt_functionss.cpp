#ifndef RRT_PLANIF_CODE
#define RRT_PLANIF_CODE
#include"rrt_functionss.hpp"
using namespace rrt_planif;


void RRT::Initialize_VicinityRRT()
{
    std::vector<double> angles(3);
    double dnprv,dnxt,dm;
    for (int j=0;j<prof_expl;j++)
    {
        vdr.TP[j][0]=Tr.xval[j];//Se carga la trayectoria predicha en esta iteracion, a los valores de trayectoria nuevos
        vdr.TP[j][1]=Tr.yval[j];
        vdr.TP[j][2]=Tr.zval[j];
    }
        for (int j=0;j<prof_expl;j++) //desde tr_brk hasta el ultmo valor de prof_e (7 que es el octavo valor), ultimo valor de trajectoria predicha
        {
            if (j==0)    //Si es el primer paso
            {
                angles=Angles_Calculation(vdr.TP[j],vdr.TP[j+1]);//calcular inicial y siguiente
                dnxt=Distance(vdr.TP[j],vdr.TP[j+1]);
                dnprv=0; //no habria distancia anterior
            }
            else
            {
                if (j==(prof_expl-1)) //Si es el ultimo paso
                {
                    angles= Angles_Calculation(vdr.TP[j-1],vdr.TP[j]); //Calculo final y anterior
                    dnxt=0;  //No habria distancia siguiente
                }
                else
                {
                    angles= Angles_Calculation(vdr.TP[j-1],vdr.TP[j],vdr.TP[j+1]);
                     dnxt=Distance(vdr.TP[j+1],vdr.TP[j]);
                }
                dnprv=Distance(vdr.TP[j],vdr.TP[j-1]);
            }
            vdr.angles[j]=angles;
            //  cout<<"Angulos: "<<angles[0]<<" "<<angles[1]<<" "<<angles[2]<<endl;
            if (j==prof_expl-1||j==0)
                dm=1.0*(dnxt+dnprv);
            else
                dm=1.0*(dnxt+dnprv)/2;

            if (dm>=0.05) dm=0.05;
            //if (dm<=0.001) dm=0.001;
            //VD.R[j][1]  Es el radio de apertura creciente
            vdr.R[j][2]=0.002;//valor de radio  z
            vdr.R[j][0]=3*dm;//dm, distancia entre puntos 

            double acDist=1.1;
            for (int k=0;k<=j;k++)
            {
                acDist += 0.8*vdr.R[k][0];
            }
            if (acDist==0) acDist = 0.01;//Quitar o revisar valor
            vdr.R[j][1] =((acDist*acDist)-1.1)/10;//+((j*j*1.0)/5000)
            if ( vdr.R[j][1] <= 0.0002) vdr.R[j][1]=0.0002;
        }
        vdr.L=prof_expl;
    return;
}
void RRT::Node_Filter()
{
    if (nodes_reordered)
    {
        OldNodes = EmptyNodes;
        
        int allowed=0;
        int cn_Old;
        std::vector<int> del_List;
        double xo ,yo,zo,rx,ry,rz,tm;
        for (int i = prof_expl; i < nodes.N; i++)
        {
            cn_Old=0;
            for (int k = 0;k < prof_expl; k++)
            {
                vector<VectorDbl > Rpitch,Rroll,Ryaw;
                Initialize_Inv_Transf_Matrices(Rpitch,Rroll,Ryaw,k);
                 xo = nodes.coord[i][0]-vdr.TP[k][0];
                 yo = nodes.coord[i][1]-vdr.TP[k][1];
                 zo = nodes.coord[i][2]-vdr.TP[k][2];
                 VectorDbl pointT{xo,yo,zo};
                VectorDbl T1 = Rotation(pointT,Rpitch,Rroll,Ryaw); 
                 rx = vdr.R[k][0];
                 ry = vdr.R[k][1];
                 rz = vdr.R[k][2];
                 tm = ((T1[0]/rx)*(T1[0]/rx))+((T1[1]/ry)*(T1[1]/ry));
                if (tm <= 1)
                {
                    allowed = 1;//Si es permitido, pasar a analizar otro punto
                    break;
                }
                else
                {                   
                    allowed=0;
                }
            }
            if (allowed == 0)  //Si es un punto rechazado
            { 
                //Save old nodes, which have passed check collision 
                
                
                del_List.push_back(i);
            }
        }
        //Print("Old Nodes Size First", OldNodes.N);
        //filtrado por trayectoria, puntos no necesarios de la antigua trayectoria
        for (int i = tr_brk-1; i < prof_expl; i++)
        {
            del_List.push_back(i);  //se elimina puntos de trayectoria y sus branches
        }
        auto temp_tic = Clock::now();
        //Ahora eliminar los puntos en la lista y sus respectivas ramas        
        for (int i = 0; i < del_List.size(); i++)
        {
            delete_branch(del_List[i]);
        }
        //Print("TIEMPO FILTER A1",toc(temp_tic).count());
    }
    return;
}

void RRT::delete_branch(int indx)
{
    std::vector<int> parents;
    parents.push_back(indx); //all the branch with this parent is eliminated
    std::vector<int> indxlist;    
    Nodes nodestemp1=nodes;
    Nodes Fin_Nodes;
     
    int ln=nodes.N;
    int k=0;bool found=false;
    std::vector<int> valid_parents;
    valid_parents.resize(0);
    while (k<ln)
    {
        found=false;        
        for (int j=0;j<parents.size();j++)
        {
            if (parents[j] == nodestemp1.parent[k]&&parents[j]>tr_brk && nodestemp1.parent[k]>tr_brk)
            {
                parents.push_back(nodestemp1.id[k]);
                //Print("old nodes");
                Push_Nodes_Elem_in_Nodes(OldNodes,nodes.id[k]);
                //Print("old nodesV");
                nodestemp1.parent[k]=-100; //borro el valor de parent para que no vuelva a caer aqui
                found=true;
                
                
            }
        }
        
        if (found==true)
          {  k=-1; } //reinicio desde cero el bucle para revisar todos los nodos otra vez4
        k++;
    }

    //Ahora quitar los nodos invalidos, y dejar los nodos permitidos unicamente.
  
    const int maxnodes=ln;
    Fin_Nodes.coord.resize(maxnodes);
    Fin_Nodes.coordT.resize(maxnodes);
    Fin_Nodes.cost.resize(maxnodes);
    Fin_Nodes.parent.resize(maxnodes);
    Fin_Nodes.id.resize(maxnodes);
    Fin_Nodes.region.resize(maxnodes);
    Fin_Nodes.N=0;
    
    int fcn=0;int badfound=0;

    for (int i=0; i<ln; i++) valid_parents.push_back(i);  
 
    //erase parents elements from valid parents vector
    for (int i=0; i<parents.size(); i++)
        valid_parents.erase(std::remove(valid_parents.begin(), valid_parents.end(), parents[i]), valid_parents.end());
   
    for(int j=0;j<valid_parents.size();j++)
    {
        int i = valid_parents[j];
       // if (i>=0 && i<ln){
            Fin_Nodes.coord[fcn]=nodes.coord[i];
            Fin_Nodes.coordT[fcn]=nodes.coordT[i];
            Fin_Nodes.cost[fcn]=nodes.cost[i];
            Fin_Nodes.parent[fcn]=nodes.parent[i]; //nodos permitidos van ordenados en Fin_Nodes
            Fin_Nodes.region[fcn]=nodes.region[i];
            Fin_Nodes.id[fcn]=fcn;
            Fin_Nodes.N=fcn+1;
            indxlist.push_back(i); //orden en la lista seria el indice nuevo y valor es el indice antiguo
            fcn++;
       // }
    }
    const int fin_sz= Fin_Nodes.N;
    Fin_Nodes.coord.resize(fin_sz);
    Fin_Nodes.coordT.resize(fin_sz);
    Fin_Nodes.cost.resize(fin_sz);
    Fin_Nodes.parent.resize(fin_sz);
    Fin_Nodes.id.resize(fin_sz);
    Fin_Nodes.region.resize(fin_sz);
//cout<<" ----DB 2---- "<<endl;
    //Ahora se corrige los parents de los nodos finales, para que apunten al nodo correcto
    //es decir, buscar si hay cambios en el indice de un nodo, y si los hay, buscar nodos hijos y corregirles el parent.
    for(int i=0;i<fcn;i++)
    {
        if(i!=indxlist[i])
        {
            for(int j=0;j<fcn;j++)
            {
                if(Fin_Nodes.parent[j]==indxlist[i])
                {
                    Fin_Nodes.parent[j]=i;
                }
            }
        }
    }
    //Print(" ----DB 4---- #nodos al entrar y salir de deletebranch: ",ln,Fin_Nodes.N);
    nodes=Fin_Nodes;//Fin_nodes seria el arreglo de nodos filtrado
}

void RRT::Nodes_Reorder()
{    
    //NUEVOS NODOS DE TRAYECTORIA
    if (nodes.coord.size()<prof_expl) nodes.coord.resize(prof_expl+1);
    for (int j=tr_brk;j < prof_expl;j++)//son los nuevos, requieren inicializar hijos, solo para los puntos de trayectoria
    {
        //Inicializacion        
        nodes.coord[j] = vdr.TP[j];

        nodes.id[j] = j;
        if(j == 0)
        {
            nodes.parent[j] = -1;
            nodes.cost[j] = 0;
        }
        else
        {
            //Print("reorder init j no 00",0);
            nodes.parent[j] = j-1;
            //Print("nodes reorder cost",double(j), nodes.cost[j-1]);
            nodes.cost[j] = Distance(vdr.TP[j-1], vdr.TP[j]) + nodes.cost[j-1]; //Aqui tiene que calcularse en funcion de la pose actual del eeff.tambien se puede calcular acumulando paso a paso
       
        }
        nodes.region[j] = j;
    }
    //ANTIGUOS NODOS DE TRAYECTORIA, REQUIEREN SWEEP Y ACTUALIZACION DE PADRES

    for (int j=0;j < tr_brk ;j++) // son los nuevos, requieren inicializar hijos, solo para los puntos de trayectoria
    {
        if (nodes_reordered==1 && tr_brk < prof_expl-1) //Revisar restriciones
        {

            nodes.coord[j]  = vdr.TP[j];
            nodes.cost[j]   = nodes.cost[j+1];
            nodes.parent[j] = nodes.parent[j+1];
            nodes.id[j]     = j;

            if (j == 0)
            {
                nodes.parent[j] = -1;
                nodes.cost[j]   =  0;
            }
            else
            {
                nodes.parent[j] = j-1;
                nodes.cost[j]   = Distance(vdr.TP[j-1],vdr.TP[j]) + nodes.cost[j-1];
            }
            nodes.region[j] = j;
        }
    }

    if (nodes.N < prof_expl-2) nodes.N = prof_expl;//ya que los nodos estan inicializados
    for (int k=prof_expl;k < nodes.N ;k++)  //  A todos lo que tengan padres entre 1 y q_tr les cambiamos el padre, a j-1
    {
        if(nodes.parent[k] >= 0 && nodes.parent[k] < tr_brk-1 )
        {

            nodes.parent[k] = nodes.parent[k]-1;
        }
    }

    nodes_reordered=1;
    return;
}


void RRT::RRT_Generation()
{
    int Num_Added_Nodes=NumNodesToAdd;
    int oldSize = nodes.N;
   //Print("********//Nodes size Start", nodes.N);
    int count=0;
    Print("********Recycled nodes", oldSize-prof_expl);     
    RRT_AddOldCoords();
    for (int j=0;j <prof_expl ;j++)//(int j=prof_expl-1;j >= 0 ;j--)
    {

    #ifdef OPENCV_DRAW
    mtxA.lock();
        //Print("AAAAAAAAAAAAAAAAAAAangles", vdr.angles[j][0],rad_to_deg(vdr.angles[j][0]));
       // Print("AAAAAAAAAAAAAAAAAAAangles", vdr.R[j][0],Img(vdr.R[j][0]), vdr.R[j][1],Img(vdr.R[j][1]));
        cv::ellipse(image_Ptraj,cv::Point(Img(vdr.TP[j][0]) ,Img(vdr.TP[j][1])),cv::Size( scale*vdr.R[j][0]/2,  scale*vdr.R[j][1]/2),rad_to_deg(vdr.angles[j][0]),0,360,Colors[j],1,8);
    mtxA.unlock();
    #endif
        if(abs(vdr.R[j][0])>=0.005)
        {
           //cout<< " radio: "<<vdr.R[j][0]<<endl;
            for (int k=0;k < Num_Added_Nodes ;k++)
            {

                Add_Node(j);//agrega 1 nodo cada vez
               // Print("Node added, radio",j);
                count++;
            }
        }
        else{}
            //Print("-----------RRT2--------j:, radio",j,vdr.R[j][0]);
       // Print("//prof  expl and count",prof_expl,count,j,nodes.N);
    }
    
    Print("NodesOld size, new size",OldNodes.N,nodes.N );

return;
}

void RRT::Add_Node(int It)
{

    double rx=vdr.R[It][0];//revisar
    double ry=vdr.R[It][1];
    double rz=vdr.R[It][2];
    VectorDbl rnTemp1(3),rnTemp_T(3); //poque despues en chequeo de colisiones se usa unicamente los 3 valores de posicion
    bool found_ik=0,allwd=0;
    VectorDbl q_rand(3);
    //====================== Creacion de puntos random centrados en cero con sus respectivos radios =========================================================
    std::random_device rdx;
    std::random_device rdy;
    std::random_device rdz;
    std::mt19937 genx(rdx());
    std::mt19937 geny(rdy());
    std::mt19937 genz(rdz());
    int try_count=0;
    double tm=100;
    bool ran_int=true;
    int prcsd=20,prcs=prcsd/2;
   
   /*
        std::uniform_int_distribution<> distxri(1,rx*prcsd);//uniform_real_distribution
        std::uniform_int_distribution<> distyri(1,ry*prcsd);
        std::uniform_int_distribution<> distzri(1,rz*prcsd);
    */
        std::normal_distribution<double> distxr(-rx*prcs,rx*prcs);//uniform_real_distribution   normal_distribution
        std::normal_distribution<double> distyr(-ry*prcs,ry*prcs);
        std::normal_distribution<double> distzr(-rz*prcs,rz*prcs);
    
    

    //Print("Radios",rx,ry,rz);
    //Print("Maximum " , xmax,ymax,zmax);
    int max_tries=5;
    int max_rnd_tries=5;
    double rnx,rny,rnz;
    while (found_ik==0)
    {
        try_count++;
        if (try_count>max_tries) {Print("fail, too much tries");break;}
        tm=100;
        bool rnd_point_found=false;
        int rnd_point_counts=0;
    while (tm>1)
    {
        //rnd_point_counts++;
        //if (rnd_point_counts > max_rnd_tries) {rnd_point_found=false; Print("fail, too much rand tries");break;}
        /*
            rnx = (distxri(genx)-rx*prcs)/prcs;
            rny = (distyri(geny)-rx*prcs)/prcs;
            rnz = (distzri(genz)-rx*prcs)/prcs;        
        */
        
            rnx = distxr(genx)/prcs;
            rny = distyr(geny)/prcs;
            rnz = distzr(genz)/prcs;
       
        
        q_rand[0]=rnx;
        q_rand[1]=rny;
        q_rand[2]=rnz;
        tm = ((rnx/rx)*(rnx/rx))+((rny/ry)*(rny/ry))+((rnz/rz)*(rnz/rz));
    }
    
   // if (tm<=1) 
     //   rnd_point_found=true; 
    //Print("RANDOM POINT",toc(temp_tic).count(),try_count,q_rand[0],tm,rnd_point_found);    
    //Print("RANDOM POINT",try_count,q_rand[0],q_rand[1],tm);    
    //======================================================================================================================================================
    //Transformaciones, rotacion y traslacion
    bool found_ik_tmp = false;
    //if (rnd_point_found){
        vector<std::vector<double> > Rpitch,Rroll,Ryaw;
        Initialize_Transf_Matrices(Rpitch,Rroll,Ryaw,It);
        rnTemp1 = Transform(q_rand,It,Rpitch,Rroll,Ryaw);  //First rotate then translate
        rnTemp_T = Translation(q_rand,It);    //Only trtaslation
        allwd = Check_Boundaries(rnTemp1);
        
        if (allwd==1)
        {
            //======Chequeo de colisiones===========================================
            std::vector<double> tempPosit(3);
            tempPosit[0] = rnTemp1[0];
            tempPosit[1] = rnTemp1[1];
            tempPosit[2] = rnTemp1[2];
            found_ik_tmp = Check_CollisionA(tempPosit,1); //modo 1 porque no estoy agregando las orientaciones en rnTemp
        }
   // }
    found_ik=found_ik_tmp;
    }

   if (try_count<=max_tries)
   {
       
        //auto temp_tic=Clock::now();
       RRT_AddValidCoord(rnTemp1,rnTemp_T,It);
       //Print("ADD NODE TIME",toc(temp_tic).count());

    }
   else
   {
       Print("-------ERROR en demasiados valores buscados en el ciclo while-------",q_rand[0],q_rand[1]);
   }
   return;
}
void RRT::RRT_AddOldCoords()
{
    double tm;
    bool allowed;
    VectorDbl ON_B(3);
    double rx,ry,rz,ON_x,ON_y,ON_z;
    int region;
    int old=0;
    for (int on=0;on<OldNodes.N;on++)
    {
        if (OldNodes.id[on]>prof_expl)
        {
            tm=100.0;
            allowed=false;
            ON_B[0] = OldNodes.coord[on][0];
            ON_B[1] = OldNodes.coord[on][1];
            ON_B[2] = OldNodes.coord[on][2];
            
            for (int It=2;It<prof_expl;It++)
            {   
                vector<VectorDbl > Rpitch,Rroll,Ryaw;
                Initialize_Inv_Transf_Matrices(Rpitch,Rroll,Ryaw,It);
                rx = vdr.R[It][0]; //revisar
                ry = vdr.R[It][1];
                rz = vdr.R[It][2];
                ON_x = ON_B[0]-vdr.TP[It][0];
                ON_y = ON_B[1]-vdr.TP[It][1];
                ON_z = ON_B[2]-vdr.TP[It][2];
                VectorDbl pointT{ON_x,ON_y,ON_z};
                VectorDbl Temp1 = Rotation(pointT,Rpitch,Rroll,Ryaw);
    
                tm = ((Temp1[0]/rx)*(Temp1[0]/rx))+((Temp1[1]/ry)*(Temp1[1]/ry))+((Temp1[2]/rz)*(Temp1[2]/rz));
            // cv::circle( image_Ptraj, cv::Point( (Temp1[0] +maxsc)*scale,(Temp1[1]+maxsc)*scale ), 1, Colors[0],CV_FILLED,  4, 8 );
                    
                //Print("OLD ALLOWED",ON_B[0],ON_B[1],tm,vdr.TP[It][0],vdr.TP[It][1],ON_x,ON_y);
            // Print("OLD ALLOWED",tm);
                if(tm<=1.0)
                {
                    //cv::circle( image_Ptraj, cv::Point( (ON_B[0] +maxsc)*scale,(ON_B[1]+maxsc)*scale ), 1, Colors[0],CV_FILLED,  4, 8 );
                    allowed = true;region=It;
                    break;
                }
            }
            if (allowed)
            {
                //Print("test2");
                RRT_AddValidCoord(OldNodes.coord[on], OldNodes.coordT[on],region);
                old++;
            }
        }        
    }
  Print("old nodes added",old);
 return;
}
void RRT::RRT_AddValidCoord(VectorDbl q_rand_TR, VectorDbl q_randA_T,int It)
{
    double r=0.01  ;   //Radio de nodos cercanos Revisar  0.009
    double EPS=0.008; //Maximo movimiento Revisar  0.005

    //AQUI EMPIEZA RRT
    double tmp_dist;
    //Hallar el minimo
    std::vector<double>  temp_coords(3);
    //Search 
    double min_ndist=1000000.0;
    int index_near=It;
    for (int k=0;k<nodes.N;k++)
    {       
        if (nodes.region[k]>=It-1&&nodes.region[k]<=It+1)
        {
        tmp_dist = Distance(q_rand_TR,nodes.coord[k]);
        if(tmp_dist<=min_ndist)
        {
            min_ndist=tmp_dist;
            index_near=k;
        }
        }
    }
    
    Node q_near,q_new;
    Extract_Node_from_Nodes( q_near,nodes,index_near); //almacenar en q_near el nodo mas cercano al punto q_new
    //Funcion steer

    q_new.coord=steer(q_rand_TR,q_near.coord,min_ndist,EPS);
    q_new.cost =Distance(q_new.coord,q_near.coord) + q_near.cost;
    //Buscar dentro del radio r los nodos mas cercanos con costo menor al punto q_new, para que se convierta en su nuevo padre
    Node q_min=q_near;
    double C_min=q_new.cost;
    int q_min_Indx=index_near;
    for (int j=0;j<nodes.N;j++)
    {
        
        if (nodes.region[j]>=It-1 && nodes.region[j]<=It+1)
        {//Print("region",nodes.region[j]);
         double Dist_node_to_qnew=Distance(nodes.coord[j],q_new.coord);
         if (Dist_node_to_qnew<=r) //Si esta dentro del circulo de radio r, entran los nearest
         {
             double Cost_q_nearest=(nodes.cost[j]+Dist_node_to_qnew);
             if ( Cost_q_nearest < C_min)
             {
                 Extract_Node_from_Nodes(q_min,nodes,j); //q_min seria el que tiene menor costo, es decir el nuevo padre de q_new
                 C_min=Cost_q_nearest;
                 q_min_Indx=j;break;
             }
         }
        }
    }
    //Print("DIST TIME",toc(ttic).count(),nodes.N,index_near,q_min_Indx);
    //Print("Point to add or not",q_rand[0],q_rand[1]);
    //Aqui q_min es el nodo mas optimo, que forma el camino mas corto
    //Update parent to least cost-from node.
    q_new.parent=q_min_Indx;
    //Ahora hacer steer otra vez para cumplir con las restricciones de dist max de movimiento
    double val=Distance(q_new.coord,q_min.coord);
    Node q_new_f;
    q_new_f.coord  = steer(q_new.coord,q_min.coord,val,EPS);
    q_new_f.coordT = q_randA_T;
    q_new_f.cost   = Distance(q_new_f.coord,q_min.coord) + q_min.cost;
    q_new_f.parent = q_min_Indx;
    q_new_f.id     = nodes.N+1;
    q_new_f.region = It;
    Insert_Node_in_Nodes(nodes,nodes.N+1,q_new_f); //Insertar nodo al final de la lista nodes, internamente se aumenta el valor de nodes.N
    //Print("Node added",q_new_f.coord[0],q_new_f.coord[1],q_new_f.coord[2]);
 #ifdef OPENCV_DRAW
    mtxA.lock();
    cv::line( image_Ptraj, cv::Point((q_new_f.coord[0]+maxsc)*scale,(q_new_f.coord[1]+maxsc)*scale ),cv::Point((q_min.coord[0]+maxsc)*scale,(q_min.coord[1]+maxsc)*scale ),  cv::Scalar( 00, 230, 50 ),  1, 8 );
    cv::circle( image_Ptraj, cv::Point( (q_new_f.coord[0] +maxsc)*scale,(q_new_f.coord[1]+maxsc)*scale ), 1, Colors[It],CV_FILLED,  1, 8 );
    cv::line( image, cv::Point((q_new_f.coord[0] -vdr.TP[It][0] +maxsc/2)*2*scale,(q_new_f.coord[1]-vdr.TP[It][1]+maxsc/2)*2*scale ),cv::Point((q_min.coord[0]-vdr.TP[It][0]+maxsc/2)*2*scale,(q_min.coord[1]-vdr.TP[It][1]+maxsc/2)*2*scale ),  cv::Scalar( 00, 230, 50 ),  1, 8 );
    
    cv::circle( image, cv::Point( (q_new_f.coord[0] -vdr.TP[It][0] +maxsc/2)*2*scale,(q_new_f.coord[1]-vdr.TP[It][1]+maxsc/2)*2*scale ), 1, Colors[It],CV_FILLED,  1, 8 );
    mtxA.unlock();
 #endif
    return;
}

void RRT::Extract_Node_from_Nodes(Node &node, Nodes &nodes, int nIndx)
{
    //Extrae un nodo del arreglo de nodos, para almacenarlo en una estructura Nodo
    node.coord.resize(3);
    node.coordT.resize(3);
    node.coord=nodes.coord[nIndx];
    node.coordT=nodes.coordT[nIndx];
    node.cost=nodes.cost[nIndx];
    node.parent=nodes.parent[nIndx];
    node.id=nodes.id[nIndx];
    node.region=nodes.region[nIndx];
    return;
}
void RRT::Insert_Node_in_Nodes(Nodes &nodes,int nIndx, Node node)
{
    //Agregar un nodo al vector de nodos en la posicion nIndx
     if (nIndx>nodes.N)
     {
         nodes.coord.resize(nIndx);
         nodes.coordT.resize(nIndx);
         nodes.cost.resize(nIndx);
         nodes.id.resize(nIndx);
         nodes.parent.resize(nIndx);
         nodes.region.resize(nIndx);
         nodes.N=nodes.N+1;
     }
    nodes.coord[nIndx-1] = node.coord;
    nodes.coordT[nIndx-1] = node.coordT;
    nodes.cost[nIndx-1]  = node.cost;
    nodes.id[nIndx-1]    = node.id;
    nodes.parent[nIndx-1]= node.parent;
    nodes.region[nIndx-1]= node.region;
    return;
}
void RRT::Push_Nodes_Elem_in_Nodes(Nodes &nodesR, int indxG)
{
    //if(indxG>=0 )      
        nodesR.coord.push_back(nodes.coord[indxG]);
        nodesR.coordT.push_back(nodes.coordT[indxG]);
        nodesR.cost.push_back(nodes.cost[indxG]);
        nodesR.parent.push_back(nodes.parent[indxG]);
        nodesR.id.push_back(nodes.id[indxG]);
        nodesR.region.push_back(nodes.region[indxG]);
        nodesR.N++;    
    return;
}


VectorDbl RRT::steer(VectorDbl qr,VectorDbl qn,double min_ndist,double EPS)
{
    VectorDbl A(3);
    if (min_ndist >= EPS||min_ndist<=EPS/5)
    {
        A[0]= qn[0] + (((qr[0]-qn[0])*EPS)/min_ndist);
        A[1]= qn[1] + (((qr[1]-qn[1])*EPS)/min_ndist);
        A[2]= qn[2] + (((qr[2]-qn[2])*EPS)/min_ndist);
    }
    else
    {
        A[0]= qr[0] ;
        A[1]= qr[1] ;
        A[2]= qr[2] ;
    }
    return A;
}


void RRT::Initialize_Transf_Matrices(vector<VectorDbl > &Rpitch,vector<VectorDbl > &Rroll,vector<VectorDbl > &Ryaw, int &It)
{
    Rpitch.resize(3); Rroll.resize(3); Ryaw.resize(3);
    for (int i=0;i<3;i++)
    {
        Rpitch[i].resize(3);Rroll[i].resize(3); Ryaw[i].resize(3);
        for (int j=0;j<3;j++)
        {
            Rpitch[i][j]=0; Rroll[i][j]=0; Ryaw[i][j]=0;
        }
    }
    //yaw,pitch,roll, ordenados
    Rpitch[0][0]=1.0; /*,  0                                ,            0   */
      /*0, */            Rpitch[1][1]=cos(vdr.angles[It][1]); Rpitch[1][2]=-sin(vdr.angles[It][1]);//X rotation
      /*0, */            Rpitch[2][1]=sin(vdr.angles[It][1]); Rpitch[2][2]=cos(vdr.angles[It][1]);

    Rroll[0][0]= cos(vdr.angles[It][2]);/*,      0            ,*/Rroll[0][2]=sin(vdr.angles[It][2]);  //Y rotation
    /*    0                            ,*/Rroll[1][1]=1.0; /* ,                0   */
    Rroll[2][0]=-sin(vdr.angles[It][2]);/*,      0            ,*/Rroll[2][2]=cos(vdr.angles[It][2]);

    Ryaw[0][0]=cos(vdr.angles[It][0]);Ryaw[0][1]=-sin(vdr.angles[It][0]); /*,       0*/   //Z rotation
    Ryaw[1][0]=sin(vdr.angles[It][0]); Ryaw[1][1]=cos(vdr.angles[It][0]); /*,       0*/
    /* ,         0                   ,                0                     ,*/Ryaw[2][2]=1.0;
    /*vector<std::vector<double> > R=Rroll;
    cout<<" Matriz "<<R[0][0]<<" "<<R[0][1]<<" "<<R[0][2]<<endl;
    cout<<" Matriz "<<R[1][0]<<" "<<R[1][1]<<" "<<R[1][2]<<endl;
    cout<<" Matriz "<<R[2][0]<<" "<<R[2][1]<<" "<<R[2][2]<<endl;*/ 
    return; 
}
void RRT::Initialize_Inv_Transf_Matrices(vector<VectorDbl > &Rpitch,vector<VectorDbl > &Rroll,vector<VectorDbl > &Ryaw, int &It)
{
    Rpitch.resize(3); Rroll.resize(3); Ryaw.resize(3);
    for (int i=0;i<3;i++)
    {
        Rpitch[i].resize(3);Rroll[i].resize(3); Ryaw[i].resize(3);
        for (int j=0;j<3;j++)
        {
            Rpitch[i][j]=0; Rroll[i][j]=0; Ryaw[i][j]=0;
        }
    }
    double InvPitch = 0.0-vdr.angles[It][1];
    double InvRoll  = 0.0-vdr.angles[It][2];
    double InvYaw   = 0.0-vdr.angles[It][0];

    //yaw,pitch,roll, ordenados
    Rpitch[0][0]=1.0; /*,  0                       ,            0                  */
      /*0, */            Rpitch[1][1]=cos(InvPitch); Rpitch[1][2]=-sin(InvPitch);//X rotation
      /*0, */            Rpitch[2][1]=sin(InvPitch); Rpitch[2][2]=cos(InvPitch);

    Rroll[0][0]= cos(InvRoll);/*,         0            ,*/Rroll[0][2]=sin(InvRoll);  //Y rotation
    /*    0                     ,*/Rroll[1][1]=1.0; /* ,                0         */
    Rroll[2][0]=-sin(InvRoll);/*,           0          ,*/Rroll[2][2]=cos(InvRoll);

    Ryaw[0][0]=cos(InvYaw);Ryaw[0][1]=-sin(InvYaw); /*,       0*/   //Z rotation
    Ryaw[1][0]=sin(InvYaw); Ryaw[1][1]=cos(InvYaw); /*,       0*/
    /* ,         0        ,           0           ,*/Ryaw[2][2]=1.0;
    return;
}


VectorDbl RRT::Transform(VectorDbl Point, int It,vector<VectorDbl > &Rpitch,vector<VectorDbl > &Rroll,vector<VectorDbl > &Ryaw)
{
    VectorDbl temp(3);
    //Inicializacion de Matrices
    temp=Matrix_Vector_MultiplyA(Rpitch,Point);
    temp=Matrix_Vector_MultiplyA(Rroll,temp);
    temp=Matrix_Vector_MultiplyA(Ryaw,temp);
    temp[0]+=vdr.TP[It][0];
    temp[1]+=vdr.TP[It][1];
    temp[2]+=vdr.TP[It][2];

    //     or eliminate the variable.
return temp;
}

VectorDbl RRT::Translation(VectorDbl Point, int It)
{
    VectorDbl temp(3);
    temp=Point;
    temp[0]+=vdr.TP[It][0];
    temp[1]+=vdr.TP[It][1];
    temp[2]+=vdr.TP[It][2];

return temp;
}
VectorDbl RRT::Rotation(VectorDbl Point,vector<VectorDbl > Rpitch,vector<VectorDbl > Rroll,vector<VectorDbl > Ryaw)
{
    VectorDbl temp(3);
    //Inicializacion de Matrices
    temp=Matrix_Vector_MultiplyA(Rpitch,Point);
    temp=Matrix_Vector_MultiplyA(Rroll,temp);
    temp=Matrix_Vector_MultiplyA(Ryaw,temp);

    //     or eliminate the variable.
return temp;
}


VectorDbl RRT::Matrix_Vector_MultiplyA(vector<VectorDbl > Matrix, VectorDbl Vector )
{
    int rowFirst=3;
    int columnFirst=3;
    VectorDbl mult(3);
    for(int i = 0; i < rowFirst; ++i)
    {   mult[i] = 0;
        for(int k=0; k<columnFirst; ++k)
           { mult[i] += (Matrix[i][k] * Vector[k]);
        }
    }
    return mult;
}

bool RRT::Check_Boundaries(VectorDbl Point)
{
    double cat1, cat2, offx,offy;
    offx = 0.0;//just in case the arm is not centered in the origin
    offy = 0.0;
    cat1 = Point[0]-offx;
    cat2 = Point[1]-offy;
    double rad = sqrt((cat1*cat1)+(cat2*cat2));

    if (rad < r_exterior && rad > r_interior)
    {//dentro del area permitida Circulo externo-circulo interno
        return true;
    }
    else
    {
        return false;
    }
}


VectorDbl  RRT::Angles_Calculation(VectorDbl P0, VectorDbl P1)
{
    VectorDbl angles;
    double dx1=P1[0]-P0[0];
    double dy1=P1[1]-P0[1];
    double dz1=P1[2]-P0[2];

    double dxy1=sqrt((dx1*dx1)+(dy1*dy1));
    double yaw1=atan2(dy1,dx1);//+M_PI_2;
    double pitch1=atan2(dz1,dxy1);
    double roll=0;
    angles.push_back(yaw1);
    angles.push_back(pitch1);
    angles.push_back(roll);
    return angles;
}
VectorDbl  RRT::Angles_Calculation( VectorDbl P0,  VectorDbl P1,  VectorDbl P2)//Overloaded
{
    std::vector<double> angles;
    double dx1=P1[0]-P0[0];
    double dy1=P1[1]-P0[1];
    double dz1=P1[2]-P0[2];
    double dxy1=sqrt((dx1*dx1)+(dy1*dy1));
    double yaw1=atan2(dy1,dx1);//+M_PI_2;
    double pitch1=atan2(dz1,dxy1);//(dxy1,dz1);
    double roll1=0;

    double dx2=P2[0]-P1[0];
    double dy2=P2[1]-P1[1];
    double dz2=P2[2]-P1[2];

    double dxy2=sqrt((dx2*dx2)+(dy2*dy2));
    double yaw2= atan2(dy2,dx2);//+M_PI_2;
    double pitch2=atan2(dz2,dxy1);
    double roll2=0;

    double yaw=(yaw1+yaw2)/2;  //mean values
    double pitch=(pitch1+pitch2)/2;
    double roll=(roll1+roll2)/2;
    angles.push_back(yaw);
    angles.push_back(pitch);
    angles.push_back(roll);
    return angles;
}
double RRT::Distance(VectorDbl P0, VectorDbl P1)
{        
    return  sqrt(((P1[0]-P0[0])*(P1[0]-P0[0]))+ ((P1[1]-P0[1])*(P1[1]-P0[1]))+((P1[2]-P0[2])*(P1[2]-P0[2])));
    //return  sqrt(((P1[0]-P0[0])*(P1[0]-P0[0]))+ ((P1[1]-P0[1])*(P1[1]-P0[1])));
}

void RRT::RRT_Sequence(geometry_msgs::Pose Marker_Abs_Pose)//extraer vecindad
{
    auto tic_time_rrt=Clock::now();
   
    tic();
    Print("-------RRt3 InitVicinity-----------");
    Initialize_VicinityRRT();
    Print("tiempo InitVicinity",toc().count());
    tic();
    Print("-------RRt4 NodelFilter------------");
    Node_Filter();
    Print("tiempo Node Filter",toc().count());
    tic();
    Print("-------RRt5 NodesReorder-----------");
    Nodes_Reorder();
    Print("tiempo Nodes Reorder",toc().count());
    tic();
    //Print("-----RRt6 RRTGEN-----------------");
    RRT_Generation();
    Print("-------RRt7 Finish-----------------");
    Print("tiempo RRT Gen",toc().count());
    Print("tiempo RRT Global",toc(tic_time_rrt).count());
    return;
}


void RRT::RRT_SequenceB()//extraer vecindad
{  //tic();
finish=false;
    Print("//-------RRt3 InitVicinity-----------");
    Initialize_VicinityRRT();
    //Print("BBtiempo InitVicinity",toc().count());
    //tic();
    Print("//-------RRt4 NodelFilter------------");
    Node_Filter();
    //Print("BBBtiempo Node Filter",toc().count());
    //tic();
Print("//-------RRt5 NodesReorder-----------");
    Nodes_Reorder();
    //Print("BBtiempo Nodes Reorder",toc().count());
    //tic();
    Print("//-----RRt6 RRTGEN-----------------");
    RRT_Generation();
    Print("//-------RRt7 Finish-----------------");
    //Print("BBtiempo RRT Gen",toc().count());
    finish=true;
    return;
}

void RRT::loop_start()
{
   // std::unique_lock<std::mutex> lck(mtx);
    sequence_loop=true;
    //cv.notify_all();
    return;
}
void RRT::loop_end()
{   //std::unique_lock<std::mutex> lck(mtx);
    sequence_loop=false;
   // cv.notify_all();
}

//================================== For SImulator =======================================================
bool RRT::Check_CollisionA(std::vector<double> posit, int i)
{

   std::this_thread::sleep_for(std::chrono::milliseconds(1));
   return true;
}
void RRT::tic()
{
    tic_clock_time = std::chrono::high_resolution_clock::now();
    return;
}
std::chrono::microseconds RRT::toc() 
{
    auto toc_clock = std::chrono::high_resolution_clock::now();        
    auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(toc_clock - tic_clock_time);
    return elapsed_time;
}

std::chrono::time_point<std::chrono::high_resolution_clock>  RRT::tic_o() 
{     
    auto tic = std::chrono::high_resolution_clock::now();
    return tic;
}

std::chrono::microseconds RRT::toc(std::chrono::time_point<std::chrono::high_resolution_clock>  tic_clock) 
{
    auto toc_clock = std::chrono::high_resolution_clock::now();        
    auto elapsed_c = std::chrono::duration_cast<std::chrono::microseconds>(toc_clock - tic_clock);
    return elapsed_c;
}
int RRT::Img(double point)
{
    point +=maxsc;
    point*=scale;
    return int(point);
}
double RRT::rad_to_deg(double rad)
{
    double deg=rad*180/PI;
    return deg;
}

#endif
