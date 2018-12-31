#include "movfcns.h"


struct Nodes{
   std::vector<std::vector<double> >  coord;
   std::vector<double> cost;
   std::vector<int >   parent;
   std::vector<int >   id;  //No usado por ahora
   int N;                   //Numero de nodos activos
};

struct Node{
     std::vector<double> coord;
     double cost;
     int parent;
     int id;
};

void Insert_Node_in_Nodes(Nodes &nodes,int nIndx, Node node)
{
    //Agregar un nodo al vector de nodos en la posicion nIndx
     if (nIndx>nodes.N)
     {
         nodes.coord.resize(nIndx);
         nodes.cost.resize(nIndx);
         nodes.id.resize(nIndx);
         nodes.parent.resize(nIndx);
         nodes.N=nodes.N+1;
     }
    nodes.coord[nIndx-1] = node.coord;
    nodes.cost[nIndx-1]  = node.cost;
    nodes.id[nIndx-1]    = node.id;
    nodes.parent[nIndx-1]= node.parent;
}

void Extract_Node_from_Nodes(Node &node, Nodes &nodes, int nIndx)
{
    //Extrae un nodo del arreglo de nodos, para almacenarlo en una estructura Nodo
    node.coord.resize(3);
    node.coord=nodes.coord[nIndx];
    node.cost=nodes.cost[nIndx];
    node.parent=nodes.parent[nIndx];
    node.id=nodes.id[nIndx];

}

void Initialize_VicinityRRT(Vicinity &vdr, int prof_e, Etraj &Traj)
{
    std::vector<double> angles(3);
    double dnprv,dnxt,dm;
    for (int j=0;j<prof_e;j++)
    {
        vdr.TP[j][0]=Traj.xval[j];//Se carga la trayectoria predicha en esta iteracion, a los valores de trayectoria nuevos
        vdr.TP[j][1]=Traj.yval[j];
        vdr.TP[j][2]=Traj.zval[j];
    }
        for (int j=0;j<prof_e;j++) //desde tr_brk hasta el ultmo valor de prof_e (7 que es el octavo valor), ultimo valor de trajectoria predicha
        {
            if (j==0)    //Si es el primer paso
            {
                angles=Angles_Calculation(vdr.TP[j],vdr.TP[j+1]);//calcular inicial y siguiente
                dnxt=Distance(vdr.TP[j],vdr.TP[j+1]);
                dnprv=0; //no habria distancia anterior
            }
            else
            {
                if (j==(prof_e-1)) //Si es el ultimo paso
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

            if (j==prof_e||j==1)
                dm=(dnxt+dnprv);
            else
                dm=(dnxt+dnprv)/2;

            if (dm>=0.05) dm=0.05;
            //if (dm<=0.001) dm=0.001;

            //  cout<<"DM "<<dm<<endl;
            //VD.R[j][2]=0.001;
            vdr.R[j][1]=0.001;//valor de z
            vdr.R[j][0]=dm;//0 es dm
            double acDist=0;

            for (int k=0;k<=j;k++)
            {
                acDist +=vdr.R[k][0];
            }

            if (acDist==0) acDist = 0.01;//Quitar o revisar valor
            vdr.R[j][2] =(acDist*acDist*1.0)/10;//+((j*j*1.0)/5000)
            if ( vdr.R[j][2] <= 0.0002) vdr.R[j][2]=0.0002;
            cout<<"Valores de Radios Generados: 0"<<vdr.R[j][0]<<", 1: "<<vdr.R[j][1]<<", 2: "<<vdr.R[j][2]<<endl;

            //vdr.R[j][0]=0.01;
            vdr.R[j][1]=0.01;
            vdr.R[j][2]=0.1;


        }
        vdr.L=prof_e;
    return;
}

void delete_branch(Nodes &nodes, int indx, int prof_e)
{
    std::vector<int> parents;
    parents.push_back(indx);
    std::vector<int> indxlist;
    int maxnodes=prof_e;
    Nodes nodestemp1=nodes;
    Nodes Fin_Nodes;
     Fin_Nodes.coord.resize(maxnodes);
     Fin_Nodes.cost.resize(maxnodes);
     Fin_Nodes.parent.resize(maxnodes);
     Fin_Nodes.id.resize(maxnodes);
     Fin_Nodes.N=0;
    int ln=nodes.N;
    int k=0;
 //cout<<" ----DB 0---- "<<ln<<endl;
    while (k<ln)
    {

        bool found=0;
        for (int j=0;j<parents.size();j++)
        {
            if (parents[j] == nodestemp1.parent[k])
            {
                parents.push_back(k);
                nodestemp1.parent[k]=-100; //borro el valor de parent para que no vuelva a caer aqui

                found=1;
            }
        }
        k++;
        if (found==1)
          {  k=0; } //reinicio desde cero el bucle para revisar todos los nodos otra vez4
    }

    //Ahora quitar los nodos invalidos, y dejar los nodos permitidos unicamente.

    int fcn=0;
 //cout<<" ----DB ---- "<<ln<<endl;
 for(int i=0;i<ln;i++)
    {
        int badfound=0;
        for(int j=0;j<parents.size();j++)
        {
            if(i==parents[j])
            {
                badfound=1;
                break;
            }
        }
        //cout<<" ----DB1 ---- "<<endl;
        if(badfound==0)
        {
            Fin_Nodes.coord.resize(fcn+1);
            Fin_Nodes.cost.resize(fcn+1);
            Fin_Nodes.parent.resize(fcn+1);
            Fin_Nodes.id.resize(fcn+1);

            Fin_Nodes.coord[fcn]=nodes.coord[i];
            Fin_Nodes.cost[fcn]=nodes.cost[i];
            Fin_Nodes.parent[fcn]=nodes.parent[i]; //nodos permitidos van ordenados en Fin_Nodes
            Fin_Nodes.id[fcn]=fcn;
            Fin_Nodes.N=fcn+1;
            indxlist.push_back(i); //orden en la lista seria el indice nuevo y valor es el indice antiguo
            fcn++;
        }
    }
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
                    //cout<<" ----DB 3---- "<<endl;
                }
            }
        }
    }
    cout<<" ----DB 4---- #nodos al salir de deletebranch: "<<Fin_Nodes.N<<endl;
    nodes=Fin_Nodes;//Fin_nodes seria el arreglo de nodos filtrado
}

void Node_Filter(Nodes &nodes,Vicinity &vdr, int prof_e, int tr_brk)
{
    int allowed=0;
    int cndl=0;
    std::vector<int> del_List;
    for (int i=prof_e;i<nodes.N;i++)
    {
        for (int k=0;k<prof_e;k++)
        {
            double xo=nodes.coord[i][0]-vdr.TP[k][0];
            double yo=nodes.coord[i][1]-vdr.TP[k][1];
            double zo=nodes.coord[i][2]-vdr.TP[k][2];
            double rx=vdr.R[k][0];
            double ry=vdr.R[k][1];
            double rz=vdr.R[k][2];
            double tm=((xo/rx)*(xo/rx))+((yo/ry)*(yo/ry))+((zo/rz)*(zo/rz));
            if (tm<=1)
            {
                allowed=1;//Si es permitido, pasar a analizar otro punto
                break;
            }
            else
            {
                allowed=0;
            }
        }
        if (allowed==0)  //Si es un punto rechazado
        {
            cndl++;
            del_List.push_back(i);
        }
    }

    //filtrado por trayectoria, puntos no necesarios de la antigua trayectoria

    for (int i=tr_brk-1;i<prof_e;i++)
    {
        cndl++;

        del_List.push_back(i);  //se elimina puntos de trayectoria y sus branches
    }

    //Ahora eliminar los puntos en la lista y sus respectivas ramas
 cout<<" ----NF---- "<<endl;
    for (int i=0;i<del_List.size();i++)
    {
        cout<<"NF DB ----"<<i<<" cndl; "<<cndl<<endl;
        cout<<"NF DB21 ----"<<del_List[i]<<endl;
        delete_branch(nodes,del_List[i],prof_e);
        cout<<"NF DL22 ----"<<del_List[i]<<endl;
    }
}



void Nodes_Reorder(Nodes &nodes,Vicinity &vdr,int prof_e, int tr_brk, int flag1)
{
    //NUEVOS NODOS DE TRAYECTORIA
    for (int j=tr_brk;j < prof_e;j++)//son los nuevos, requieren inicializar hijos, solo para los puntos de trayectoria
    {
        //Inicializacion
        nodes.coord[j] = vdr.TP[j];
        nodes.id[j] = j;
        if (j == 0)
        {
            nodes.parent[j] = -1;
            nodes.cost[j] = 0;
        }
        else
        {
            nodes.parent[j] = j-1;
            nodes.cost[j] = Distance(vdr.TP[j-1], vdr.TP[j]) + nodes.cost[j-1]; //Aqui tiene que calcularse en funcion de la pose actual del eeff..Aqui tambien se puede calcular acumulando paso a paso
        }
    }

    //ANTIGUOS NODOS DE TRAYECTORIA, REQUIEREN SWEEP Y ACTUALIZACION DE PADRES

    for (int j=0;j < tr_brk ;j++) // son los nuevos, requieren inicializar hijos, solo para los puntos de trayectoria
    {
        if (flag1==1 && tr_brk < prof_e-1) //Revisar restriciones
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
        }
    }
    if (nodes.N < prof_e-2) nodes.N = prof_e;//ya que los nodos ya estan inicializados

    for (int k=prof_e;k < nodes.N ;k++)  //  A todos lo que tengan padres entre 1 y q_tr les cambiamos el padre, a j-1
    {
        if(nodes.parent[k] >= 0 && nodes.parent[k] < tr_brk-1 )
        {
            nodes.parent[k] = nodes.parent[k]-1;
        }
    }
}
std::vector<double> steer(std::vector<double> qr,std::vector<double> qn,double min_ndist,double EPS)
{
    std::vector<double> A(3);
    if (min_ndist >= EPS)
    {
        A[0]= qn[0] + ((qr[0]-qn[0])*EPS)/min_ndist;
        A[1]= qn[1] + ((qr[1]-qn[1])*EPS)/min_ndist;
        A[2]= qn[2] + ((qr[2]-qn[2])*EPS)/min_ndist;
    }
    else
    {
        A[0]= qr[0] ;
        A[1]= qr[1] ;
        A[2]= qr[2] ;
    }
    return A;
}

void Add_Node(Nodes &nodes, Vicinity &vdr,int It, robot_state::RobotStatePtr &kinematic_state)
{
    double EPS=0.004; //Maximo movimiento Revisar
    double r=0.007;   //Radio de nodos cercanos Revisar
    double rx=vdr.R[It][0];//revisar
    double ry=vdr.R[It][1];
    double rz=vdr.R[It][2];
    std::vector<double> rnTemp1(3); //poque despues en chequeo de colisiones se usa unicamente los 3 valores de posicion
    bool found_ik=0,allwd=0;
    std::vector<double> q_rand(3);
    //======================Creacion de puntos random centrados en cero con sus respectivos radios=========================================================
    std::random_device rdx;
    std::random_device rdy;
    std::random_device rdz;
    std::mt19937 genx(rdx());
    std::mt19937 geny(rdy());
    std::mt19937 genz(rdz());
    cout<<" ----ADDN radios---- "<<vdr.R[It][0]<<" , "<<vdr.R[It][1]<<" , "<<vdr.R[It][2]<<endl;
    const int xmax=round(vdr.R[It][0] *100000);
    const int ymax=round(vdr.R[It][1] *100000);
    const int zmax=round(vdr.R[It][2] *100000);
     cout<<" ----ADDN0---- "<<xmax<<" , "<<ymax<<" , "<<zmax<<endl;
    int try_count=0;
    double tm=100;
    std::uniform_int_distribution<int> distx(-xmax,xmax);
    std::uniform_int_distribution<int> disty(-ymax,ymax);
    std::uniform_int_distribution<int> distz(-zmax,zmax);
    int max_tries=20;
    while (found_ik==0)
    {
        try_count++;
        if (try_count>max_tries) break;
        tm=100;

    while (tm>1)
    {
        double rnx = distx(genx)*0.00001;
        double rny = disty(geny)*0.00001;
        double rnz = distz(genz)*0.00001;
        q_rand[0]=rnx;
        q_rand[1]=rny;
        q_rand[2]=rnz;
        tm = ((rnx/rx)*(rnx/rx))+((rny/ry)*(rny/ry))+((rnz/rz)*(rnz/rz));
        //cout<<" ----ADDN---- "<<tm<<" , "<<rnx<<" , "<<rny<<" , "<<rnz<<endl;
    }
    //======================================================================================================================================================
    cout<<" ----raw random found ---- "<<endl;
    //Transformaciones, rotacion y traslacion
    vector<std::vector<double> > Rpitch,Rroll,Ryaw;
    Initialize_Transf_Matrices(Rpitch,Rroll,Ryaw,vdr,It);
    cout<<" ----raw random found 11---- "<<endl;
    rnTemp1 = Transform(q_rand,vdr,It,Rpitch,Rroll,Ryaw);
    cout<<" ----raw random found 12---- "<<endl;
    allwd=Check_Boundaries(rnTemp1);
    cout<<" ----raw random found 12 allow---- "<<allwd<<" tries: "<<try_count<<endl;
    bool found_ik_tmp=0;
    if (allwd==1)
      {
          //======Chequeo de colisiones===========================================
         cout<<" entra: "<<endl;
         cout<<" x: "<<rnTemp1[0]<<" y: "<<rnTemp1[1]<<" z: "<<rnTemp1[2]<<endl;
           std::vector<double> tempPosit(7);
           tempPosit[0] = rnTemp1[0];
           tempPosit[1] = rnTemp1[1];
           tempPosit[2] = rnTemp1[2];
           found_ik_tmp = Check_Collision(tempPosit,1,kinematic_state);//modo 1 porque no estoy agregando las orientaciones en rnTemp

          //  if (found_ik==1)
              cout<<" ----Chequeo de Colisiones ---- "<<found_ik<<endl;
                    cout<<" x: "<<rnTemp1[0]<<" y: "<<rnTemp1[1]<<" z: "<<rnTemp1[2]<<endl;
      }
    found_ik=found_ik_tmp;
    }


   if (try_count<=max_tries)
   {
    cout<<" ----ADDNpa---- "<<nodes.N<<endl;

    q_rand=rnTemp1;//Valor final del numero random ya transformado y chequeado

    //AQUI EMPIEZA RRT
    std::vector<double> ndist;
    double tmp_dist;
    //Hallar el minimo
    for (int k=0;k<nodes.N;k++)
    {
        //cout<<" ----ADDNpadentro---- "<<k<<"Nodos en arreglo: "<<nodes.N<<endl;
        //cout<<" x: "<<nodes.coord[k][0]<<" y: "<<nodes.coord[k][1]<<" z: "<<nodes.coord[k][2]<<endl;
        //cout<<" q_rand x: "<<q_rand[0]<<" y: "<<q_rand[1]<<" z: "<<q_rand[2]<<"nodescoord size: "<<nodes.coord.size()<<"qrand size: "<<q_rand.size()<<endl;
        std::vector<double>  temp_coords(3);
        temp_coords[0]=nodes.coord[k][0];
        temp_coords[1]=nodes.coord[k][1];
        temp_coords[2]=nodes.coord[k][2];
        //cout<<" temp coords x: "<<temp_coords[0]<<" y: "<<temp_coords[1]<<" z: "<<temp_coords[2]<<"temp coords size: "<<temp_coords.size()<<endl;

        tmp_dist = Distance(q_rand,temp_coords);
        //cout<<" TEMP value:  "<<tmp_dist<<endl;
        ndist.push_back(tmp_dist);
    }
    cout<<" ----ADDNpa1---- "<<endl;
    double min_ndist=1000000.0;
    int index_near;
    for (int i = 0 ; i < ndist.size() ; i++ )
    {
        if(ndist[i]<=min_ndist)
        {
            min_ndist=ndist[i];
            index_near=i;
        }
    }
cout<<" ----ADDNpa2---- "<<endl;
    Node q_near,q_new;

    Extract_Node_from_Nodes( q_near,nodes,index_near); //almacenar en q_near el nodo mas cercano al punto q_new
    //Funcion steer

    q_new.coord=steer(q_rand,q_near.coord,min_ndist,EPS);
    q_new.cost =Distance(q_new.coord,q_near.coord) + q_near.cost;
cout<<" ----ADDNpa3---- "<<endl;
    //Buscar dentro del radio r los nodos mas cercanos con costo menor al punto q_new, para que se convierta en su nuevo padre
    Node q_min=q_near;
    double C_min=q_new.cost;
    int q_min_Indx=index_near;
    for (int j=0;j<nodes.N;j++)
    {
         double Dist_node_to_qnew=Distance(nodes.coord[j],q_new.coord);
         if (Dist_node_to_qnew<=r) //Si esta dentro del circulo de radio r, entran los nearest
         {
             double Cost_q_nearest=(nodes.cost[j]+Dist_node_to_qnew);
             if ( Cost_q_nearest < C_min)
             {
                 Extract_Node_from_Nodes(q_min,nodes,j); //q_min seria el que tiene menor costo, es decir el nuevo padre de q_new
                 C_min=Cost_q_nearest;
                 q_min_Indx=j;
             }
         }
    }
    //Aqui q_min es el nodo mas optimo, que forma el camino mas corto

    //Update parent to least cost-from node.
    q_new.parent=q_min_Indx;

    //Ahora hacer steer otra vez para cumplir con las restricciones de dist max de movimiento
    double val=Distance(q_new.coord,q_min.coord);
    Node q_new_f;
    q_new_f.coord  = steer(q_new.coord,q_min.coord,val,EPS);
    q_new_f.cost   = Distance(q_new_f.coord,q_min.coord) + q_min.cost;
    q_new_f.parent = q_min_Indx;
    q_new_f.id     = nodes.N+1;

    Insert_Node_in_Nodes(nodes,nodes.N+1,q_new_f); //Insertar nodo al final de la lista nodes, internamente se aumenta el valor de nodes.N
    cv::circle( image, cv::Point( (q_new_f.coord[0] +maxsc)*scale,(q_new_f.coord[1]+maxsc)*scale ), 2, cv::Scalar( 00, 230, 50 ),  1, 8 );

    //cv::line( image, cv::Point((q_new_f.coord[0]+maxsc)*scale,(q_new_f.coord[1]+maxsc)*scale ),cv::Point((q_min.coord[0]+maxsc)*scale,(q_min.coord[1]+maxsc)*scale ),  cv::Scalar( 00, 230, 50 ),  1, 8 );
    }
   else
   {
       cout<<"-------ERROR en demasiados valores buscados en el ciclo while-------"<<endl;
   }
}

void RRT(Nodes &nodes, Vicinity &vdr, int prof_e, int tr_brk,robot_state::RobotStatePtr &kinematic_state)
{
    int Num_Added_Nodes=5;//2*prof_e;

    for (int j=0;j < prof_e ;j++)
    {
        if(abs(vdr.R[j][0])>=0.003)
        {
           cout<< " radio: "<<vdr.R[j][0]<<endl;
        for (int k=0;k < Num_Added_Nodes ;k++)
        {
            Add_Node(nodes, vdr,j,kinematic_state);//agrega 1 nodo cada vez

        }
        }
        else
            cout<<"-----------RRT--------"<<j<<" radio: "<<vdr.R[j][0]<<endl;
    }



}






