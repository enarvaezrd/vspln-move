#ifndef RRT_PLANIF_CODE
#define RRT_PLANIF_CODE
#include"rrt_functionss.hpp"
using namespace rrt_planif;

void RRT::Trajectory_Prediction(geometry_msgs::Pose Marker_Abs_Pose)
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


    Position CurrentPoint;
    CurrentPoint.xval=Marker_Abs_Pose.position.x;
    CurrentPoint.yval=Marker_Abs_Pose.position.y;
    CurrentPoint.zval=Marker_Abs_Pose.position.z;
    int n=2;
    tr_brk=0;

    std::vector<double> coeffs(n+1);
    Etraj traj;
    traj.xval.resize(prof_expl+1);
    traj.yval.resize(prof_expl+1);
    traj.zval.resize(prof_expl+1);
    
    double fixed_dist=f_dist;//seria la distancia fija a la que se extiende la prediccion
    double zvalue=eeff_min_height;
    if (acum_values<d_pr_m)   //acum_values come from XYMean_Calculation
    {   if (acum_values==1)
        {
            for(int i=0;i<prof_expl;i++)
            {
               traj.xval[i]=CurrentPoint.xval ;
               traj.yval[i]=CurrentPoint.yval ;   //asignar un solo punto xyz a toda la trayectoria, constante
               traj.zval[i]=zvalue;
            }
        }

        else
        {
            for(int i=0;i<prof_expl;i++)
            {
               traj.xval[i]=CurrentPoint.xval+(i*mean.vx/2) ;
               traj.yval[i]=CurrentPoint.yval+(i*mean.vy/2) ; //Prediccion totalmente lineal, no hay suficientes datos para calcular regresion
               traj.zval[i]=zvalue; //cambiar segun convenga habra que ingresar valor desde fuera de la funcion
            }
        }
        Tr=traj;
        tr_brk=prof_expl;
    }
    else
    {  //Cuando ya se pueda calcular regresion, es decir cuando ya se hayan acumulado muchos valores para mean.vx mean.vy
        //Print("step-Prediction -3 ");
        double vx=mean.vx, vy=mean.vy;
        double vxtm=0.002,vytm=0.002;
        float maxdm=0.035,pnd=1.0;
        const int stepc=10000;
        std::vector<double> xvala(stepc),yvala(stepc);
        //xvala.resize(stepc); si compila estas lineas no son necesarias
        //yvala.resize(stepc);

        if (vy<=-maxdm) vy=-maxdm; if (vx<=-maxdm) vx=-maxdm;
        if (vy>= maxdm) vy= maxdm; if (vy>= maxdm) vy= maxdm;

        for(int i=0;i<prof_expl;i++)
        {
          //  traj.xval[i]=CurrentPoint.xvalc+(i*pnd*vx);  //creacion de estructuras basicas, para luego escoger que eje sera abcisa...
           // traj.yval[i]=CurrentPoint.yvalc+(i*pnd*vy);
            traj.zval[i]=zvalue;
        }
        double Fixf_dist = 0.002;
        if ( abs(acum_x[d_prv]-acum_x[d_prv-1]) <= Fixf_dist && abs(acum_y[d_prv]-acum_y[d_prv-1]) <= Fixf_dist) { 
            fixed_dist=0.003; 
            Print("fixed in 0.003");//antes era 0.1. Para cuando el UAV esta quieto
        }
        else {
            fixed_dist=f_dist; 
           // Print("fixed in ",fixed_dist);
            }
        //=======================================================================================================================================
        if (abs(mean.vx) >= abs(mean.vy)) //Seleccion de modo, que eje es absisa y que eje es ordenadas , se escoge el que tenga mayou informacion, pasos mas grandes
        {
            Regression(acum_x,acum_y,d_prv,1,n,coeffs);

            double signx = acum_x[d_prv]-acum_x[d_prv-1];
            if (signx>=0)
                        vxtm=vxtm;
                     else
                        vxtm=-vxtm;

            for(int i=0;i<stepc;i++)
                xvala[i]=CurrentPoint.xval+(i*vxtm);//disponer de muchos puntos

            int indxj=1;
            for(int i=0;i<stepc;i++)  //Calculo de distancia junto con la regresion
            {
                yvala[i]=0.0;
                for (int j=0;j<=n;j++)
                    yvala[i] += (coeffs[j] * (pow(xvala[i],j)));

                double hipx=sqrt((pow((CurrentPoint.yval-yvala[i]),2)) + (pow((CurrentPoint.xval-xvala[i]),2)) );
                if (hipx > fixed_dist)
                {
                    indxj=i;
                    break;
                }
            }
            double stepx=(vxtm*indxj/prof_expl);//paso resultante, para distancia fija
            for(int i=0;i<prof_expl;i++)
            {   traj.xval[i]=CurrentPoint.xval+(i*stepx);
                //cout<<"==XVAl "<<traj.xval[i]<<endl;
            }

            for(int i=0;i<prof_expl;i++)  //Calculo, ahora si, de los datos de trayectoria
            {
                traj.yval[i]=0.0;
                for (int j=0;j<=n;j++)
                    traj.yval[i] += (coeffs[j] * (pow(traj.xval[i],j)));
            }
        }
        else
        {
            Regression(acum_y,acum_x,d_prv,1,n,coeffs); //lado contrario, ejes cambiados
            double signy = acum_y[d_prv]-acum_y[d_prv-1];
            if (signy>=0)
                        vytm=vytm;
                     else
                        vytm=-vytm;
            for(int i=0;i<stepc;i++)
                yvala[i]=CurrentPoint.yval+(i*vytm);

            int indyj=1;
            for(int i=0;i<stepc;i++)  //Calculo de distancia junto con la regresion-
            {
                xvala[i]=0.0;
                for (int j=0;j<=n;j++)
                    xvala[i] += (coeffs[j] * (pow(yvala[i],j)));

                double hipy=sqrt((pow((CurrentPoint.yval-yvala[i]),2)) + (pow((CurrentPoint.xval-xvala[i]),2)) );
                if (hipy > fixed_dist)
                {
                    indyj=i;
                    break;
                }
            }
            double stepy = (vytm*indyj/prof_expl);
            for(int i=0;i<prof_expl;i++)
                traj.yval[i]=CurrentPoint.yval + (i*stepy);

            for(int i=0;i<prof_expl;i++)
            {
                traj.xval[i]=0.0;
                for (int j=0;j<=n;j++)
                    traj.xval[i] += (coeffs[j]* (pow(traj.yval[i],j)));
            }
        }
        //=========================================================================================================================================
        //=========================================COMPOSICION DE TRAYECTORIA======================================================================
       
       double maxsc1=0.4;
        double scale1=floor(400/(2*maxsc1));
        if (nodes_reordered == 1)
        {
            double dvxy=abs((mean.vx+mean.vy)/2);
            tr_brk = prof_expl-1; //Primero suponer que toda la trayectoria debe reemplazarse
            for(int j=0;j<prof_expl-1;j++)
            {
                //cv::circle( image, cv::Point( ( traj.xval[j]+maxsc1)*scale1,(traj.yval[j]+maxsc1)*scale1 ), 1, cv::Scalar( 240, 0, 0 ),  2, 8 );
                double d_tr=sqrt( (traj.xval[j]-Tr_old.xval[j+1])*(traj.xval[j]-Tr_old.xval[j+1]) + (traj.yval[j]-Tr_old.yval[j+1])*(traj.yval[j]-Tr_old.yval[j+1]) );
                if (d_tr>5*dvxy)
                {
                    tr_brk=j; break;
                }
            }
            for (int j=0;j<prof_expl;j++)
            {
                if (j<tr_brk)
                {
                    Tr.xval[j]=Tr_old.xval[j+1];  //Asignar trayectoria antigua antes del punto de quiebre
                    Tr.yval[j]=Tr_old.yval[j+1];
                    Tr.zval[j]=zvalue;
                }
                else
                {
                    Tr.xval[j]=traj.xval[j];      //Asignar trayectoria nueva despues del punto de quiebre
                    Tr.yval[j]=traj.yval[j];
                    Tr.zval[j]=zvalue;
                }
                //Print("trajvalues", Tr.xval[j],Tr.yval[j]);
                 //cv::circle( image1, cv::Point( ( Tr.xval[j]+maxsc1)*scale1, (Tr.yval[j]+maxsc1)*scale1 ), 1, cv::Scalar( 240, 0, 0 ),  2, 8 );
            }
            CheckandFix_Boundaries(Tr.xval, Tr.yval, prof_expl);
        }
        fixed_dist=f_dist;
    }
    //Print("step-Prediction -7 ",tr_brk);
    Tr_old = Tr;
return;
}

struct MeanValues RRT::XYMean_Calculation(geometry_msgs::Pose Marker_Abs_Pose)
{
    tic();
    White_Imag.copyTo(image_Ptraj);
    //  image_Ptraj = White_Imag.clone();//.setTo(cv::Scalar(255,255,255));
    Print("tiempo GEN MATRIZ",toc());
    mean.vx = 0.0;
    mean.vy = 0.0;
    for(int i = 0;i < d_prv;i++)
    {
        acum_x[i] = acum_x[i+1];
        acum_y[i] = acum_y[i+1];
    }
    acum_x[d_prv] = Marker_Abs_Pose.position.x;
    acum_y[d_prv] = Marker_Abs_Pose.position.y;

    cv::circle( image_Ptraj, cv::Point(( Marker_Abs_Pose.position.x+maxsc)*scale,( Marker_Abs_Pose.position.y+maxsc)*scale), 1, cv::Scalar( 220, 0, 0 ),  2, 8 );
  

    if (acum_values != (d_pr_m+1))
        acum_values++;
    else
        acum_values = (d_pr_m+1);

    //for(int i = (d_prv-d_pr_m+1);i<(d_prv);i++)
    for(int i = d_prv;i>(d_prv-d_pr_m);i--)
    {
        mean.vx += (acum_x[i]-acum_x[i-1]); //Acumulo todo el vector (diferencias)
        mean.vy += (acum_y[i]-acum_y[i-1]);
    }
    mean.vx /= (acum_values-1); // Acumulacion sobre numero de datos　vx　es la variacion promedio en x
    mean.vy /= (acum_values-1);
return mean;
}

void RRT::Regression(std::vector<double> x,std::vector<double> y,int ndatos,int it,int order, std::vector<double> &coeffs)
{
    //x-Valores de X
    //y-Valores de Y
    //ndatos - Cantidad de datos para realizar regresion
    //prof - Cuantos datos se van a predecir en futuro
    //it - numero de iteracion en la que inicia la regresion
    //order - Orden de regresion
    int n=order;
    const int nt=n;
    int tn=2*n+1;
    std::vector<double> SX(tn),SY(tn);
    std::vector<std::vector<double> > B(n+1, std::vector<double>(n+2,0));
    std::vector<double> a(order+1);
   // std::vector<double> coeffs(order+2);

    Etraj traj;
    traj.xval.resize(ndatos);
    traj.yval.resize(ndatos);
    traj.zval.resize(ndatos);
    int cnt=0;
    double tmp;
    for (int i=0;i<ndatos;i++)
    {
        traj.xval[cnt]=x[i];
        traj.yval[cnt]=y[i];
        cnt++;
    }

    for (int m=0;m<(tn);++m)
    {
        SX[m]=0.0;
        for (int j = 0; j < ndatos; ++j)
            SX[m] += pow(traj.xval[j], m);
    }
    for (int m=0;m<=n;++m)
    {
        SY[m]=0.0;
        for (int j = 0; j < ndatos; ++j)
            SY[m] += pow(traj.xval[j], m)* traj.yval[j];
    }
    for (int i = 0; i <= n; ++i)
        for (int j = 0; j <= n; ++j)
            B[i][j] = SX[i + j];

    for (int i = 0; i <= n; ++i)
        B[i][n+1] = SY[i];
    n += 1;
    int nm1 = n-1;

    // Pivotisation of the B matrix.
    for (int i = 0; i < n; ++i)
        for (int k = i+1; k < n; ++k)
            if (B[i][i] < B[k][i])
                for (int j = 0; j <= n; ++j) {
                    tmp = B[i][j];
                    B[i][j] = B[k][j];
                    B[k][j] = tmp;
                }
    // Performs the Gaussian elimination.
    // (1) Make all elements below the pivot equals to zero
    //     or eliminate the variable.
    for (int i=0; i<nm1; ++i)
        for (int k =i+1; k<n; ++k) {
            double t = B[k][i] / B[i][i];
            for (int j=0; j<=n; ++j)
                B[k][j] -= t*B[i][j];      // (1)
        }
    // Back substitution.
    // (1) Set the variable as the rhs of last equation
    // (2) Subtract all lhs values except the target coefficient.
    // (3) Divide rhs by coefficient of variable being calculated.
    for (int i=nm1; i >= 0; --i) {
        a[i] = B[i][n];                   // (1)
        for (int j = 0; j<n; ++j)
            if (j != i)
                a[i] -= B[i][j] * a[j];       // (2)
        a[i] /= B[i][i];                  // (3)
    }
    //coeffs.resize(a.size());
    int NN=a.size();

    for(int i=0; i<NN; ++i)
       { coeffs[i]=a[i];//cout<<"Coeficientes"<<a[i]<<endl;
    }
return;
}


void RRT::CheckandFix_Boundaries(std::vector<double>  &x, std::vector<double>  &y, int &prof_e)
{
    double cat1, cat2, offx,offy;
    offx=0.0;
    offy=0.0;
    double theta=0;
    double radext=0.41;
    double rad;
    float sx=0,sy=0,corg=0.0011;
    double radint=0.08,diff_prev=0;
    float xf,yf,xo,yo;
    int cndf=1,colorred,sc;
    double diffxa=0,diffya=0,sqdiffs;
    int div=1;

    Positions op;
    op.xval.resize(6);
    op.yval.resize(6);
    for (int i=0;i<6;i++)
    {
        op.xval[i]=0;
        op.yval[i]=0;
    }
    for (int i=0;i<prof_e;i++)
    {
        cat1= x[i]-offx;
        cat2= y[i]-offy;
        rad=sqrt((cat1*cat1)+(cat2*cat2));
        if (rad>=radext){//Circulo externo
            if(x[i]<0){
                theta=PI+atan(y[i]/x[i]);
            }
            else{
                theta=atan(y[i]/x[i]);
            }
            y[i]=radext*sin(theta);
            x[i]=radext*cos(theta);
        }
        sx=0;sy=0;
        if (rad<=radint){  //Circulo interno
            xf=x[i], yf=y[i];
            if (i==0) {xo=x[i], yo=y[i];}
            else {xo=x[i-1], yo=y[i-1];}

            float dx=1.5*(xf-xo)/1;
            float dy=1.5*(yf-yo)/1, maxC=1.0;

            if ((xf>=0) && (-dx > maxC*xf)){
                dx=-maxC*xf;}
            //     or eliminate the variable.
            else{
                if ((xf<0) && (-dx<maxC*xf))
                {dx=-maxC*xf;}
            }

            if ((yf>=0) && (-dy>maxC*yf)){
                dy=-maxC*yf;
            }
            else
            {
                if ((yf<0) && (-dy<maxC*yf))
                {dy=-maxC*yf;}
            }

            for (int j=0;j<5;j++)
            {
                op.xval[j]=op.xval[j+1];
                op.yval[j]=op.yval[j+1];
            }
            op.xval[5]=dx;
            op.yval[5]=dy;

            div=i+1;
            if (i>5) div=6;

            double dxa=(op.xval[0]+op.xval[1]+op.xval[2]+op.xval[3]+op.xval[4]+op.xval[5])/div;
            double dya=(op.yval[0]+op.yval[1]+op.yval[2]+op.yval[3]+op.yval[4]+op.yval[5])/div;
            dx=dxa;
            dy=dya;

            if ((xo>0)&&(xf<0)||(xo<0)&&(xf>0))
            {sx=xf-offx+(dx/30);}
            else
            {sx=xf-offx+dx;}
            if ((yo>0)&&(yf<0)||(yo<0)&&(yf>0))
            {sy=yf-offy+(dy/30);}
            else
            {sy=yf-offy+dy;}

            theta=std::atan2(sy,sx);

            double xf3=radint*cos(theta)+offx;
            double yf3=radint*sin(theta)+offy;

            double corrx=0.05*(xf3-xo);
            double corry=0.05*(yf3-yo);

            if (dx>0&&corrx<0) corrx=0;
            if (dx<0&&corrx>0) corrx=0;
            if (dy>0&&corry<0) corry=0;
            if (dy<0&&corry>0) corry=0;
            if (corrx> corg) corrx= corg;
            if (corrx<-corg) corrx=-corg;
            if (corry> corg) corry= corg;
            if (corry<-corg) corry=-corg;
            x[i] += corrx ;
            y[i] += corry ;
        }

        xf=x[i], yf=y[i];
        if (i==0) {xo=x[i], yo=y[i];}
        else {xo=x[i-1], yo=y[i-1];}
        colorred=220,sc=1;
             diff_prev=sqrt((xf-xo)*(xf-xo)+(yf-yo)*(yf-yo));
             //Print("Difference: ",i, diff_prev);
             if (diff_prev>0.1)
             {
                 cndf++;
                 if (diff_prev>0.15) sc=100;

                 x[i] = xo + ((diffxa) / (cndf*sc));
                 y[i] = yo + ((diffya) / (cndf*sc));
                 diffxa  = x[i] - xo;
                 diffya  = y[i] - yo;
                 sqdiffs = sqrt((diffxa*diffxa)+(diffya*diffya));
                 if (sqdiffs>0.6)
                 {
                     prof_e=i;break;
                 }
                 colorred=160;
             }
      cv::circle( image_Ptraj, cv::Point( round(( x[i]+maxsc)*scale),round(( y[i]+maxsc)*scale) ), 1, cv::Scalar( 0, 0, 150 ),  2, 8 );
     // Print("Image size and points from check function", image_Ptraj.cols, round(( x[i]+maxsc)*scale),round(( y[i]+maxsc)*scale));

    }
    return;
}

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

            if (j==prof_expl||j==1)
                dm=1.5*(dnxt+dnprv);
            else
                dm=1.5*(dnxt+dnprv)/2;

            if (dm>=0.05) dm=0.05;
            //if (dm<=0.001) dm=0.001;

            //VD.R[j][2]=0.001;
            vdr.R[j][2]=0.002;//valor de z
            vdr.R[j][0]=dm;//0 es dm
            Print("DDDD DM",dm);
            double acDist=1.0;

            for (int k=0;k<=j;k++)
            {
                acDist +=1*vdr.R[k][0];
            }

            if (acDist==0) acDist = 0.01;//Quitar o revisar valor
            vdr.R[j][1] =((acDist*acDist)-1.0)/6;//+((j*j*1.0)/5000)
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
        int cndl=0;
        std::vector<int> del_List;
        for (int i = prof_expl; i < nodes.N; i++)
        {
            for (int k = 0;k < prof_expl; k++)
            {
                double xo = nodes.coord[i][0]-vdr.TP[k][0];
                double yo = nodes.coord[i][1]-vdr.TP[k][1];
                double zo = nodes.coord[i][2]-vdr.TP[k][2];
                double rx = vdr.R[k][0];
                double ry = vdr.R[k][1];
                double rz = vdr.R[k][2];
                double tm = ((xo/rx)*(xo/rx))+((yo/ry)*(yo/ry))+((zo/rz)*(zo/rz));
                if (tm <= 1)
                {
                    allowed = 1;//Si es permitido, pasar a analizar otro punto
                    break;
                }
                else
                {
                    //Save old nodes, which have passed check collision 
                    Push_Nodes_Elem_in_Nodes(OldNodes,i);
                    allowed=0;
                }
            }
            if (allowed == 0)  //Si es un punto rechazado
            {
                cndl++;
                del_List.push_back(i);
            }
        }
        //filtrado por trayectoria, puntos no necesarios de la antigua trayectoria
        for (int i = tr_brk-1; i < prof_expl; i++)
        {
            cndl++;
            del_List.push_back(i);  //se elimina puntos de trayectoria y sus branches
        }
      
        //Ahora eliminar los puntos en la lista y sus respectivas ramas        
        for (int i = 0; i < del_List.size(); i++)
        {            
            delete_branch(del_List[i]);            
        }
    }
    return;
}

void RRT::delete_branch(int indx)
{
    std::vector<int> parents;
    parents.push_back(indx); //all the branch with this parent is eliminated
    std::vector<int> indxlist;
    int maxnodes=prof_expl;
    Nodes nodestemp1=nodes;
    Nodes Fin_Nodes;
     Fin_Nodes.coord.resize(maxnodes);
     Fin_Nodes.cost.resize(maxnodes);
     Fin_Nodes.parent.resize(maxnodes);
     Fin_Nodes.id.resize(maxnodes);
     Fin_Nodes.N=0;
    int ln=nodes.N;
    int k=0;
    while (k<ln)
    {

        bool found=0;
        for (int j=0;j<parents.size();j++)
        {
            if (parents[j] == nodestemp1.parent[k]&&parents[j]>tr_brk && nodestemp1.parent[k]>tr_brk)
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
        //Print("rearrange",ln,i);
        int badfound=0;
        for(int j=0;j<parents.size();j++)
        {
            if(i==parents[j])  //lista de parents a borrar
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
Print("nodes size",nodes.coord.size());
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
    Print("Nodes size",nodes.N);
    RRT_AddOldCoords();
    Print("NodesOld size, new size",OldNodes.N,nodes.N );
    int count=0;
    for (int j=prof_expl-1;j >= 0 ;j--)
    {
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
    Print("********//Nodes size", nodes.N,count);

return;
}

void RRT::Add_Node(int It)
{

    double rx=vdr.R[It][0];//revisar
    double ry=vdr.R[It][1];
    double rz=vdr.R[It][2];
    VectorDbl rnTemp1(3); //poque despues en chequeo de colisiones se usa unicamente los 3 valores de posicion
    bool found_ik=0,allwd=0;
    VectorDbl q_rand(3);
    //====================== Creacion de puntos random centrados en cero con sus respectivos radios =========================================================
    std::random_device rdx;
    std::random_device rdy;
    std::random_device rdz;
    std::mt19937 genx(rdx());
    std::mt19937 geny(rdy());
    std::mt19937 genz(rdz());
    const int xmax=round(vdr.R[It][0] *100000);
    const int ymax=round(vdr.R[It][1] *100000);
    const int zmax=round(vdr.R[It][2] *100000);
    int try_count=0;
    double tm=100;
    std::uniform_int_distribution<int> distx(-xmax,xmax);
    std::uniform_int_distribution<int> disty(-ymax,ymax);
    std::uniform_int_distribution<int> distz(-zmax,zmax);
    //Print("Radios",rx,ry,rz);
    //Print("Maximum " , xmax,ymax,zmax);
    int max_tries=20;
    int max_rnd_tries=50;
    while (found_ik==0)
    {
        try_count++;
        if (try_count>max_tries) {Print("fail, too much tries");break;}
        tm=100;
        bool rnd_point_found=false;
        int rnd_point_counts=0;
    while (tm>1)
    {
        rnd_point_counts++;
        if (rnd_point_counts > max_rnd_tries) {rnd_point_found=false; Print("fail, too much rand tries");break;}
        double rnx = distx(genx)*0.00001;
        double rny = disty(geny)*0.00001;
        double rnz = distz(genz)*0.00001;

        q_rand[0]=rnx;
        q_rand[1]=rny;
        q_rand[2]=rnz;
        tm = ((rnx/rx)*(rnx/rx))+((rny/ry)*(rny/ry))+((rnz/rz)*(rnz/rz));
    }
    if (tm<=1) 
        rnd_point_found=true; 
        
    //Print("Distribution",q_rand[0],q_rand[1],q_rand[2]);
    //======================================================================================================================================================
    //Transformaciones, rotacion y traslacion
    bool found_ik_tmp = false;
    if (rnd_point_found){
        vector<std::vector<double> > Rpitch,Rroll,Ryaw;
        Initialize_Transf_Matrices(Rpitch,Rroll,Ryaw,It);
        rnTemp1 = Transform(q_rand,It,Rpitch,Rroll,Ryaw);
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
    }
    found_ik=found_ik_tmp;
    }

   if (try_count<=max_tries)
   {
       RRT_AddValidCoord(rnTemp1);
    }
   else
   {
       Print("-------ERROR en demasiados valores buscados en el ciclo while-------",xmax,ymax,q_rand[0],q_rand[1]);
   }
   return;
}
void RRT::RRT_AddOldCoords()
{
    if(OldNodes.N>0)
    {           
        double tm;
        bool allowed;
        for (int on=0;on<OldNodes.N;on++)
        { 
            tm=100;allowed=false;
            VectorDbl ON(3);
            ON[0] = OldNodes.coord[on][0];
            ON[1] = OldNodes.coord[on][1];
            ON[2] = OldNodes.coord[on][2];

            for (int It=0;It<prof_expl;It++)
            {                 
                double rx=vdr.R[It][0];//revisar
                double ry=vdr.R[It][1];
                double rz=vdr.R[It][2];

                tm = ((ON[0]/rx)*(ON[0]/rx))+((ON[1]/ry)*(ON[1]/ry))+((ON[2]/rz)*(ON[2]/rz));
                if(tm<=1)
                {
                    allowed = true;
                    Print("ALLOWED");
                    break;
                }
            }
            if (allowed)
            {
                RRT_AddValidCoord(ON);
            }
        }
    }
 return;
}
void RRT::RRT_AddValidCoord(VectorDbl q_randA)
{
    double r=0.009;   //Radio de nodos cercanos Revisar
    double EPS=0.005; //Maximo movimiento Revisar//Valor final del numero random ya transformado y chequeado

    //AQUI EMPIEZA RRT
    std::vector<double> ndist;
    double tmp_dist;
    //Hallar el minimo
    //Print("add nodes 1");
    for (int k=0;k<nodes.N;k++)
    {
       std::vector<double>  temp_coords(3);
        temp_coords[0]=nodes.coord[k][0];
        temp_coords[1]=nodes.coord[k][1];
        temp_coords[2]=nodes.coord[k][2];
        tmp_dist = Distance(q_randA,temp_coords);
        ndist.push_back(tmp_dist);
    }
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
    Node q_near,q_new;
    Extract_Node_from_Nodes( q_near,nodes,index_near); //almacenar en q_near el nodo mas cercano al punto q_new
    //Funcion steer

    q_new.coord=steer(q_randA,q_near.coord,min_ndist,EPS);
    q_new.cost =Distance(q_new.coord,q_near.coord) + q_near.cost;
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
    //Print("Point to add or not",q_rand[0],q_rand[1]);
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
    //Print("Node added",q_new_f.coord[0],q_new_f.coord[1], rx, ry);
    mtxA.lock();
   

   // cv::line( image_Ptraj, cv::Point((q_new_f.coord[0]+maxsc)*scale,(q_new_f.coord[1]+maxsc)*scale ),cv::Point((q_min.coord[0]+maxsc)*scale,(q_min.coord[1]+maxsc)*scale ),  cv::Scalar( 00, 230, 50 ),  1, 8 );
     cv::circle( image_Ptraj, cv::Point( (q_new_f.coord[0] +maxsc)*scale,(q_new_f.coord[1]+maxsc)*scale ), 1, cv::Scalar( 00, 20, 10 ),CV_FILLED,  1, 8 );
    mtxA.unlock();
    return;
}


void RRT::Extract_Node_from_Nodes(Node &node, Nodes &nodes, int nIndx)
{
    //Extrae un nodo del arreglo de nodos, para almacenarlo en una estructura Nodo
    node.coord.resize(3);
    node.coord=nodes.coord[nIndx];
    node.cost=nodes.cost[nIndx];
    node.parent=nodes.parent[nIndx];
    node.id=nodes.id[nIndx];
    return;
}
void RRT::Insert_Node_in_Nodes(Nodes &nodes,int nIndx, Node node)
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
    return;
}
void RRT::Push_Nodes_Elem_in_Nodes(Nodes &nodesR, int indxG)
{
    if(indxG>=0)
    {  
       // const int szcrd = nodesR.coord.size();
        //nodesR.coord.resize(szcrd+1); 
       // nodesR.coord[szcrd].resize(3);
       /* nodesR.coord[szcrd][0]=nodes.coord[indxG][0];
        nodesR.coord[szcrd][1]=nodes.coord[indxG][1]; 
        nodesR.coord[szcrd][2]=nodes.coord[indxG][2];*/
        nodesR.coord.push_back(nodes.coord[indxG]);
        nodesR.cost.push_back(nodes.cost[indxG]);
        nodesR.parent.push_back(nodes.parent[indxG]);
        nodesR.id.push_back(nodes.id[indxG]);
        nodesR.N++;
    }
    return;
}


VectorDbl RRT::steer(VectorDbl qr,VectorDbl qn,double min_ndist,double EPS)
{
    VectorDbl A(3);
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
    double Dst=sqrt(((P1[0]-P0[0])*(P1[0]-P0[0]))+ ((P1[1]-P0[1])*(P1[1]-P0[1]))+((P1[2]-P0[2])*(P1[2]-P0[2])));
    return Dst;
}

void RRT::RRT_Sequence(geometry_msgs::Pose Marker_Abs_Pose)//extraer vecindad
{
    double tic_time_rrt=clock();
    tic();
    Print("-------RRt1 XYMeanCalc-------------");
    XYMean_Calculation(Marker_Abs_Pose);
    Print("tiempo RRT XY Mean Calc",toc());
    tic();
    Print("-------RRt2 TrajPredict------------");
    Trajectory_Prediction(Marker_Abs_Pose);
    Print("tiempo Traj Predict",toc());
    tic();
    Print("-------RRt3 InitVicinity-----------");
    Initialize_VicinityRRT();
    Print("tiempo InitVicinity",toc());
    tic();
    Print("-------RRt4 NodelFilter------------");
    Node_Filter();
    Print("tiempo Node Filter",toc());
    tic();
    Print("-------RRt5 NodesReorder-----------");
    Nodes_Reorder();
    Print("tiempo Nodes Reorder",toc());
    tic();
    //Print("-----RRt6 RRTGEN-----------------");
    RRT_Generation();
    Print("-------RRt7 Finish-----------------");
    Print("tiempo RRT Gen",toc());
    Print("tiempo RRT Global",toc(tic_time_rrt));
    return;
}
void RRT::RRT_SequenceA(geometry_msgs::Pose Marker_Abs_Pose)//extraer vecindad
{
    finish=false;
    tic();
    Print("-------RRt Sequence A-------------");
    XYMean_Calculation(Marker_Abs_Pose);
    //Print("tiempo RRT XY Mean Calc",toc());
    //tic();
    //Print("-------RRt2 TrajPredict------------");
    Trajectory_Prediction(Marker_Abs_Pose);
    Print("tiempo Traj Predict",toc());
   
    return;    
}
void RRT::RRT_SequenceB()//extraer vecindad
{  tic();
finish=false;
   // Print("//-------RRt3 InitVicinity-----------");
    Initialize_VicinityRRT();
    //Print("tiempo InitVicinity",toc());
    tic();
   // Print("//-------RRt4 NodelFilter------------");
    Node_Filter();
   // Print("tiempo Node Filter",toc());
    tic();
    //Print("//-------RRt5 NodesReorder-----------");
    Nodes_Reorder();
   // Print("tiempo Nodes Reorder",toc());
    tic();
    Print("//-----RRt6 RRTGEN-----------------");
    RRT_Generation();
    Print("//-------RRt7 Finish-----------------");
   // Print("tiempo RRT Gen",toc());
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

   std::this_thread::sleep_for(std::chrono::milliseconds(5));
   return true;
}
void RRT::tic()
{
    tic_clock_time=0;
    tic_clock_time = clock();
}
long double RRT::toc() 
{
     double elapsed_time_clocks = clock() - tic_clock_time;
    long double elapsed_time = elapsed_time_clocks*1.0/CLOCKS_PER_SEC;//tiempo transcurrido en el codigo
    return elapsed_time;
}
long double RRT::toc(double tic_clock) 
{
     double elapsed_time_clocks;
        elapsed_time_clocks= clock() - tic_clock;
    long double elapsed_time = elapsed_time_clocks*1.0/CLOCKS_PER_SEC;//tiempo transcurrido en el codigo
    return elapsed_time;
}

#endif
