#ifndef PREDICITON_CODE
#define PREDICTION_CODE

#include"prediction.hpp"

using namespace PredNs;




void Prediction::Trajectory_Prediction(geometry_msgs::Pose Marker_Abs_Pose)
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


        TP_Mtx.lock();
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
            Stop_RRT_flag=true;
            Print("fixed in 0.003");//antes era 0.1. Para cuando el UAV esta quieto
        }
        else {
            fixed_dist=f_dist; 
            Stop_RRT_flag=false;
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
        Tr=traj;
        //=========================================================================================================================================
        //=========================================COMPOSICION DE TRAYECTORIA======================================================================
       
       double maxsc1=0.4;
        double scale1=floor(400/(2*maxsc1));
        if (first_tr)
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
    TP_Mtx.unlock();
    first_tr=true;
    //Print("step-Prediction -7 ",tr_brk);
    Tr_old = Tr;
return;
}

struct rrtns::MeanValues Prediction::XYMean_Calculation(geometry_msgs::Pose Marker_Abs_Pose)
{
#ifdef OPENCV_DRAW
    White_Imag.copyTo(image_Ptraj);
#endif
    //  image_Ptraj = White_Imag.clone();//.setTo(cv::Scalar(255,255,255));
    //Print("tiempo GEN MATRIZ",toc());
    mean.vx = 0.0;
    mean.vy = 0.0;
    for(int i = 0;i < d_prv;i++)
    {
        acum_x[i] = acum_x[i+1];
        acum_y[i] = acum_y[i+1];
    }
    acum_x[d_prv] = Marker_Abs_Pose.position.x;
    acum_y[d_prv] = Marker_Abs_Pose.position.y;
    eeff_min_height = Marker_Abs_Pose.position.z; //this is the z value for the entire rrt, modify here to contact phase

#ifdef OPENCV_DRAW
    //cv::circle( image_Ptraj, cv::Point(( Marker_Abs_Pose.position.x+maxsc)*scale,( Marker_Abs_Pose.position.y+maxsc)*scale), 1, cv::Scalar( 220, 0, 0 ),  2, 8 );
#endif

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

void Prediction::Regression(std::vector<double> x,std::vector<double> y,int ndatos,int it,int order, std::vector<double> &coeffs)
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


void Prediction::CheckandFix_Boundaries(std::vector<double>  &x, std::vector<double>  &y, int &prof_e)
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
    #ifdef OPENCV_DRAW
    //mtxA.lock();
      cv::circle( image_Ptraj, cv::Point( round(( x[i]+maxsc)*scale),round(( y[i]+maxsc)*scale) ), 1, Colors[i],  2, 8 );
    //mtxA.unlock();
    #endif
     // Print("Image size and points from check function", image_Ptraj.cols, round(( x[i]+maxsc)*scale),round(( y[i]+maxsc)*scale));

    }
    return;
}

void Prediction::Selection()
{
   
    if( NodesCharged && !Stop_RRT_flag )
    { int d=0;
        NodesMtx.lock();
       // Charge_Nodes(); //the copied nodes are now in nodes
        double DFactor=0.0;
        double d_traj_adv=0.0;

        double speed = sqrt((mean.vx*mean.vx)+(mean.vy*mean.vy));
        DFactor = nodes.cost[prof_expl-1] / (1.0 + speed) ;
        VectorDbl TR{Tr.xval[0],Tr.yval[0],Tr.zval[0]};  //VS position
        double D_traj_avd = Distance(nodes.coord[0],TR);
        double Min_RRTVS_Dist=100000.0,Min_VS_Node_Dist=10000.0;
        int RRTVS_Indx, VS_Node_Indx;
        for(int i=0;i<nodes.N;i++)
        {
            double DistC = abs(nodes.cost[i] - D_traj_avd);//dist real of each node
            double NodeDist = abs(DFactor-DistC); // difference with Dfactor
            //Find node to a certain distance from VS position (indicated by DFactor)
            if(NodeDist < Min_RRTVS_Dist)   
            {
                Min_RRTVS_Dist=NodeDist;
                RRTVS_Indx=i;
            }
            //Find a the closest node to VS point
            if(DistC < Min_VS_Node_Dist)
            {
                Min_VS_Node_Dist=DistC;
                VS_Node_Indx=i;
            }
        }
        
        cv::circle( image_Ptraj, cv::Point( (nodes.coord[VS_Node_Indx][0] +maxsc)*scale,(nodes.coord[VS_Node_Indx][1]+maxsc)*scale ), 4, Colors[0],CV_FILLED,  3, 8 );
        cv::circle( image_Ptraj, cv::Point( (nodes.coord[RRTVS_Indx][0] +maxsc)*scale,(nodes.coord[RRTVS_Indx][1]+maxsc)*scale ), 4, Colors[1],CV_FILLED,  3, 8 );
    
        NodesMtx.unlock();

        NodesAvailable = false;

    }

return;
}

void Prediction::Planif_SequenceA(geometry_msgs::Pose Marker_Abs_Pose)//extraer vecindad
{
    //tic();
    //Print("-------RRt Sequence A-------------");
    XYMean_Calculation(Marker_Abs_Pose);
    //Print("tiempo RRT XY Mean Calc",toc());
    //tic();
    //Print("-------RRt2 TrajPredict------------");
    Trajectory_Prediction(Marker_Abs_Pose);
   // Print("AAA tiempo Traj Predict",toc());
   
    return;    

}
double Prediction::Distance(VectorDbl P0, VectorDbl P1)
{        
    return  sqrt(((P1[0]-P0[0])*(P1[0]-P0[0]))+ ((P1[1]-P0[1])*(P1[1]-P0[1]))+((P1[2]-P0[2])*(P1[2]-P0[2])));
}

#endif //PREDICTION
