#include "uavarmfcns.h"

double maxsc=0.4;
double scale=floor(400/(2*maxsc));

struct Etraj { //Trajectory vector
     std::vector<double> xval;
     std::vector<double> yval;
     std::vector<double> zval;
     std::vector<double> w;
     std::vector<double> x;
     std::vector<double> y;
     std::vector<double> z;
};

struct Point_c { //Current point
     double xvalc;
     double yvalc;
     double zvalc;
};

struct Delta_Prev { //Current point
     double dx;
     double dy;
     double dz;
};

struct Vicinity{
   std::vector<std::vector<double> >  TP;
   std::vector<std::vector<long double> >  R;
   std::vector<std::vector<std::vector<double> > > RP;
   std::vector<std::vector<double> > angles;
   std::vector<double> N;
   int L;
};

struct PRMData{
   geometry_msgs::Pose Point;
   std::vector<double> ind_PRM_V;
   std::vector<double> ind_PRM_P;
};

void Regression(std::vector<double> x,std::vector<double> y,int ndatos,int it,int order, std::vector<double> &coeffs)
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

void CheckandFix_Boundaries(std::vector<double>  &x, std::vector<double>  &y, std::vector<double>  &z, int &prof_e, Point_c CurrentPoint)
{
    double cat1, cat2, offx,offy,xt,yt;
    offx=0.0;
    offy=0.0;
    double maxsc1=0.4;
    double scale1=floor(400/(2*maxsc1));
    double theta=0;
    double radext=0.4;
    double rad;
    float sx=0,sy=0,corg=0.0011;
    double radint=0.08,diff_prev=0;
    float xf,yf,xo,yo;
    int cndf=1,colorred,sc;
    double diffxa,diffya,sqdiffs;
    int div=1;

    oldPositions op;
    op.x.resize(6);
    op.y.resize(6);
    for (int i=0;i<6;i++)
    {
        op.x[i]=0;
        op.y[i]=0;
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
                op.x[j]=op.x[j+1];
                op.y[j]=op.y[j+1];
            }
            op.x[5]=dx;
            op.y[5]=dy;

            div=i+1;
            if (i>5) div=6;

            double dxa=(op.x[0]+op.x[1]+op.x[2]+op.x[3]+op.x[4]+op.x[5])/div;
            double dya=(op.y[0]+op.y[1]+op.y[2]+op.y[3]+op.y[4]+op.y[5])/div;
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
            if (corrx>corg) corrx=corg;
            if (corrx<-corg) corrx=-corg;
            if (corry>corg) corry=corg;
            if (corry<-corg) corry=-corg;
            x[i] += corrx ;
            y[i] += corry ;
        }

        xf=x[i], yf=y[i];
        if (i==0) {xo=x[i], yo=y[i];}
        else {xo=x[i-1], yo=y[i-1];}
        colorred=220,sc=1;
             diff_prev=sqrt((xf-xo)*(xf-xo)+(yf-yo)*(yf-yo));
             if (diff_prev>0.1)
             {
                 cndf++;
                 if (diff_prev>0.1) sc=100;

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
      cv::circle( image1, cv::Point( ( x[i]+maxsc1)*scale1,( y[i]+maxsc1)*scale1 ), 1, cv::Scalar( 0, 0, colorred ),  2, 8 );
    }
}
//     or eliminate the variable.

struct Etraj Traj_Prediction(int vcnt, int d_prm, int d_prv, int &prof_e, Delta_Prev Dv, Point_c CurrentPoint, std::vector<double> acum_x, std::vector<double> acum_y, int &flag1, int &tr_brk,Etraj tr_old ,double alturap)
{
    //1. vcnt - conteo de acumulaciones,
    //2. d_prm - profundidad de datos previos para acumulaciones,
    //3. d_prv -  profundidad de datos previos para prediccion
    //4. prof_e - profundidad de valores a predecir,
    //5. Valores de Delta de distancias previas,
    //6. Current end effector point,
    //7.8. acumulaciones de Valores X Y de la trajectoria previa
    //9. flag1 - Bandera de primera entrada, indica creacion de primeras vecinadades y almacenamiento de anterior trayectoria
    //10. tr_brk - Trajectory break, punto de quiebre de trayectorias, indica en donde termina la anterior y empieza la nueva trayectoria. q_tr en matlab
    //11. tr_old - Anterior trayectoria almacenada, para comparar con la nueva tr, y realizar composicion de trayectoria
    int n=2;
    std::vector<double> coeffs(n+1);
    Etraj traj, Tr;
    traj.xval.resize(prof_e+1);
    traj.yval.resize(prof_e+1);
    traj.zval.resize(prof_e+1);
    Tr=traj;
    double fixed_dist=0.1;


    double zvalue=alturap;
    if (vcnt<d_prm)
    {
        if (vcnt==1)
        {
            for(int i=0;i<prof_e;i++)
            {
               traj.xval[i]=CurrentPoint.xvalc ;
               traj.yval[i]=CurrentPoint.yvalc ;   //asignar un solo punto xyz a toda la trayectoria, constante
               traj.zval[i]=zvalue;
            }
        }
        else
        {
            for(int i=0;i<prof_e;i++)
            {
               traj.xval[i]=CurrentPoint.xvalc+(i*Dv.dx/2) ;
               traj.yval[i]=CurrentPoint.yvalc+(i*Dv.dy/2) ; //Prediccion totalmente lineal, no hay suficientes datos para calcular regresion
               traj.zval[i]=zvalue; //cambiar segun convenga habra que ingresar valor desde fuera de la funcion
            }
        }
        Tr=traj;
    }
    else
    {  //Cuando ya se pueda calcular regresion, es decir cuando ya se hayan acumulado muchos valores para dx dy
        double vx=Dv.dx, vy=Dv.dy;
        double vxtm=0.0002,vytm=0.0002;
        float maxdm=0.035,pnd=1.0;
        int stepc=10000;
        std::vector<double> xvala,yvala;
        xvala.resize(stepc);
        yvala.resize(stepc);

        if (vy<=-maxdm) vy=-maxdm; if (vx<=-maxdm) vx=-maxdm;
        if (vy>= maxdm) vy= maxdm; if (vy>= maxdm) vy= maxdm;

        for(int i=0;i<prof_e;i++)
        {
          //  traj.xval[i]=CurrentPoint.xvalc+(i*pnd*vx);  //creacion de estructuras basicas, para luego escoger que eje sera abcisa...
           // traj.yval[i]=CurrentPoint.yvalc+(i*pnd*vy);
            traj.zval[i]=zvalue;
        }


        if ( abs(acum_x[d_prv]-acum_x[d_prv-1]) < 0.0025 && abs(acum_y[d_prv]-acum_y[d_prv-1]) < 0.0025) fixed_dist=0.003;//antes sin restriccion era 0.1

        //=======================================================================================================================================
        if (abs(Dv.dx) > abs(Dv.dy)) //Seleccion de modo, que eje es absisa y que eje es ordenadas , se escoge el que tenga mayor informacion, pasos mas grandes
        {
            Regression(acum_x,acum_y,d_prv,1,n,coeffs);

            double signx = acum_x[d_prv]-acum_x[d_prv-1];
            if (signx>=0)
                        vxtm=vxtm;
                     else
                        vxtm=-vxtm;

            for(int i=0;i<stepc;i++)
                xvala[i]=CurrentPoint.xvalc+(i*vxtm);

            int indxj=1;
            for(int i=0;i<stepc;i++)  //Calculo de distancia junto con la regresion
            {
                yvala[i]=0.0;
                for (int j=0;j<=n;j++)
                    yvala[i] += (coeffs[j] * (pow(xvala[i],j)));

                double hipx=sqrt((pow((CurrentPoint.yvalc-yvala[i]),2)) + (pow((CurrentPoint.xvalc-xvala[i]),2)) );
                if (hipx > fixed_dist)
                {
                    indxj=i;cout<<"===============Indice x "<<i<<endl;
                    break;
                }
            }
            double stepx=(vxtm*indxj/prof_e);
            for(int i=0;i<prof_e;i++)
            {traj.xval[i]=CurrentPoint.xvalc+(i*stepx);
            cout<<"==XVAl "<<traj.xval[i]<<endl;
            }

            for(int i=0;i<prof_e;i++)  //Calculo, ahora si, de los datos de trayectoria
            {
                traj.yval[i]=0.0;
                for (int j=0;j<=n;j++)
                    traj.yval[i] += (coeffs[j] * (pow(traj.xval[i],j)));
            }


        }
        //==========================Esz==============================================================================================================
        else
        {
            Regression(acum_y,acum_x,d_prv,1,n,coeffs);

            double signy = acum_y[d_prv]-acum_y[d_prv-1];
            if (signy>=0)
                        vytm=vytm;
                     else
                        vytm=-vytm;
            for(int i=0;i<stepc;i++)
                yvala[i]=CurrentPoint.yvalc+(i*vytm);

            int indyj=1;
            for(int i=0;i<stepc;i++)  //Calculo de distancia junto con la regresion
            {
                xvala[i]=0.0;
                for (int j=0;j<=n;j++)
                    xvala[i] += (coeffs[j] * (pow(yvala[i],j)));

                double hipy=sqrt((pow((CurrentPoint.yvalc-yvala[i]),2)) + (pow((CurrentPoint.xvalc-xvala[i]),2)) );
                if (hipy > fixed_dist)
                {
                    indyj=i;cout<<"===============Indice Y "<<i<<endl;
                    break;
                }
            }
            double stepy=(vytm*indyj/prof_e);
            for(int i=0;i<prof_e;i++)
                traj.yval[i]=CurrentPoint.yvalc + (i*stepy);

            for(int i=0;i<prof_e;i++)
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
        if (flag1==1)
        {
            double dvxy=abs((Dv.dx+Dv.dy)/2);
            tr_brk=prof_e-1; //Primero suponer que toda la trayectoria debe reemplazarse
            for(int j=0;j<prof_e-1;j++)
            {
                //cv::circle( image, cv::Point( ( traj.xval[j]+maxsc1)*scale1,(traj.yval[j]+maxsc1)*scale1 ), 1, cv::Scalar( 240, 0, 0 ),  2, 8 );
                double d_tr=sqrt( (traj.xval[j]-tr_old.xval[j+1])*(traj.xval[j]-tr_old.xval[j+1]) + (traj.yval[j]-tr_old.yval[j+1])*(traj.yval[j]-tr_old.yval[j+1]) );
                if (d_tr>5*dvxy)
                {
                    tr_brk=j; break;
                }
            }
            for (int j=0;j<prof_e;j++)
            {
                if (j<tr_brk)
                {
                    Tr.xval[j]=tr_old.xval[j+1];  //Asignar trayectoria antigua antes del punto de quiebre
                    Tr.yval[j]=tr_old.yval[j+1];
                    Tr.zval[j]=zvalue;
                }
                else
                {
                    Tr.xval[j]=traj.xval[j];      //Asignar trayectoria nueva despues del punto de quiebre
                    Tr.yval[j]=traj.yval[j];
                    Tr.zval[j]=zvalue;
                }
                 //cv::circle( image1, cv::Point( ( Tr.xval[j]+maxsc1)*scale1, (Tr.yval[j]+maxsc1)*scale1 ), 1, cv::Scalar( 240, 0, 0 ),  2, 8 );
            }
            CheckandFix_Boundaries(Tr.xval, Tr.yval, Tr.zval, prof_e, CurrentPoint);
        }
    }
    fixed_dist=0.1;
return Tr;
}

//Vecindades

std::vector<double>  Angles_Calculation( std::vector<double> P0,  std::vector<double> P1)
{
    std::vector<double> angles;
    double dx1=P1[0]-P0[0];
    double dy1=P1[1]-P0[1];
    double dz1=P1[2]-P0[2];

    double dxy1=sqrt((dx1*dx1)+(dy1*dy1));
    double yaw1=atan2(dy1,dx1);//+M_PI_2;
    double pitch1=atan2(dxy1,dz1);
    double roll=0;
    angles.push_back(yaw1);
    angles.push_back(pitch1);
    angles.push_back(roll);
    return angles;
}

std::vector<double>  Angles_Calculation( std::vector<double> P0,  std::vector<double> P1,  std::vector<double> P2)//Overload
{
    std::vector<double> angles;
    double dx1=P1[0]-P0[0];
    double dy1=P1[1]-P0[1];
    double dz1=P1[2]-P0[2];

    double dxy1=sqrt((dx1*dx1)+(dy1*dy1));
    double yaw1=atan2(dy1,dx1);//+M_PI_2;
    double pitch1=atan2(dxy1,dz1);
    double roll1=0;

    double dx2=P2[0]-P1[0];
    double dy2=P2[1]-P1[1];
    double dz2=P2[2]-P1[2];

    double dxy2=sqrt((dx2*dx2)+(dy2*dy2));
    double yaw2= atan2(dy2,dx2);//+M_PI_2;
    double pitch2=atan2(dxy2,dz2);
    double roll2=0;

    double yaw=(yaw1+yaw2)/2;
    double pitch=(pitch1+pitch2)/2;
    double roll=(roll1+roll2)/2;
    angles.push_back(yaw);
    angles.push_back(pitch);
    angles.push_back(roll);
    return angles;
}

double Distance(std::vector<double> P0, std::vector<double> P1)
{
    double Dst=sqrt(((P1[0]-P0[0])*(P1[0]-P0[0]))+ ((P1[1]-P0[1])*(P1[1]-P0[1]))+((P1[2]-P0[2])*(P1[2]-P0[2])));
    return Dst;
}

std::vector<double> Matrix_Vector_MultiplyA(std::vector<std::vector<double> > Matrix, std::vector<double> Vector )
{

    int rowFirst=3;
    int columnFirst=3;
    std::vector<double> mult(3);


    for(int i = 0; i < rowFirst; ++i)
    {   mult[i] = 0;
        for(int k=0; k<columnFirst; ++k)
           { mult[i] += (Matrix[i][k] * Vector[k]);
        //cout<<"Valores de multiplicacion"<<Matrix[i][k]<<" Vector: "<< Vector[k] <<endl;
        }
    }

    return mult;
}

void Initialize_Transf_Matrices(vector<std::vector<double> > &Rpitch,vector<std::vector<double> > &Rroll,vector<std::vector<double> > &Ryaw, Vicinity &VD, int &It)
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
    Rpitch[0][0]=1;
    Rpitch[1][1]=std::cos(VD.angles[It][1]); Rpitch[1][2]=-sin(VD.angles[It][1]);
    Rpitch[2][1]=sin(VD.angles[It][1]);Rpitch[2][2]=cos(VD.angles[It][1]);

    Rroll[0][0]=cos(VD.angles[It][2]);Rroll[0][2]=sin(VD.angles[It][2]);
    Rroll[1][1]=1;
    Rroll[2][0]=-sin(VD.angles[It][2]);Rroll[2][2]=cos(VD.angles[It][2]);

    Ryaw[0][0]=cos(VD.angles[It][0]);Ryaw[0][1]=-sin(VD.angles[It][0]);
    Ryaw[1][0]=sin(VD.angles[It][0]); Ryaw[1][1]=cos(VD.angles[It][0]);
    Ryaw[2][2]=1;
    /*vector<std::vector<double> > R=Rroll;
    cout<<" Matriz "<<R[0][0]<<" "<<R[0][1]<<" "<<R[0][2]<<endl;
    cout<<" Matriz "<<R[1][0]<<" "<<R[1][1]<<" "<<R[1][2]<<endl;
    cout<<" Matriz "<<R[2][0]<<" "<<R[2][1]<<" "<<R[2][2]<<endl;*/
}

std::vector<double> Transform(std::vector<double> Point, Vicinity &VD, int &It,vector<std::vector<double> > &Rpitch,vector<std::vector<double> > &Rroll,vector<std::vector<double> > &Ryaw)
{
    std::vector<double> temp(3);
    //Inicializacion de Matrices
//cout<<VD.angles[It][0]<<" ang"<<VD.angles[It][1]<<" "<<VD.angles[It][2]<<endl;
//cout<<Point[0]<<"Puntos antes de transf "<<Point[1]<<" "<<Point[2]<<endl;

    temp=Matrix_Vector_MultiplyA(Rpitch,Point);
    temp=Matrix_Vector_MultiplyA(Rroll,temp);
    temp=Matrix_Vector_MultiplyA(Ryaw,temp);
// cout<<temp2[0]<<"pts Centrados en cero "<<temp2[1]<<" "<<temp2[2]<<endl;
    temp[0]+=VD.TP[It][0];
    temp[1]+=VD.TP[It][1];
    temp[2]+=VD.TP[It][2];
    //     or eliminate the variable.
 //cout<<temp2[0]<<"pts finales "<<temp2[1]<<" "<<temp2[2]<<endl;
return temp;
}

bool Check_Boundaries(std::vector<double> Point)
{
    bool allowed=0;
    double cat1, cat2, offx,offy;
    offx=0.0;
    offy=0.0;
    cat1= Point[0]-offx;
    cat2= Point[1]-offy;
    double rad=sqrt((cat1*cat1)+(cat2*cat2));

    double radext=0.45;double radint=0.08;
    if (rad<radext && rad>radint)
    {//Circulo externo
        allowed=1;
    }
    else
    {
        allowed=0;
    }
    return allowed;

}

double Inside_Elipse( std::vector<double> Point,  std::vector<long double> Radios, std::vector<double> offset)
{
    double tm=0.0;
    double xcomp=((Point[0]-offset[0])/Radios[0]);//Ya que el punto viene en coordenadas de marker
    double ycomp=((Point[1]-offset[1])/Radios[1]);
    double zcomp=((Point[2]-offset[2])/Radios[2]);

    tm=(xcomp*xcomp)+(ycomp*ycomp)+(zcomp*zcomp);

    return tm;

}

void Expand_Vicinity(Vicinity &VD, int Npx, int It, robot_state::RobotStatePtr &kinematic_state)
{   //VD - Vecindad
    //Npx - number of new points used to expand the ellipse
    //It - Iteracion actual de la vecindad

    vector<std::vector<double> > Rpitch,Rroll,Ryaw;
    Initialize_Transf_Matrices(Rpitch,Rroll,Ryaw,VD,It);
    int const nr=30*Npx;
    double tm;
    std::vector<double> rnx(nr),rny(nr),rnz(nr);
    std::vector<double> rnTemp(7),rnTemp1(7);

    std::vector<std::vector<double> > rf,tempP;
    int maxnm=70;
    rf.resize(maxnm);
    tempP.resize(maxnm);
    for(int i=0;i<maxnm;i++)
    {
        rf[i].resize(7);
        tempP[i].resize(7);
    }

    int cn=0;
    bool found_ik,allwd;

    const int nrolls=10;  // number of experiments
    clock_t t1;
    long double t2;

    //======================Creacion de puntos random centrados en cero con sus respectivos radios=========================================================
    std::random_device rdx;
    std::random_device rdy;
    std::random_device rdz;
    std::mt19937 genx(rdx());
    std::mt19937 geny(rdy());
    std::mt19937 genz(rdz());

    const int xmax=round(VD.R[It][0] *10000);
    const int ymax=round(VD.R[It][1] *10000);
    const int zmax=round(VD.R[It][2] *10000);

    std::uniform_int_distribution<int> distx(-xmax,xmax);
    std::uniform_int_distribution<int> disty(-ymax,ymax);
    std::uniform_int_distribution<int> distz(-zmax,zmax);

    for (int i=0; i<nr; ++i) {
        rnx[i] = distx(genx)*0.0001;
        rny[i] = disty(geny)*0.0001;
        rnz[i] = distz(genz)*0.0001;
    }
    //======================================================================================================================================================

    if (Npx!=0)
    {
        //Chequeo de colision del mismo punto de origen, (Tal vez sea redundante)
        if(VD.N[It]==0){
            found_ik = Check_Collision(VD.TP[It],2,kinematic_state);
            if (found_ik==1)
            {
                rf[cn]=VD.TP[It];
                cn++;
               // VD.N[It]=1;
            }
        }
        found_ik=0;

        //cv::circle( image, cv::Point( (VD.TP[It][0]+maxsc)*scale,(VD.TP[It][1]+maxsc-0.1)*scale ), 2, cv::Scalar( 0, 0, 230 ),  1, 8 );

        //Chequeo de cada punto random hasta completar la cuota
        for (int i=0;i<nr;i++)
        {
            double distCenter=((rnx[i]/VD.R[It][0])*(rnx[i]/VD.R[It][0]))+((rny[i]/VD.R[It][1])*(rny[i]/VD.R[It][1]))+((rnz[i]/VD.R[It][2])*(rnz[i]/VD.R[It][2]));

            if(distCenter<=1.0 && cn<Npx)
            {
                rnTemp[0]=rnx[i];rnTemp[1]=rny[i];rnTemp[2]=rnz[i]; //AGREGAR AQUI ORIENTACIONES TODO

                //==========Transformaciones===============================================

                rnTemp1=Transform(rnTemp,VD,It,Rpitch,Rroll,Ryaw);


                //==========Chequeo de restricciones geometricas===========================

                allwd=Check_Boundaries(rnTemp1);

                if (allwd==1)
                {
                    t1 = clock();
                    //======Chequeo de colisiones===========================================
                    found_ik = Check_Collision(rnTemp1,1,kinematic_state);//modo 1 porque no estoy agregando las orientaciones en rnTemp

                    t1 = clock() - t1;
                    t2=t1*1.0/CLOCKS_PER_SEC;
                  if (found_ik==1) std::cout<<"CN "<<cn<<"found? "<<found_ik<<"==== TIEMPO DE CALCULO ======"<< t2<<"  clocks  "<<t1<<"  TIEMPO  "<<CLOCKS_PER_SEC <<std::endl;

                    if (found_ik)
                    {
                        //cv::circle( image, cv::Point( (rnTemp1[0]+maxsc)*scale,(rnTemp1[1]+maxsc)*scale ), 2, cv::Scalar( 230, 120, 0 ),  1, 8 );

                        //  std::cout<<"Punto de trayectoria x: "<<VD.TP[It][0]<<" y: "<<VD.TP[It][1]<<" Z : "<<VD.TP[It][2] <<endl;
                        //  std::cout<<"Punto Random x: "<<rnTemp1[0]<<"y: "<<rnTemp1[1]<<"z: "<<rnTemp1[2]<<endl;
                        //  std::cout<<"CN "<<cn<<endl;
                        rf[cn]=rnTemp1;
                        cn++;
                    }
                }
            }
        }

        if (cn<2) std::cout<<"====CN Fail===== "<<cn<<endl;
    }

    int cn1=0;

    if (VD.N[It]>0)
    {
        for(int i=0;i<VD.N[It];i++)
        {
           tm = Inside_Elipse(VD.RP[It][i],VD.R[It],VD.TP[It]);
            //std::cout<<"============ENTRADA===== "<<VD.N[It]<<", tm: "<<tm<<endl;
            if (tm<=1.0)
            {
                tempP[cn1]=VD.RP[It][i];
                cn1++;
            }
        }
    }
    int lcp;
    if(Npx != 0)
    {
        if ( VD.N[It] > 0 && cn1>0 )
            lcp=(cn1-1);
        else
            lcp=0;

        for (int i=0;i<cn;i++)
        {
            if ((lcp+i) <= maxnm)
            {
                tempP[lcp+i] = rf[i];

            }
        }
    }

    VD.RP[It]=tempP;
    VD.N[It]=cn+cn1;
    for (int i=0;i<(VD.N[It]);i++)
    cv::circle( image, cv::Point( (VD.RP[It][i][0]+maxsc)*scale,(VD.RP[It][i][1]+maxsc)*scale ), 2, cv::Scalar( 230, 120, 0 ),  1, 8 );

    cout<<"=====Numero de Puntos en : "<<It<<" Es : "<<VD.N[It]<<", cn: "<<cn<<", cn1: "<<cn1<<endl;
    return;
}

void Initialize_Vicinity(Vicinity &VD, int &tr_brk, int prof_e, Etraj &Traj, robot_state::RobotStatePtr &kinematic_state)
{
    std::vector<double> angles(3);
    double dnprv,dnxt,dm;
    int newp=4; //puntos nuevos para cada region en inicializacion
    int iv=tr_brk,ck=0;

    for (int k=0;k<(tr_brk - 1);k++)
    {
        VD.TP[k]=VD.TP[k+1]; //Los antiguos valores se mueven hacia abajo, ya que pertenecen al anterior estado temporal
        VD.RP[k]=VD.RP[k+1];
        VD.angles[k]=VD.angles[k+1];
        VD.N[k]=VD.N[k+1];
    }

    if (iv<(prof_e-1))
    {
        for (int j=iv;j<prof_e;j++) //desde tr_brk hasta el ultmo valor de prof_e (7 que es el octavo valor), ultimo valor de trajectoria predicha
        {
            VD.TP[j][0]=Traj.xval[j];//Se carga la trayectoria predicha en esta iteracion, a los valores de trayectoria nuevos
            VD.TP[j][1]=Traj.yval[j];
            VD.TP[j][2]=Traj.zval[j];

           /* VD.R[j][2] = 0.003+((j)/500);
            if (VD.R[j][2] >= 0.09)
                VD.R[j][2] = 0.09;*/

            if (j==iv)    //Si es el primer paso
            {
                 cout<<"Checkpoint Init "<<ck++<<"iv: "<<iv<<endl;
                angles=Angles_Calculation(VD.TP[j],VD.TP[j+1]);//calcular inicial y siguiente
                dnxt=Distance(VD.TP[j],VD.TP[j+1]);
                dnprv=0; //no habria distancia anterior
            }
            else
            {
                if (j==(prof_e-1)) //Si es el ultimo paso
                {
                    angles= Angles_Calculation(VD.TP[j-1],VD.TP[j]); //Calculo final y anterior
                    dnxt=0;  //No habria distancia siguiente
                }
                else
                {
                    angles= Angles_Calculation(VD.TP[j-1],VD.TP[j],VD.TP[j+1]);
                     dnxt=Distance(VD.TP[j+1],VD.TP[j]);
                }
                dnprv=Distance(VD.TP[j],VD.TP[j-1]);
            }
            VD.angles[j]=angles;
          //  cout<<"Angulos: "<<angles[0]<<" "<<angles[1]<<" "<<angles[2]<<endl;

            if (j==prof_e||j==iv)
                dm=(dnxt+dnprv);
            else
                dm=(dnxt+dnprv)/2;

            if (dm>=0.05) dm=0.05;

          //  cout<<"DM "<<dm<<endl;
            //VD.R[j][2]=0.001;
            VD.R[j][1]=0.001;//valor de z
            VD.R[j][0]=dm;//0 es dm
            double acDist=0;

            for (int k=0;k<=j;k++)
            {
                acDist +=VD.R[k][0];
            }

            acDist = acDist + 0.0;
            VD.R[j][2] =(acDist*acDist*1.0)/10;//+((j*j*1.0)/5000)

            if (j<prof_e) //puntos de trayectoria pt(tr_brk+(prof_e-tr_brk)/2)
            {
                Expand_Vicinity(VD,newp,j,kinematic_state); //se ingresa todo por la estructura VD
            }
            else
            {
                VD.N[j]=0;//Asumo que el resto de puntos de trayectoria, despues de tr_brk, son nuevos
            }
        }
        VD.L=prof_e;
    }
    return;
}

void Reorder_Vicinity(Vicinity &VD, int prof_e, robot_state::RobotStatePtr &kinematic_state)
{
    double dnxt=0,dnprv=0,dm=0;
    std::vector<double> angles(3);
    int newp=3;
   float valbase=prof_e/5;


    for (int j=0;j<prof_e;j++) //Recorre toda la trayectoria predicha
    {
        if (j==0)    //Si es el primer paso
        {
            angles=Angles_Calculation(VD.TP[j],VD.TP[j+1]);//calcular inicial y siguiente

            dnxt=Distance(VD.TP[j],VD.TP[j+1]);
            dnprv=0; //no habria distancia anterior
        }
        else
        {
            if (j==(prof_e-1)) //Si es el ultimo paso
            {
                angles= Angles_Calculation(VD.TP[j-1],VD.TP[j]); //Calculo final y anterior
                dnxt=0;  //No habria distancia siguiente
            }
            else
            { //En el resto de pasos, es decir en los puntos de trayectoria localizados en el medio
                angles= Angles_Calculation(VD.TP[j-1],VD.TP[j],VD.TP[j+1]);
                dnxt=Distance(VD.TP[j+1],VD.TP[j]);
            }
            dnprv=Distance(VD.TP[j],VD.TP[j-1]);
        }
        VD.angles[j]=angles;
        if (j==prof_e||j==0)
            dm=(dnxt+dnprv);
        else
            dm=(dnxt+dnprv)/2;

        if (dm>=0.15) dm=0.15;

         VD.R[j][1]=0.002;
         VD.R[j][0]=dm;
         double acDist=0;
         for (int k=0;k<=j;k++)
         {
             acDist+=VD.R[k][0];
         }
         acDist= acDist + 0.7;
         VD.R[j][2]=((j*j*1.0)/5000) + (acDist*acDist*1.0)/70;
         cout<<"RADIO =========="<<VD.R[j][2]<<endl;

        if (j<=floor(valbase))
        {newp=0;}
        else
        {
            if ( j == (floor(valbase+1))) newp=6;
            if ( j == (floor(valbase+2))) newp=7;
            if ( j == (floor(valbase+3)) && j < (prof_e-1)) newp=8;
            if ( j == (prof_e-1)) newp=9;
            if ( j > (prof_e-1))  newp=8;
        }
        Expand_Vicinity(VD,newp,j,kinematic_state);
    }
return;
}


void Planif_PRM(Vicinity &VD, PRMData &PRM_Data)
{
    double d_goal,d_cpt,d_total,mindist=1000000;
    std::vector<double> PRM_pt;
    int tempIV,tempIP,ind_PRM=0;
    PRM_pt.resize(3);
    PRM_pt[0]=PRM_Data.Point.position.x;
    PRM_pt[1]=PRM_Data.Point.position.y;
    PRM_pt[2]=PRM_Data.Point.position.z;

     for(int j=0;j<VD.L;j++)
     {
         for(int k=0;k<1;k++)
         {
             for (int l=(VD.N[j+k]-1);l>=0;l--)
             {
                 d_goal  = Distance(VD.RP[j+k][l],VD.TP[VD.L-1]);//Ultimo punto de trayectoria
                 d_cpt   = Distance(VD.RP[j+k][l],PRM_pt);//punto actual de PRM, inicia con posicion del eeff
                 d_total = d_goal + d_cpt;//(1/2*VD.L)*
                 if (d_total<mindist)
                 {
                     mindist = d_total;
                     tempIV  = j+k;
                     tempIP  = l;
                 }
             }
         }
         if (mindist<0.5 && ind_PRM<(VD.L-1) )
         {
            PRM_Data.ind_PRM_V[ind_PRM]=tempIV;
            PRM_Data.ind_PRM_P[ind_PRM]=tempIP;
            PRM_pt= VD.RP[tempIV][tempIP];
            ind_PRM++;
            mindist=10000000;
             cv::circle( image1, cv::Point( (PRM_pt[0]+maxsc)*scale,(PRM_pt[1]+maxsc)*scale ), 2, cv::Scalar( 50, 50, 180), 1, 8 );
             cout <<"Punto de PRM: "<<tempIV<< "  **   "<<tempIP<< " x "<<PRM_pt[0]<<" y: "<<PRM_pt[1]<<endl;;
         }
     }
     PRM_Data.Point.position.x=VD.RP[PRM_Data.ind_PRM_V[0]][PRM_Data.ind_PRM_P[0]][0];
     PRM_Data.Point.position.y=VD.RP[PRM_Data.ind_PRM_V[0]][PRM_Data.ind_PRM_P[0]][1];
     PRM_Data.Point.position.z=VD.RP[PRM_Data.ind_PRM_V[0]][PRM_Data.ind_PRM_P[0]][2];

     return;
}















