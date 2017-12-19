#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <stdlib.h> 
#include <iostream>
#include <math.h>
#include <limits>

//variables LiDAR
double deltaangulo, angulomin;
double inf = std::numeric_limits<double>::infinity();
double poseEst[4][1], poseAct[4][1], F[4][4], H[2][4], P[4][4], Pf[4][4], s[2][2], v[2][1], z[2][1], GanK[4][2];
double aux[4][4], aux2[2][4], aux3[4][2], aux4[4][4], Ftrans[4][4], Htrans[4][2], sinv[2][2], Identity[4][4];
double posx,posy, deltaT, ayudante;
bool flag1, flag2;
std::vector<float> datos;

// callback para leer distancia del obstaculo
void poseMessageReceived (const sensor_msgs::LaserScan& msg1){
	//Angulo en radianes
	deltaangulo=msg1.angle_increment;
	angulomin=msg1.angle_min;
	datos=msg1.ranges;
	flag2=true;
}

void prediction(){
	int i,j,k,r1,c1,c2,r,c;
	//Obtener x gorro
	r1=4;
	c1=4;
	c2=1;
	for(i = 0; i < r1; ++i)
        for(j = 0; j < c2; ++j){
        	poseEst[i][j] = 0;
            for(k = 0; k < c1; ++k)
            {
                poseEst[i][j] += F[i][k] * poseAct[k][j];
            }
        }
    //Obtener P gorro
    r1=4;
	c1=4;
	c2=4;
	for(i = 0; i < r1; ++i)
        for(j = 0; j < c2; ++j){
        	aux[i][j]=0;
            for(k = 0; k < c1; ++k)
            {
                aux[i][j] += F[i][k] * Pf[k][j];
            }
        }
    r=4;
    c=4;
    for(i = 0; i < r; ++i)
        for(j = 0; j < c; ++j)
        {
            Ftrans[j][i]=F[i][j];
        }

    r1=4;
	c1=4;
	c2=4;
	for(i = 0; i < r1; ++i)
        for(j = 0; j < c2; ++j){
            P[i][j]=0;
            for(k = 0; k < c1; ++k)
            {
                P[i][j] += aux[i][k] * Ftrans[k][j];
            }
        }

}

void actualizacion(){
	int i,j,k,a, b, c, d,r1,c1,c2,r;
	//Obtener y
	r1=2;
	c1=4;
	c2=1;
	for(i = 0; i < r1; ++i)
        for(j = 0; j < c2; ++j){
        	v[i][j]=0;
            for(k = 0; k < c1; ++k)
            {
                v[i][j] += H[i][k] * poseEst[k][j];
            }
            ROS_INFO_STREAM("v="<< v[i][j]);
            v[i][j] = z[i][j]-v[i][j];
            ROS_INFO_STREAM("v="<< v[i][j]);
        }
    ROS_INFO_STREAM("vultima x="<< y[0][0] << " y=" << y[1][0]);
    //Obtener S
    r1=2;
	c1=4;
	c2=4;
	for(i = 0; i < r1; ++i)
        for(j = 0; j < c2; ++j){
            aux2[i][j]=0;
            for(k = 0; k < c1; ++k)
            {
                aux2[i][j] += H[i][k] * P[k][j];
            }
        }

    r=2;
    c=4;
    for(i = 0; i < r; ++i)
        for(j = 0; j < c; ++j)
        {
            Htrans[j][i]=H[i][j];
        }

    r1=4;
	c1=4;
	c2=4;
	for(i = 0; i < r1; ++i)
        for(j = 0; j < c2; ++j){
        	s[i][j]=0;
            for(k = 0; k < c1; ++k)
            {
                s[i][j] += aux2[i][k] * Htrans[k][j];
            }
        }

    //Obtener s inversa
    a=s[0][0];
    b=s[0][1];
    c=s[1][0];
    d=s[1][1];
    sinv[0][0]=d/(a*d-b*c);
    sinv[0][1]=-b/(a*d-b*c);
    sinv[1][0]=-c/(a*d-b*c);
    sinv[1][1]=a/(a*d-b*c);
    //Obtener K
    r1=4;
	c1=4;
	c2=2;
	for(i = 0; i < r1; ++i)
        for(j = 0; j < c2; ++j){
        	aux3[i][j]=0;
            for(k = 0; k < c1; ++k)
            {
                aux3[i][j] += P[i][k] * Htrans[k][j];
            }
        }
    r1=4;
	c1=2;
	c2=2;
	for(i = 0; i < r1; ++i)
        for(j = 0; j < c2; ++j){
        	GanK[i][j]=0;
            for(k = 0; k < c1; ++k)
            {
                GanK[i][j] += aux3[i][k] * sinv[k][j];
            }
        }
    //Obtener P gorro
    r1=4;
	c1=2;
	c2=4;
	for(i = 0; i < r1; ++i)
        for(j = 0; j < c2; ++j){
        	aux4[i][j]=0;
            for(k = 0; k < c1; ++k)
            {
                aux4[i][j] += GanK[i][k] * H[k][j];
            }
            aux4[i][j]=Identity[i][j]-aux4[i][j];
        }
    r1=4;
	c1=4;
	c2=4;
	for(i = 0; i < r1; ++i)
        for(j = 0; j < c2; ++j){
        	Pf[i][j]=0;
            for(k = 0; k < c1; ++k)
            {
                Pf[i][j] += aux4[i][k] * P[k][j];
            }
        }

    //Obtener poseAct
    r1=4;
	c1=2;
	c2=1;
	for(i = 0; i < r1; ++i)
        for(j = 0; j < c2; ++j){
        	poseAct[i][j]=0;
            for(k = 0; k < c1; ++k)
            {
                poseAct[i][j] += GanK[i][k] * v[k][j];
            }
            poseAct[i][j]=poseAct[i][j]+poseEst[i][j];
        }

}

void inicializacion(){
	F[0][0]=1;
	F[0][1]=0;
	F[0][2]=deltaT;
	F[0][3]=0;

	F[1][0]=0;
	F[1][1]=1;
	F[1][2]=0;
	F[1][3]=deltaT;

	F[2][0]=0;
	F[2][1]=0;
	F[2][2]=1;
	F[2][3]=0;

	F[3][0]=0;
	F[3][1]=0;
	F[3][2]=0;
	F[3][3]=1;

	H[0][0]=1;
	H[0][1]=0;
	H[0][2]=0;
	H[0][3]=0;

	H[1][0]=0;
	H[1][1]=1;
	H[1][2]=0;
	H[1][3]=0;

	poseAct[0][0]=0;
	poseAct[1][0]=0;
	poseAct[2][0]=0;
	poseAct[3][0]=0;

	for(int i = 0; i < 4; ++i)
        for(int j = 0; j < 4; ++j)
        	Identity[i][j]=0;
    Identity[0][0]=1;
    Identity[1][1]=1;
    Identity[2][2]=1;
    Identity[3][3]=1;

	for(int i = 0; i < 4; ++i)
        for(int j = 0; j < 4; ++j)
        	Pf[i][j]=Identity[i][j];
}

int main (int argc, char **argv){
	double  distancia, angulo;
	int i,j;
	ros::Time time1, time2;
	int cont;
	// Inicializa ROS system y crea un nodo.
	ros::init(argc, argv, "car_controller");
	ros::NodeHandle nh ;
	// Crea un objeto publicador .
	ros::Publisher pubS=nh.advertise<std_msgs::Float32>("/AutoNOMOS_mini_1/manual_control/steering",1000);
	// Crea un objeto suscriptor
	ros::Subscriber subP = nh.subscribe("/AutoNOMOS_mini_1/laser_scan", 1000, &poseMessageReceived);
	//Crea el objeto mensaje

	// Ciclo a hz Hz
	ros::Rate rate (10);

	time1  =ros::Time::now();
	time2  =ros::Time::now();

    inicializacion();
    
	//Ciclo principal
    	while (ros::ok()) {
    	//Procesar datos del LiDAR
		distancia=0;
		angulo=0;
		cont=0;
		flag1=false;

		if (flag2){
			//Obtener tiempo trascurrido entre mediciones
			time2  =ros::Time::now();
			deltaT=time2.toSec()-time1.toSec();
			time1=time2;

			//Recorrer el vector de distancias
			for(i=0; i<360; i++){
				if(datos[i]>0.2 && datos[i]<inf){
					distancia=distancia+datos[i];
					cont++;
					if(i<30){
						flag1=true;
					}
					if(i>330 && flag1){
						angulo=angulo+(i*deltaangulo-2*3.141592);
					}
					else{
						angulo=angulo+i*deltaangulo;
					}
				}
			}
			//Verificar si se detecto obstaculo
			if(cont>0){
				distancia=distancia/cont;
				angulo=angulo/cont;
			}
			ROS_INFO_STREAM("angulo: "<<angulo<<" distancia: "<<distancia << " DeltaT: " << deltaT);
			flag2=false;

			F[0][2]=deltaT;
			F[1][3]=deltaT;

			posx=distancia*cos(angulo);
			posy=distancia*sin(angulo);

			z[0][0]=posx;
			z[1][0]=posy;

			ROS_INFO_STREAM("posx=" << posx << " posy=" << posy);

			//Filtro de Kalman
			prediction();
			actualizacion();
			ROS_INFO_STREAM("Pose est x"<< poseEst[0][0] << " y " << poseEst[1][0]);
			ROS_INFO_STREAM("v x="<< v[0][0] << " y=" << v[1][0]);
			ROS_INFO_STREAM("x="<<poseAct[0][0]<<" y="<<poseAct[1][0]<<" Velx="<<poseAct[2][0]<<" Vely="<<poseAct[3][0]);
		}
		
		ros::spinOnce();
		// Espera hasta que es tiempo de la siguiente iteracion
		rate.sleep();
	}	
	
}
