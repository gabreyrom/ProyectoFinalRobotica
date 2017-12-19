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
	for(i = 0; i < r1; ++i){
        for(j = 0; j < c2; ++j){
        	poseEst[i][j] = 0;
            for(k = 0; k < c1; ++k)
            {
                poseEst[i][j] += F[i][k] * poseAct[k][j];
            }
        }
    }
    ROS_INFO_STREAM("F");
    ROS_INFO_STREAM(""<< F[0][0] << " " << F[0][1] << " "<< F[0][2] << " " << F[0][3]);
	ROS_INFO_STREAM(""<< F[1][0] << " " << F[1][1] << " "<< F[1][2] << " " << F[1][3]);
    ROS_INFO_STREAM(""<< F[2][0] << " " << F[2][1] << " "<< F[2][2] << " " << F[2][3]);
    ROS_INFO_STREAM(""<< F[3][0] << " " << F[3][1] << " "<< F[3][2] << " " << F[3][3]);
    //Obtener P gorro
    r1=4;
	c1=4;
	c2=4;
	for(i = 0; i < r1; ++i){
        for(j = 0; j < c2; ++j){
        	aux[i][j]=0;
            for(k = 0; k < c1; ++k)
            {
                aux[i][j] += F[i][k] * Pf[k][j];
            }
        }
    }
    ROS_INFO_STREAM("AUX");
    ROS_INFO_STREAM(""<< aux[0][0] << " " << aux[0][1] << " "<< aux[0][2] << " " << aux[0][3]);
	ROS_INFO_STREAM(""<< aux[1][0] << " " << aux[1][1] << " "<< aux[1][2] << " " << aux[1][3]);
    ROS_INFO_STREAM(""<< aux[2][0] << " " << aux[2][1] << " "<< aux[2][2] << " " << aux[2][3]);
    ROS_INFO_STREAM(""<< aux[3][0] << " " << aux[3][1] << " "<< aux[3][2] << " " << aux[3][3]);
    r=4;
    c=4;
    for(i = 0; i < r; ++i){
        for(j = 0; j < c; ++j)
        {
            Ftrans[j][i]=F[i][j];
        }
	}
	ROS_INFO_STREAM("FTRANS");
    ROS_INFO_STREAM(""<< Ftrans[0][0] << " " << Ftrans[0][1] << " "<< Ftrans[0][2] << " " << Ftrans[0][3]);
	ROS_INFO_STREAM(""<< Ftrans[1][0] << " " << Ftrans[1][1] << " "<< Ftrans[1][2] << " " << Ftrans[1][3]);
    ROS_INFO_STREAM(""<< Ftrans[2][0] << " " << Ftrans[2][1] << " "<< Ftrans[2][2] << " " << Ftrans[2][3]);
    ROS_INFO_STREAM(""<< Ftrans[3][0] << " " << Ftrans[3][1] << " "<< Ftrans[3][2] << " " << Ftrans[3][3]);
    r1=4;
	c1=4;
	c2=4;
	for(i = 0; i < r1; ++i){
        for(j = 0; j < c2; ++j){
            P[i][j]=0;
            for(k = 0; k < c1; ++k)
            {
                P[i][j] += aux[i][k] * Ftrans[k][j];
            }
        }
    }
    ROS_INFO_STREAM("P");
    ROS_INFO_STREAM(""<< P[0][0] << " " << P[0][1] << " "<< P[0][2] << " " << P[0][3]);
	ROS_INFO_STREAM(""<< P[1][0] << " " << P[1][1] << " "<< P[1][2] << " " << P[1][3]);
    ROS_INFO_STREAM(""<< P[2][0] << " " << P[2][1] << " "<< P[2][2] << " " << P[2][3]);
    ROS_INFO_STREAM(""<< P[3][0] << " " << P[3][1] << " "<< P[3][2] << " " << P[3][3]);
}

void actualizacion(){
	int i,j,k,r1,c1,c2,r;
	double a, b, c, d;
	ROS_INFO_STREAM("HTRANS");
	ROS_INFO_STREAM(""<< Htrans[0][0] << " " << Htrans[0][1]);
	ROS_INFO_STREAM(""<< Htrans[1][0] << " " << Htrans[1][1]);
    ROS_INFO_STREAM(""<< Htrans[2][0] << " " << Htrans[2][1]);
    ROS_INFO_STREAM(""<< Htrans[3][0] << " " << Htrans[3][1]);

	//Obtener y
	r1=2;
	c1=4;
	c2=1;
	for(i = 0; i < r1; ++i){
        for(j = 0; j < c2; ++j){
        	v[i][j]=0;
            for(k = 0; k < c1; ++k)
            {
                v[i][j] += H[i][k] * poseEst[k][j];
            }
            v[i][j] = z[i][j]-v[i][j];
        }
    }
    ROS_INFO_STREAM("vActualiza x="<< v[0][0] << " y=" << v[1][0]);
    //Obtener S
    r1=2;
	c1=4;
	c2=4;
	for(i = 0; i < r1; ++i){
        for(j = 0; j < c2; ++j){
            aux2[i][j]=0;
            for(k = 0; k < c1; ++k)
            {
                aux2[i][j] += H[i][k] * P[k][j];
            }
        }
    }

    r1=2;
	c1=4;
	c2=2;
	for(i = 0; i < r1; ++i){
        for(j = 0; j < c2; ++j){
        	s[i][j]=0;
            for(k = 0; k < c1; ++k)
            {
                s[i][j] += aux2[i][k] * Htrans[k][j];
            }
        }
    }
    ROS_INFO_STREAM("S");
    ROS_INFO_STREAM(""<< s[0][0] << " " << s[0][1]);
    ROS_INFO_STREAM(""<< s[1][0] << " " << s[1][1]);
    
    //Obtener s inversa
    a=s[0][0];
    b=s[0][1];
    c=s[1][0];
    d=s[1][1];
    sinv[0][0]=d/(a*d-b*c);
    sinv[0][1]=-b/(a*d-b*c);
    sinv[1][0]=-c/(a*d-b*c);
    sinv[1][1]=a/(a*d-b*c);
    ROS_INFO_STREAM("Sinv");
    ROS_INFO_STREAM(""<< sinv[0][0] << " " << sinv[0][1]);
    ROS_INFO_STREAM(""<< sinv[1][0] << " " << sinv[1][1]);
    
    //Obtener K
    r1=4;
	c1=4;
	c2=2;
	for(i = 0; i < r1; ++i){
        for(j = 0; j < c2; ++j){
        	aux3[i][j]=0;
            for(k = 0; k < c1; ++k)
            {
                aux3[i][j] += P[i][k] * Htrans[k][j];
            }
        }
    }
    r1=4;
	c1=2;
	c2=2;
	for(i = 0; i < r1; ++i){
        for(j = 0; j < c2; ++j){
        	GanK[i][j]=0;
            for(k = 0; k < c1; ++k)
            {
                GanK[i][j] += aux3[i][k] * sinv[k][j];
            }
        }
    }
    ROS_INFO_STREAM("GanK");
    ROS_INFO_STREAM(""<< GanK[0][0] << " " << GanK[0][1]);
    ROS_INFO_STREAM(""<< GanK[1][0] << " " << GanK[1][1]);
    ROS_INFO_STREAM(""<< GanK[2][0] << " " << GanK[2][1]); 
    ROS_INFO_STREAM(""<< GanK[3][0] << " " << GanK[3][1]);

    //Obtener P gorro
    r1=4;
	c1=2;
	c2=4;
	for(i = 0; i < r1; ++i){
        for(j = 0; j < c2; ++j){
        	aux4[i][j]=0;
            for(k = 0; k < c1; ++k)
            {
                aux4[i][j] += GanK[i][k] * H[k][j];
            }
            aux4[i][j]=Identity[i][j]-aux4[i][j];
        }
    }
    r1=4;
	c1=4;
	c2=4;
	for(i = 0; i < r1; ++i){
        for(j = 0; j < c2; ++j){
        	Pf[i][j]=0;
            for(k = 0; k < c1; ++k)
            {
                Pf[i][j] += aux4[i][k] * P[k][j];
            }
        }
    }
    
    //Obtener poseAct
    r1=4;
	c1=2;
	c2=1;
	for(i = 0; i < r1; ++i){
        for(j = 0; j < c2; ++j){
        	poseAct[i][j]=0;
            for(k = 0; k < c1; ++k)
            {
                poseAct[i][j] += GanK[i][k] * v[k][j];
            }
            poseAct[i][j]=poseAct[i][j]+poseEst[i][j];
        }
    }
    

}	

void inicializacion(){
	int r,c,i,j;
	F[0][0]=1;
	F[0][1]=0;
	F[0][2]=0;
	F[0][3]=0;

	F[1][0]=0;
	F[1][1]=1;
	F[1][2]=0;
	F[1][3]=0;

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

	r=2;
    c=4;
    for(i = 0; i < r; ++i){
        for(j = 0; j < c; ++j)
        {
            Htrans[j][i]=H[i][j];
        }
    }

	poseAct[0][0]=0;
	poseAct[1][0]=0;
	poseAct[2][0]=0;
	poseAct[3][0]=0;

	for(i = 0; i < 4; ++i){
        for(j = 0; j < 4; ++j){
        	Identity[i][j]=0;
        }
	}
    Identity[0][0]=1;
    Identity[1][1]=1;
    Identity[2][2]=1;
    Identity[3][3]=1;

	for(i = 0; i < 4; ++i){
        for(j = 0; j < 4; ++j){
        	Pf[i][j]=Identity[i][j];
        }
	}
	ROS_INFO_STREAM("Pf");
    ROS_INFO_STREAM(""<< Pf[0][0] << " " << Pf[0][1] << " "<< Pf[0][2] << " " << Pf[0][3]);
	ROS_INFO_STREAM(""<< Pf[1][0] << " " << Pf[1][1] << " "<< Pf[1][2] << " " << Pf[1][3]);
    ROS_INFO_STREAM(""<< Pf[2][0] << " " << Pf[2][1] << " "<< Pf[2][2] << " " << Pf[2][3]);
    ROS_INFO_STREAM(""<< Pf[3][0] << " " << Pf[3][1] << " "<< Pf[3][2] << " " << Pf[3][3]);
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
	ros::Rate rate (5);

	rate.sleep();

    inicializacion();
    time1 =ros::Time::now();
	time2 =ros::Time::now();
	ROS_INFO_STREAM("time1=" << time1);
	ROS_INFO_STREAM("time2=" << time2);
	rate.sleep();
	rate.sleep();
	rate.sleep();
	rate.sleep();
	rate.sleep();
	
    
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
			ROS_INFO_STREAM("angulo: "<<angulo<<" distancia: "<<distancia);
			ROS_INFO_STREAM("DeltaT: " << deltaT);
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
			ROS_INFO_STREAM("vMain x="<< v[0][0] << " y=" << v[1][0]);
			ROS_INFO_STREAM("x="<<poseAct[0][0]<<" y="<<poseAct[1][0]<<" Velx="<<poseAct[2][0]<<" Vely="<<poseAct[3][0]);
		}
		
		ros::spinOnce();
		// Espera hasta que es tiempo de la siguiente iteracion
		rate.sleep();
	}	
	
}
