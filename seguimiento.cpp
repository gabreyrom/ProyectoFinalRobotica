#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <stdlib.h> 
#include <iostream>
#include <math.h>

//variables recibidas
double x, y, theta, xd, yd, thetad, L, hz, e, a, deltax, deltay, vectx, vecty;
//variables internas de control
bool flag1,flag2;
//variable de signo
int sign;

// callback para leer la posicion del movil
// void poseMessageReceived (const geometry_msgs::Pose2D& msg1){
// 	x=msg1.x;
// 	y=msg1.y;
// 	theta=msg1.theta;
// 	flag1=true;
// }

// callback para leer la posicion deseada del movil (posición del obstáculo)
void poseDMessageReceived (const geometry_msgs::Twist& msg2){
	xd=linear.x;
	yd=linear.y;
	Velxd=angular.x;
	Velyd=angular.y;
	flag2=true;
	//Condiciones iniciales	
	deltax=(xd-0);
	deltay=(yd-0);
	vectx=deltay;
	vecty=deltay;
	a=deltax*vectx+deltay*vecty;
	//Si el punto deseado esta detras del movil, avanza de reversa
	ROS_INFO_STREAM("Producto punto: "<< a);
	if (a<0){
		sign=-1;
	}else{
		sign=1;
	}
}

int main (int argc, char **argv){
	//variables internas
	double rho, alpha, beta, krho, kalpha, kbeta, w, error;	
	//variables de publicacion
	double v, gamma;

	flag1=false;
	flag2=false;
	
	// Inicializa ROS system y crea un nodo.
	ros::init(argc, argv, "seguimiento_obstaculo");
	ros::NodeHandle nh ;
	// Crea un objeto publicador .
	ros::Publisher pubS=nh.advertise<std_msgs::Float32>("/AutoNOMOS_mini_1/manual_control/steering",1000);
	ros::Publisher pubV=nh.advertise<std_msgs::Float32>("/AutoNOMOS_mini_1/manual_control/velocity",1000);
	// Crea un objeto suscriptor
	ros::Subscriber subP = nh.subscribe("/pose_objetivo", 1000, &poseMessageReceived);
	//ros::Subscriber subPD = nh.subscribe("robot/next_pose", 1000, &poseDMessageReceived);
	//Crea el objeto mensaje
	std_msgs::Float32 msgSteering;
	std_msgs::Float32 msgVelocity;

	// Ciclo a 10 Hz
	ros::Rate rate (5);

	//Constante del auto	
	L=0.32;

	//Valores de control
	krho=1;
	kalpha=1.5;
	kbeta=-1;
	e=0.2;
	sign=1;
	
	//Ángulos alpha y beta se calculan en radianes
	//STEERING ESTA EN GRADOS

	//Ciclo principal
    	while (ros::ok()) {
		if (flag2){
			//Calculos para pose deseada
			deltax=(xd-0);
			deltay=(yd-0);
			//Detener si la distancia es tolerable
			error=sqrt(deltax*deltax+deltay*deltay);
			if(e>error){
				gamma=0;
				v=0;
				flag2=false;	
			}else{
				rho=error;
				alpha=(atan(deltay/deltax)-theta)*180/3.1416;
				beta=-(theta*180/3.1416)-alpha;

				v=krho*rho*sign;
				gamma=(kalpha*alpha+kbeta*beta)*sign;
				//Poner maximo a velocidad y giro
				if(v>6){
					v=6;
				}else if(v<-6){
					v=-6;
				}
				if(gamma>90)
					gamma=90;
				else if(gamma<-90){
					gamma=-90;
				}
			}
			
			//Publicar control
			msgSteering.data=gamma;
			msgVelocity.data=v;
			pubS.publish(msgSteering);
			pubV.publish(msgVelocity);
			ROS_INFO_STREAM("Velocity: "<< v<< ", Gamma: " << gamma);
			
			//flag1=false;
		}else{
			//No se sabe de la ubicacion del auto
			//Publicar detenerse
			msgSteering.data=0;
			msgVelocity.data=0;
			pubS.publish(msgSteering);
			pubV.publish(msgVelocity);
			ROS_INFO_STREAM("Velocity: 0" << ", Gamma: 0");

		}
		
		ros::spinOnce();
		// Espera hasta que es tiempo de la siguiente iteracion
		rate.sleep();
	}	
	
}
