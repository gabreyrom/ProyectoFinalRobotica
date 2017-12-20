//Proyecto Final Robotica - Parte 3
//Equipo: Trifuerza & Ganondorf
//Programa de control para seguir un objetivo. Las coordenas de dicho objetivo
//son obtenidos a traves de un topico
//Se implementa la estrategia "Mover a un punto"
//Los comandos de velocidad y angulo de volante se mandan a traves de los topicos
//AutoNOMOS_mini_1/manual_control/velocity y /AutoNOMOS_mini_1/manual_control/steering
//respectivamente

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <stdlib.h> 
#include <iostream>
#include <math.h>

//variables recibidas
double xd, yd, L, hz, e, deltax, deltay, Velxd, Velyd;
//variables internas de control
bool flag1,flag2;
//variable de signo
int sign;

// callback para leer la posicion deseada del movil (posición del obstáculo)
void poseMessageReceived (const geometry_msgs::Twist& msg2){
	xd=msg2.linear.x;
	yd=msg2.linear.y;
	Velxd=msg2.angular.x;
	Velyd=msg2.angular.y;
	flag2=true;
	//Condiciones iniciales	
	deltax=(xd-0);
	deltay=(yd-0);
	//Si el punto deseado esta detras del movil, el angulo es negativo
	if (deltay<0 && deltax<0){
		sign=-1;
	}else{
		sign=1;
	}
}

int main (int argc, char **argv){
	//variables internas
	double krho, kgamma, error;	
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
	//Crea el objeto mensaje
	std_msgs::Float32 msgSteering;
	std_msgs::Float32 msgVelocity;

	// Ciclo a 10 Hz
	ros::Rate rate (5);

	//Constante del auto	
	L=0.32;

	//Valores de control
	krho=4;
	kgamma=2;
	
	e=1;
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
				gamma=kgamma*atan(yd/xd)*180/3.1416*sign;
				v=krho*sqrt(xd*xd + yd*yd);
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
