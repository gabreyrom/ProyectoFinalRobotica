#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
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
void poseMessageReceived (const std_msgs::Int32MultiArray& msg1){
	x=msg1.x;
	y=msg1.y;
	theta=msg1.theta;
	flag1=true;
}


int main (int argc, char **argv){
	//variables internas
	double rho, alpha, beta, krho, kalpha, kbeta, w, error;	
	//variables de publicacion
	double v, gamma;

	flag1=false;
	flag2=false;
	
	// Inicializa ROS system y crea un nodo.
	ros::init(argc, argv, "robot_controller");
	ros::NodeHandle nh ;
	// Crea un objeto publicador .
	ros::Publisher pubV=nh.advertise<std_msgs::Float32>("/AutoNOMOS_mini/manual_control/velocity",1000);
	// Crea un objeto suscriptor
	ros::Subscriber subPCentro = nh.subscribe("/points/center", 1000, &poseMessageReceived);
	ros::Subscriber subPDerecha = nh.subscribe("/points/right", 1000, &poseMessageReceived);
	ros::Subscriber subPIzquierda = nh.subscribe("/points/left", 1000, &poseDMessageReceived);
	//Crea el objeto mensaje
	std_msgs::Float32 msgSteering;
	std_msgs::Float32 msgVelocity;


	//Obtener informacion del usuario
	ROS_INFO_STREAM("Indique los Hertz");
	std::cin >> hz;

	// Ciclo a hz Hz
	ros::Rate rate (hz);
	
	//Ángulos alpha y beta se calculan en radianes
	//STEERING ESTA EN GRADOS

	//Ciclo principal
    	while (ros::ok()) {
		if (flag1 && flag2){
			//Calculos para pose deseada
			ROS_INFO_STREAM("Velocity: "<< v<< ", Gamma: " << gamma);
			
			flag1=false;
		}else{
			ROS_INFO_STREAM("Velocity: 0" << ", Gamma: 0");

		}
		
		ros::spinOnce();
		// Espera hasta que es tiempo de la siguiente iteracion
		rate.sleep();
	}	
	
}
