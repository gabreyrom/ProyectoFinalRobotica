#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <stdlib.h> 
#include <iostream>
#include <math.h>

/*Arreglo de probabilidades para cada una de las probabilidades
 *Dependiendo de qué líneas se ven, es dónde puede estar el auto 
 *
 *Left	Center	Right		Probabilidad
 *0	0	0		Desconocido
 *0	0	1		Derecha-->Sobre línea derecha
 *0	1	0		Centro-->Centro con probabilidad en carriles
 *0	1	1		CentroDer-->Carril derecho
 *1	0	0		Izquierda-->Sobre línea izquierda
 *1	0	1		Centro-->Centro con probabilidad en carriles
 *1	1	0		CentroIzq-->Carril izquierdo
 *1	1	1		Centrado-->Sobre la línea central
 *
*/
double estado[8]={0.125,0.125,0.125,0.125,0.125,0.125,0.125,0.125};
double desconocido[8]={0.015,0.015,0.015,0.015,0.015,0.015,0.015,0.895};
double derecha[8]={0.015,0.015,0.015,0.015,0.2,0.225,0.5,0.015};
double centro[8]={0.015,0.015,0.05,0.825,0.05,0.015,0.015,0.015};
double centroDer[8]={0.015,0.015,0.015,0.1,0.725,0.1,0.015,0.015};
double izquierda[8]={0.5,0.225,0.2,0.015,0.015,0.015,0.015,0.015};
double centroIzq[8]={0.015,0.1,0.725,0.1,0.015,0.015,0.015,0.015};
double centrado[8]={0.015,0.015,0.25,0.425,0.25,0.015,0.015,0.015};

//variables internas de control sobre qué lína ve
bool line_left,line_right,line_center;


// callback para leer la posicion del movil
void lineLeft (const std_msgs::Int32MultiArray& msg1){
	line_left=true;
}

void lineCenter (const std_msgs::Int32MultiArray& msg2){
	line_center=true;
}

void lineRight (const std_msgs::Int32MultiArray& msg3){
	line_right=true;
}

//Función que hace la multiplicación de probabilidades y su normalización
void actualizaEst (double a[], double b[]){

	int i,j;
	double suma=0.0;
	double edoAct[8]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

	for(i=0;i<8;i=i+1){
		edoAct[i]=a[i]*b[i];
		suma=suma+edoAct[i];
	}

	for(j=0;j<8;j=j+1){
		edoAct[j]=edoAct[j]/suma;
		estado[j]=edoAct[j];
	}


} 


int main (int argc, char **argv){

//Se inicializan las banderas como verdaderas
	line_left=true;
	line_right=true;
	line_center=true;
	
	// Inicializa ROS system y crea un nodo.
	ros::init(argc, argv, "robot_controller");
	ros::NodeHandle nh ;
	// Crea un objeto publicador para el estadoEstimado (probabilidad de estar en cada línea)
	ros::Publisher pubEst=nh.advertise<std_msgs::Float32MultiArray>("/estadoEstimado",1000);
	// Crea un objeto suscriptor
	ros::Subscriber subPCentro = nh.subscribe("/points/center", 1000, &lineCenter);
	ros::Subscriber subPDerecha = nh.subscribe("/points/right", 1000, &lineRight);
	ros::Subscriber subPIzquierda = nh.subscribe("/points/left", 1000, &lineLeft);
	//Crea el objeto mensaje
	std_msgs::Float32MultiArray array;


	// Ciclo a hz Hz
	ros::Rate rate (4);

	while (ros::ok()) {
	//Se toman todas las condiciones posibles para cada iteración
		if (line_left && line_center && line_right)
		{
			actualizaEst(estado,centrado);
			ROS_INFO_STREAM("Centrado");
		}
		else if (line_left && line_center && !line_right)
		{
			actualizaEst(estado,centroIzq);
			ROS_INFO_STREAM("CentroIzq");
		}
		else if (line_left && !line_center && line_right)
		{
			actualizaEst(estado,centro);
			ROS_INFO_STREAM("Centro");
		}
		else if (line_left && !line_center && !line_right)
		{
			actualizaEst(estado,izquierda);
			ROS_INFO_STREAM("Izquierda");
		}
		else if (!line_left && line_center && line_right)
		{
			actualizaEst(estado,centroDer);
			ROS_INFO_STREAM("CentroDer");
		}
		else if (!line_left && line_center && !line_right)
		{
			actualizaEst(estado,centro);
			ROS_INFO_STREAM("Centro");
		}
		else if (!line_left && !line_center && line_right)
		{
			actualizaEst(estado,derecha);
			ROS_INFO_STREAM("Derecha");
		}
		else{
			actualizaEst(estado,desconocido);
			ROS_INFO_STREAM("Desconocido");
		}

		//Se limpia el arreglo auxiliar para publicar y se le meten los datos
		array.data.clear();

		for (int i = 0; i < 8; i++){
			array.data.push_back(estado[i]);
		}
		//Se publica y se vuelven falsas las banderas
		pubEst.publish(array);
		line_left=false;
		line_right=false;
		line_center=false;
		ros::spinOnce();
		// Espera hasta que es tiempo de la siguiente iteracion
		rate.sleep();
	}
}	
	

