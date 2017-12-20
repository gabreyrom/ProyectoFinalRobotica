# Proyecto Final Robótica 
## ITAM

###### *Diego Amaya 149119
###### *Gabriel Reynoso 150904
###### *Gumer Rodríguez 149109
###### *Julio Sánchez 148221

Este repositorio incluye los programas que cumplen con las tres partes del proyecto. El archivo histograma.cpp corresponde a la primera parte (Estimación de estado propio), kalman.cpp corresponde a la segunda (Estimación de estado de obstáculo móvil) y seguimiento.cpp a la tercera (Movimiento para seguir obstáculo móvil). Este proyecto fue hecho en ROS Kinetic. Para la implementación de Gazebo se supone que ya se descargaron los modelos y se ejecuta sin problemas el EK_AutoNOMOS. La documentación está en: https://github.com/EagleKnights/SDI-11911/wiki. También que se tiene descargada la rosbag.bag del link: robotica.itam.mx/rosbags/rosbag_SDI11911.bag . La rosbag debe estar dentro del workspace.
El primer paso es situarse en la carpeta src y clonar este repositorio mediante el comando:

```
git clone https://github.com/gabreyrom/proyecto_final.git
```
Situarse en el workspace de ROS y ejecutar los siguientes comandos:
```
catkin_make
source devel/setup.bash
```
Antes de comenzar con la simulación se tiene que iniciar el roscore en una terminal.

### Parte 1:
Para esta parte se necesitan los datos de la bolsa. Se abren dos terminales situadas en el workspace y en una de ellas se corre el comando:
```
rosbag play rosbag.bag
```
Y en la otra terminal:
```
rosrun proyecto_final histograma
```
En esta ventana se imprimirá la posición más probable dependiendo de los datos leídos de la bolsa.

### Parte 2 y 3:
En esta seccion se utiliza Gazebo, por lo que para inicializarlo se necesita situarse en el directorio de AutoNOMOS_simulation y correr:

```
roslaunch autonomos_gazebo straight_road.launch
```
Una vez abierto, se verán los dos autos en Gazebo. Ahora, estando en el workspace de ROS, se correrá el código que inicializará el filtro de Kalman y el código que permite seguir al otro auto. En terminales diferentes se publican la velocidad y ángulo de giro del carro a seguir.

En una terminal:
```
rosrun proyecto_final kalman
```
En otra terminal:
```
rosrun proyecto_final seguimiento
```
En la terminal para cambiar el giro del volante:
```
rostopic pub /AutoNOMOS_mini_2/manual_control/steering /std_msgs/Float32 '{data: VALUE}'
```
Para cambiar la velocidad:
```
rostopic pub /AutoNOMOS_mini_2/manual_control/velocity /std_msgs/Float32 '{data: VALUE}'

```
En la ventana de Gazebo se verá cómo un carro sigue al otro.
