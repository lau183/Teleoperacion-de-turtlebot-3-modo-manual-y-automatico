Projecto desarrollado por: [Zuleika María Redondo García] y [Laura García García]
# Teleoperacion-de-turtlebot-3-modo-manual-y-automatico

Repositorio de la práctica 3 de la asignatura de robots móviles del grado de Ingeniería robótica de la Universidad de Alicante

## Requisitos
- ROS noetic
- [Paquete turtlebot3]
- [Paquete frontier exploration]
- [Paquete Navigation stack]

## Instalación
```sh
conda create -n <nombre_entorno> python=3.8
conda activate <nombre_entorno>
pip install -r requirements.txt
sudo apt install ros-noetic-arbotix
```

## Preparación del entorno de Gazebo

1. Descomprimir la carpeta "modelos_arucos.tar.gz" en la ruta /.gazebo/models
2. Descomprimir la carpeta "launch_mundos.tar.gz" en /turtlebot3_simulations/turtlebot3_gazebo/launch
3. Descomprimir la carpeta "mundos.tar.gz" en /turtlebot3_simulations/turtlebot3_gazebo/worlds

# Guía de ejecución

En todos los terminales se debe incluir el modelo de  usado en la variable TURTLEBOT3_MODEL, de la siguiente forma:
```sh
export TURTLEBOT3_MODEL=waffle
```

## Modo manual

Terminal 1: Simulación del turtlebot en gazebo con el entorno creado

```sh
 roslaunch turtlebot3_gazebo turtlebot3_aruco_world_2_wa.launch
```

Terminal 2: Script para enviar el movimiento al robot en función del comando recibido

```sh
rosrun manos move_manual.py
```
Después, se añade la plantilla "RobotModel" en _rviz_ para visualizar el movimiento del robot.

Terminal 3: Ejecutable para la simulacion y planificacion de movimiento del brazo _PhantomX Reactor Robot_.

```sh
roslaunch phantomx_reactor_arm_moveit_config demo.launch
```

Terminal 4: Script para la selección y ejecución del modo

```sh
rosrun move hands.py
```

Aparecerá la interfaz gráfica y se selecciona el modo manual. Una vez seleccionado este modo, aparecerá la cámara y para cada uno de los gestos indicados, se tendrá que mostrar el gesto delante de la cámara y guardar con la tecla "t".

Ya con todos los gestos introducidos, se puede utilizar sin problemas, y comenzar a dirigir al robot.

## Modo automático y automático teleoperado

Terminal 1:  Simulación del turtlebot en gazebo con el entorno creado.

```sh
roslaunch turtlebot3_gazebo turtlebot3_aruco_world_2_wa.launch 
```

Terminal 2: Abre el entorno en rviz y lo prepara para su mapeo con el método indicado.

```sh
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
Terminal 3:  Ejecutable para permitir el empleo de la acción move_base para la planificación y ejecución de trayectorias del robot sobre el mapa.

```sh
roslaunch turtlebot3_navigation move_base.launch
```

Terminal 4: Ejecutable que realiza la detección y estimación de posición y orientación de los Arucos visualizados por el robot. Además, abre rviz para mostrar los resultados.

```sh
roslaunch aruco_detector_ocv detector.launch
```

Terminal 5: Script para la selección y ejecución del modo.

```sh
rosrun move hands.py 
```
 Aparecerá la interfaz de usuario y se seleccionará el modo automático o el automático teleoperado.



















[Paquete turtlebot3]: https://github.com/ROBOTIS-GIT/turtlebot3.git
[Paquete frontier exploration]: https://github.com/nocoinman/frontier_exploration.git
[Paquete aruco detector osv]: https://github.com/CesMak/aruco_detector_ocv.git
[Paquete Navigation stack]: https://github.com/ros-planning/navigation.git
[Zuleika María Redondo García]: https://github.com/zuleikarg
[Laura García García]: https://github.com/lau183
