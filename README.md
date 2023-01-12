Projecto desarrollado por: [Zuleika María Redondo García] y [Laura García García]
# Teleoperacion-de-turtlebot-3-modo-manual-y-automatico

Repositorio de la práctica 3 de la asignatura de robots móviles del grado de Ingeniería robótica de la Universidad de Alicante

## Requisitos
- ROS noetic
- [Paquete turtlebot3] para probar proyecto en simulación
- [Paquete frontier exploration] paquete desarrollado por Ashwin A Nayar
- [Paquete Navigation stack]
- [Paquete turtlebot 2] para probar proyecto en robot real

## Instalación
```sh
conda create -n <nombre_entorno> python=3.8
conda activate <nombre_entorno>
pip install -r requirements.txt
sudo apt install ros-noetic-arbotix
sudo apt install ros-noetic-moveit*
```
# Guía de ejecución en simulación con turtlebot 3

## Preparación del entorno de Gazebo para simulación del turtlebot 3

1. Descomprimir la carpeta "modelos_arucos.tar.gz" en la ruta /.gazebo/models
2. Descomprimir la carpeta "launch_mundos.tar.gz" en /turtlebot3_simulations/turtlebot3_gazebo/launch
3. Descomprimir la carpeta "mundos.tar.gz" en /turtlebot3_simulations/turtlebot3_gazebo/worlds

## Preparación de scripts

Para poder usar el modo automático:
- Indicar en el script hands.py y el  hands_with_teleop.py  en la variable FILE_PATH_COSTMAP la ruta donde se tenga el archivo explore_costmap.launch del paquete frontier_exploration.
- Indicar en el script hands_with_teleop.py en la variable FILE_PATH_TELEOP la ruta donde se tenga el archivo turtlebot3_teleop.launch del paquete de turtlebot3

## Ejecución proyecto

En todos los terminales se debe incluir el modelo de  usado en la variable TURTLEBOT3_MODEL, de la siguiente forma:
```sh
export TURTLEBOT3_MODEL=waffle
```

### Modo manual

Terminal 1: Simulación del turtlebot en gazebo con el entorno creado

```sh
 roslaunch turtlebot3_gazebo turtlebot3_aruco_world_2_wa.launch
```

Terminal 2: Ejecutable para la simulacion y planificacion de movimiento del brazo _PhantomX Reactor Robot_.

```sh
roslaunch phantomx_reactor_arm_moveit_config demo.launch
```
Después, se añade la plantilla "RobotModel" en _rviz_ para visualizar el movimiento del robot.

Terminal 3: Script para enviar el movimiento al robot en función del comando recibido

```sh
rosrun manos move_manual.py
```

Terminal 4: Script para la selección y ejecución del modo

```sh
rosrun manos hands.py
```

Aparecerá la interfaz gráfica y se selecciona el modo manual. Una vez seleccionado este modo, aparecerá la cámara y para cada uno de los gestos indicados, se tendrá que mostrar el gesto delante de la cámara y guardar con la tecla "t".

Ya con todos los gestos introducidos, se puede utilizar sin problemas, y comenzar a dirigir al robot.

### Modo automático y automático teleoperado

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
rosrun manos hands.py 
```
En caso, de que se quiera usar el modo automático teleoperado, para que en la interfaz aparezeca dicha opción se debe usar el script hands_with_teleop.py en lugar del hands.py.

```sh
rosrun manos hands_with_teleop.py 
```
 Aparecerá la interfaz de usuario y se seleccionará el modo automático o el automático teleoperado.


# Guía de ejecución en  robot real turtlebot 2
## Preparación de scripts

Para poder usar el modo automático:
- Indicar en el script hands.py y el  hands_with_teleop.py  en la variable FILE_PATH_COSTMAP  la ruta donde se tenga el archivo explore_costmap.launch del paquete frontier_exploration.
-Indicar en el script hands_with_teleop.py en la variable FILE_PATH_TELEOP  la ruta donde se tenga el archivo  keyboard_teleop.launch del paquete de turtlebot_teleop

## Ejecuión proyecto
PA
Se debe conectar el ordenador a la red local del laboratorio e inicializar las siguientes variables de entorno en cada uno de los terminales que se usen o bien tenerlas inicializadas en el bashrc:
```sh
export ROS_MASTER_URI=http://<ip_turtlebot>:11311
export ROS_HOSTNAME=<ip_turtlebot>
export ROS_IP=<ip_ordenador>
```

Para conectarse al robot real se hace a través de SSH:
- Si  se usa el Turtlebot 5, se emplea el siguiente comando:
```sh
 ssh tb2@<ip_turtlebot>
```
En caso de usar otro de los cuatro Turtlebots presentes en el laboratorio se usa el comando:
```sh
ssh turtlebot@<ip_turtlebot>
```

### Modo manual

Abrimos los siguientes terminales desde dentro del robot (SSH):

Terminal 1:  Arranque mínimo del robot.

```sh
roslaunch turtlebot_bringup minimal.launch
```

 Terminal 2: Controlador para el uso del brazo del robot. 
 ```sh
roslaunch phantomx_reactor_arm_controller arbotix_phantomx_reactor_arm_wrist.launch
```

Abrimos los siguientes terminales desde fuera del robot:

 Terminal 1: Script para enviar el movimiento al robot en función del comando recibido.
 ```sh
rosrun manos move_manual.py
```

Terminal 2: Script para la selección y ejecución del modo.
 ```sh
 rosrun manos hands_with_teleop.py 
```

### Modo automático y automático teleoperado

Abrimos los siguientes terminales desde dentro del robot (SSH):

Terminal 1:  Arranque mínimo del robot.
```sh
roslaunch turtlebot_bringup minimal.launch
```
Terminal 2: Controlador para utilizar el láser del robot.
```sh
 roslaunch turtlebot_bringup hokuyo_ust10lx.launch
```
Terminal 3: Controlador para utilizar la cámara del robot.
```sh
 roslaunch astra_launch astra.launch
```
Terminal 4: Ejecutable para permitir la realización del mapeo con la creación de los topics necesarios, entre ellos, uno esencial para la tarea, move_base.
```sh
 export TURTLEBOT_3D_SENSOR=astra 
 roslaunch turtlebot_navigation gmapping_demo.launch 
```

Abrimos los siguientes terminales desde fuera del robot:

Terminal 1: Opertura del entorno de rviz para visualizar el mapeo, la posición del robot y, el láser y la cámara del robot, principalmente.
```sh
rosrun rviz rviz
```

 Una vez se haya abierto, se cambia el fixed frame a /odom y es necesario incluir la plantilla:
 - LaserScan con topic /scan
 - Axes con topic /base_link
 - Map con topic /map
 - RobotModel con topic /camera/rgb/image_rect_color

Terminal 2: Script para la selección y ejecución del modo.
```sh
 rosrun manos hands.py
```

 En caso, de que se quiera usar el modo automático teleoperado, para que en la interfaz aparezeca dicha opción se debe usar el script hands_with_teleop.py  en lugar del hands.py. Para ello usar el siguiente comando:
 
 ```sh
 rosrun manos hands_with_teleop.py
```


## Referencias usadas
- [Código detección gestos]
- [Detección de Arucos] 









[Detección de Arucos]: https://github.com/CesMak/aruco_detector_ocv
[Código detección gestos]: https://toptechboy.com/improved-gesture-recognition-in-python-and-mediapipe/
[Paquete turtlebot 2]: https://github.com/turtlebot/turtlebot.git
[Paquete turtlebot3]: https://github.com/ROBOTIS-GIT/turtlebot3.git
[Paquete frontier exploration]: https://github.com/nocoinman/frontier_exploration.git
[Paquete aruco detector osv]: https://github.com/CesMak/aruco_detector_ocv.git
[Paquete Navigation stack]: https://github.com/ros-planning/navigation.git
[Zuleika María Redondo García]: https://github.com/zuleikarg
[Laura García García]: https://github.com/lau183
