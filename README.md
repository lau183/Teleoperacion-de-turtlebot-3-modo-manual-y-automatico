# Teleoperacion-de-turtlebot-3-modo-manual-y-automatico

Repositorio de la práctica 3 de la asignatura de robots móviles del grado de Ingeniería robótica de la Universidad de Alicante

## Requisitos
- ROS noetic
- OpenCV
- guizero
- [Paquete turtlebot3]
- [Paquete frontier exploration]
- [Paquete aruco detector osv]

## Guía de ejecución
En todos los terminales se debe incluir el modelo de  usado en la variable TURTLEBOT3_MODEL, de la siguiente forma:
```sh
export TURTLEBOT3_MODEL=waffle
```
# Modo manual

Terminal 1: Simulación del turtlebot en gazebo con el entorno creado

```sh
 roslaunch turtlebot3_gazebo turtlebot3_aruco_world_2_wa.launch
```

Terminal 2: Script para enviar el movimiento al robot en función del comando recibido

```sh
rosrun manos move_manual.py
```

Terminal 3: Script para la selección y ejecución del modo

```sh
rosrun move hands.py
```

Aparecerá la interfaz gráfica y se selecciona el modo manual. Una vez seleccionado este modo, aparecerá la cámara y para cada uno de los gestos indicados, se tendrá que mostrar el gesto delante de la cámara y guardar con la tecla "t".

Ya con todos los gestos introducidos, se puede utilizar sin problemas, y comenzar a dirigir al robot.



















[Paquete turtlebot3]: https://github.com/ROBOTIS-GIT/turtlebot3.git
[Paquete frontier exploration]: https://github.com/nocoinman/frontier_exploration.git
[Paquete aruco detector osv]: https://github.com/CesMak/aruco_detector_ocv.git
