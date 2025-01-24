
# Project Title

A brief description of what this project does and who it's for


## Preparación Inicial

Antes de seguir las instrucciones de uso, hay que corroborar que los diferentes archivos están listos para
la manera de despliegue deseada (simulación o real). Hay comentarios que indican qué líneas de código
son adecuadas para cada situación. Compruebe:

- Delivery_State.py: Línea 53, el proceso que lanzan esas funciones solo es necesario en simulación. Además, si se va a usar en simulación, ajustar la ruta de guardado del mapa en la línea 132.
- Main.py: Usar Línea 17 o Línea 20, en función del modo deseado.

Todo el proyecto debe alojarse dentro de un espacio de trabajo de catkin. Para ello, siga estos pasos:
```
mkdir tu_workspace
cd tu_workspace
mkdir src
cd src
git clone https://github.com/JuanPerez07/Proyecto-Moviles/tree/Repositorio-Proyecto-Mailguard
cd ..
catkin_make
source devel/setup.bash 
```

Con el entorno de trabajo preparado, siga una de las dos secciones a continuación para el despliegue del
proyecto en función del medio que desee:

## Simulación

Indicar que el modelo usado es el Waffle:

```export TURTLEBOT_3D_MODEL=waffle```

Lanzar el mundo de Gazebo que desee (empty, world...):

```roslaunch turtlebot3_gazebo turtlebot3_world.launch```

Lanzar la Máquina de estados, va a iniciar el mapeado (Rviz) y se va a quedar a la espera de recibir la transformación:

```python Main.py```

Lanzar el script en otra terminal para encontrar y enviar la transformación, una vez ejecutado deberia continuar el proceso de la máquina de estados:

```python find_transform.py```

Lanzar el paquete mi_navigation desde el workspace, contiene los paquetes para el move base action:

```roslaunch navigation_stage mi_navigation```

Ejecutar el script, encargado de la detección de la referencia del cartero:

```python tracking.py```

Ejecutar la interfaz de usuario, en otra terminal:

```python Nodo_Interfaz.py```


## Robot Real
1. Conectar con el Turtlebot para lanzar los paquetes:
1.1. En una terminal:

```roslaunch turtlebot_bringup minimal.launch```

1.2. En otra terminal:

```roslaunch turtlebot_bringup hokuyo_ust10lx.launch```

1.3. En otra terminal:

```
export TURTLEBOT_3D_SENSOR=astra
roslaunch turtlebot_navigation gmapping_demo.launch
```

2. Desde el espacio de trabajo en su equipo:
2.1. Lanzar la Máquina de estados, va a iniciar el mapeado (Rviz) y se va a quedar a la espera de recibir la transformación:

```python Main.py```

2.2. Lanzar el script en otra terminal para encontrar y enviar la transformación, una vez ejecutado deberia continuar el proceso de la máquina de estados:

```python find_transform.py```

2.3. Lanzar el paquete mi_navigation desde el workspace, contiene los paquetes para el move base action:

```roslaunch navigation_stage mi_navigation```

2.4 Ejecutar el script, encargado de la detección de la referencia del cartero:

```python tracking.py```

2.5. Ejecutar la interfaz de usuario, en otra terminal:

```python Nodo_Interfaz.py```

2.6. Usar la máquina de estados a través de la interfaz.
