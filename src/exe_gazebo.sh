#!/bin/bash
cd ~/Desktop/moviles
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle
# escenarios abiertos: turtlebot3_stage_1 | turtlebot3_empty_world
# Verifica si se proporcionó un parámetro
if [ -z "$1" ]; then
    echo "Error: Debes proporcionar el nombre del archivo Python sin la extensión."
    echo "Uso: ./exe.sh <nombre_archivo>"
    exit 1
fi

# Construye el nombre del archivo Python
FILE_NAME="$1.launch"

# Ejecuta el .launch dado por parametro

roslaunch turtlebot3_gazebo $FILE_NAME 
