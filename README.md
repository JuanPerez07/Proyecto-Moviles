# Práctica 3 de robots móviles

## Ejecutar la simulación

```bash
roslaunch navigation_stage mi_navigation.launch
```
## Ejemplos

En la carpeta `src`

- `test_smach.py` es un ejemplo básico de una máquina de estados de SMAC con dos estados que se van alternando
- `smach_actionstate.py` es un ejemplo de cómo llamar a una acción de ROS (en este caso una MoveBaseAction) desde SMACH
- `practica3_base.py` ejemplo que podéis tomar como punto de partida para vuestro código de la práctica 3
