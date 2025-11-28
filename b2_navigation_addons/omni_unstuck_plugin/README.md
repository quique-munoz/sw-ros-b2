# Omni Unstuck plugin
Este paquete incluye técnicamente dos plugins. Uno implementa un comportamiento que se encarga de sacar un robot omnidireccional de posiciones en las que el planificador o controlador fallan y no consiguen salir (`omni_unstuck_behavior`). El otro plugin simplemente se encarga de llamar al anterior desde el behavior tree (`omni_unstuck_bt_node`).

El comportamiento se puede modificar desde el behavior tree, modificando los parámetros:
* `dist_to_travel`: distancia que tiene que moverse el vehículo para darse por finalizado el comportamiento de desatascado.
* `speed`: velocidad de movimiento.
* `time_allowance`: tiempo máximo que se puede pasar en el comportamiento.

Ejemplo de definición del nodo del BT con parámetros:
```
<OmniUnstuck dist_to_travel="1.0" 
         speed="0.5" 
         time_allowance="3.0"
         server_timeout="5" />
```

## Notas
El comportamiento (`omni_unstuck_behavior`) crea un servidor de acción, por lo que se puede llamar esta acción para probar el correcto funcionamiento del comportamiento, sin que el BT tenga que llegar al nodo.

```
ros2 action send_goal /omni_unstuck nav2_msgs/action/DriveOnHeading -f
    'target:
        x: 1.0
        y: 0.0
        z: 0.0
    speed: 0.5
    time_allowance:
        sec: 0
        nanosec: 0' 
```