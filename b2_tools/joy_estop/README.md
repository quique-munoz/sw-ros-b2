# Joy E-Stop

## Descripción

Este nodo ROS 2 convierte las señales de botones de un joystick en mensajes de parada de emergencia (E-Stop). Permite activar y desactivar el estado de emergencia del robot mediante botones específicos del controlador. Es especialmente útil para proporcionar una forma rápida y accesible de detener el robot en caso de emergencia.

## Funcionalidad

El nodo escucha los mensajes del joystick (`sensor_msgs/msg/Joy`) y publica mensajes booleanos (`std_msgs/msg/Bool`) que indican el estado de la parada de emergencia:
- **True**: E-Stop activado (robot detenido)
- **False**: E-Stop desactivado (robot puede moverse)

## Parámetros

| Parámetro | Valor por Defecto | Descripción |
|-----------|-------------------|-------------|
| `input_topic` | `/joy` | Tópico de entrada para los mensajes del joystick |
| `output_topic` | `/e_stop` | Tópico de salida para los mensajes de E-Stop |
| `estop_button` | `1` | Número del botón para activar E-Stop (Círculo en PS5) |
| `reset_button` | `2` | Número del botón para desactivar E-Stop (Triángulo en PS5) |

## Mapeo de Botones para PS5

| Botón | Número | Descripción |
|-------|--------|-------------|
| X | 0 | Cuadrado |
| Círculo | 1 | **E-Stop por defecto** |
| Triángulo | 2 | **Reset por defecto** |
| Cuadrado | 3 | Cuadrado |
| L1 | 4 | Gatillo izquierdo superior |
| R1 | 5 | Gatillo derecho superior |
| L2 | 6 | Gatillo izquierdo inferior |
| R2 | 7 | Gatillo derecho inferior |
| Select | 8 | Botón Select |
| Start | 9 | Botón Start |
| PS | 10 | Botón PlayStation |
| L3 | 11 | Stick izquierdo presionado |
| R3 | 12 | Stick derecho presionado |

## Uso

### Lanzamiento Básico

Para lanzar el nodo con los parámetros por defecto:

```bash
ros2 launch joy_estop joy_estop.launch.py
```

### Lanzamiento con Parámetros Personalizados

Para configurar botones específicos o cambiar los tópicos:

```bash
ros2 launch joy_estop joy_estop.launch.py estop_button:=0 reset_button:=3
```

### Ejemplo Completo

```bash
ros2 launch joy_estop joy_estop.launch.py \
    input_topic:=/my_joy \
    output_topic:=/my_e_stop \
    estop_button:=0 \
    reset_button:=3
```

## Integración con Twist Mux

Este nodo está diseñado para trabajar con `twist_mux`, que puede usar el tópico `/e_stop` para bloquear los comandos de velocidad cuando se activa la parada de emergencia. El `twist_mux` debe configurarse con la prioridad más alta (255) para el lock de E-Stop.

## Notas Importantes

- Los botones de E-Stop y Reset no pueden ser el mismo
- El nodo verifica que el mensaje del joystick contenga suficientes botones
- Solo se publica un mensaje cuando se presiona el botón de E-Stop o Reset
- El estado de E-Stop se mantiene hasta que se presiona el botón de Reset