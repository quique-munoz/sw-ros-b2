# Twist -> TwistStamped

## Descripción

Este nodo ROS 2 convierte mensajes del tipo `geometry_msgs::msg::Twist` a `geometry_msgs::msg::TwistStamped`, añadiendo un `header` con el timestamp actual y un sistema de referencia predefinido. La idea es poder visualizar los comandos de velocidad comandados a un robot en Rviz, que no se puede hacer con Twist porque no tiene header.

## Parámetros

| Parámetro | Valor por Defecto | Descripción |
|-----------|-------------------|-------------|
| `input_topic` | `/cmd_vel` | Tópico de entrada para el mensaje Twist |
| `output_topic` | `/cmd_vel_stamped` | Tópico de salida para el mensaje TwistStamped |
| `frame_id` | `/base_link` | Sistema de referencia que introducir en el header |

## Uso

Hay que configurar los parámetros para que la visualización sea correcta. Lo primero es elegir el nombre de los topic, el de entrada que se quiere visualizar, y el de salida que se podrá ver en Rviz. En cuanto al sistema de referencia, suele ser `base_link`, ya que habitualmente es el frame base del robot sobre el que se mide la odometría y se comanda la velocidad.

Para lanzar el nodo:

```
ros2 launch twist_to_stamped twist_to_stamped.launch.py
```

Los parámetros se pueden configurar al lanzar desde terminal para mayor comodidad:

```
ros2 launch twist_to_stamped twist_to_stamped.launch.py input_topic:=/cmd_vel output_topic:=/cmd_vel_stamped frame_id:=/base_link
```