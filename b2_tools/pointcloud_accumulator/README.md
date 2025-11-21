# Acumulador de Nubes de Puntos

## Descripción

Este nodo ROS 2 acumula múltiples nubes de puntos (pointclouds) en un buffer temporal y las publica como una sola nube de puntos acumulada. Es útil para sensores sin patrón de repetición que proporcionan nubes de puntos poco densas.

## Funcionalidad

- **Acumulación**: Almacena nubes de puntos en un buffer durante una ventana de tiempo configurable
- **Transformación**: Transforma cada nube de puntos hasta el sistema de referencia objetivo, usando el sistema de referencia fijo `fixed_frame` (normalmente `/odom`)
- **Publicación**: Publica la nube de puntos acumulada periódicamente
- **Sincronización**: Soporta tanto tiempo de simulación como tiempo del sistema

## Parámetros

| Parámetro | Valor por Defecto | Descripción |
|-----------|-------------------|-------------|
| `input_topic` | `/pointcloud` | Tópico de entrada para las nubes de puntos |
| `output_topic` | `/pointcloud_accumulated` | Tópico de salida para la nube de puntos acumulada |
| `clock_topic` | `/clock` | Tópico del reloj para tiempo de simulación (dejar vacío para usar el tiempo de ROS) |
| `max_buffer_size` | `50` | Número máximo de nubes de puntos en el buffer |
| `time_window_seconds` | `2.0` | Ventana de tiempo para mantener nubes de puntos (segundos) |
| `timer_period_ms` | `2000` | Período del timer para publicar (milisegundos) |
| `target_frame` | `radar_flat` | Sistema de referencia final para las transformaciones |
| `fixed_frame` | `odom` | Sistema de referencia fijo sobre el que acumular las transformaciones |

## Uso

```
ros2 launch pointcloud_accumulator pointcloud_accumulator.launch.py
```