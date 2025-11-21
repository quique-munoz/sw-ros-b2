# Reconstructor de Nubes de Puntos

## Descripción

Este nodo ROS 2 reconstruye nubes de puntos 3D mediante la acumulación y registro de múltiples nubes de puntos usando el algoritmo GICP (Generalized Iterative Closest Point). El sistema permite activar y desactivar la reconstrucción en función a un topic externo.

## Funcionalidad

- **Transformación**: La nube de puntos de entrada se transforma al frame fijo
- **Filtrado**: Eliminación de outliers estadísticos y filtrado por profundidad
- **Downsample**: Reducción de densidad usando voxel grid para optimizar rendimiento
- **Registro GICP**: Alineación automática de nubes de puntos usando Generalized ICP (GICP)

## Parámetros

| Parámetro | Valor por Defecto | Descripción |
|-----------|-------------------|-------------|
| `verbose` | `false` | Habilitar logs detallados |
| `input_topic` | `/pointcloud` | Tópico de entrada para las nubes de puntos |
| `activation_topic` | `/reconstruction/activation` | Tópico de entrada para activar/desactivar la reconstrucción |
| `output_topic` | `/pointcloud_reconstructed` | Tópico de salida para la nube reconstruida |
| `centroid_topic` | `/reconstruction/centroid` | Tópico para publicar el centroide de la nube reconstruida |
| `fixed_frame` | `odom` | Sistema de referencia fijo para las transformaciones |
| `tf_timeout_ms` | `50` | Timeout para transformaciones TF (milisegundos) |
| `max_depth` | `1.0` | Profundidad máxima para filtrar puntos (metros) |
| `voxel_grid_size` | `0.01` | Tamaño del voxel para submuestreo (metros) |
| `max_accumulated_points` | `2000000` | Número máximo de puntos acumulados |

## Tópicos

### Entrada
- **`/pointcloud`** (sensor_msgs/PointCloud2): Nube de puntos de entrada
- **`/reconstruction/activation`** (std_msgs/Bool): Señal para activar/desactivar reconstrucción

### Salida
- **`/pointcloud_reconstructed`** (sensor_msgs/PointCloud2): Nube de puntos reconstruida
- **`/reconstruction/centroid`** (geometry_msgs/PointStamped): Centroide de la nube reconstruida

## Proceso de Reconstrucción

1. **Activación**: El sistema espera una señal `true` en el tópico de activación
2. **Filtrado**: Se aplican filtros de profundidad y eliminación de outliers
3. **Transformación**: Conversión al sistema de referencia fijo usando TF2
4. **Downsample**: Reducción de densidad con voxel grid
5. **Registro**: Alineación con la nube acumulada usando GICP
6. **Acumulación**: Adición de la nube registrada a la reconstrucción
7. **Publicación**: Envío de la nube reconstruida y su centroide

## Uso

```bash
# Lanzar el reconstructor
ros2 launch pointcloud_reconstruct pointcloud_reconstruct.launch.py

# Activar reconstrucción
ros2 topic pub /reconstruction/activation std_msgs/msg/Bool "data: true"

# Desactivar reconstrucción
ros2 topic pub /reconstruction/activation std_msgs/msg/Bool "data: false"
```
