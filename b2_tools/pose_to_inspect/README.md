# Pose to Inspect

## Descripción

Servidor de acción que ejecuta un patrón de inspección sistemático moviendo el Go2 a través de diferentes ángulos de pitch y yaw. Genera un patrón de barrido en zigzag y publica comandos de ángulos euler en un tópico especificado.

## Parámetros del Nodo

| Parámetro | Valor por Defecto | Descripción |
|-----------|-------------------|-------------|
| `output_topic` | `/euler` | Tópico para publicar comandos de ángulos euler |
| `velocity` | `0.1` | Velocidad de movimiento en rad/s |
| `control_dt_ms` | `100` | Período del bucle de control en milisegundos |

## Parámetros del Objetivo (Goal)

| Parámetro | Descripción |
|-----------|-------------|
| `min_pitch` | Ángulo mínimo de pitch en radianes |
| `max_pitch` | Ángulo máximo de pitch en radianes |
| `min_yaw` | Ángulo mínimo de yaw en radianes |
| `max_yaw` | Ángulo máximo de yaw en radianes |
| `max_pitch_step` | Tamaño máximo del paso de pitch en radianes |

## Uso

### Lanzar el Servidor

```bash
ros2 launch pose_to_inspect pose_to_inspect.launch.py
```

### Enviar Objetivo de Inspección

```bash
ros2 action send_goal /inspect_poses pose_to_inspect_interfaces/action/InspectPoses \
  "{min_pitch: -0.1, max_pitch: 0.5, min_yaw: -0.2, max_yaw: 0.2, max_pitch_step: 0.1}"
```

### Con Feedback

```bash
ros2 action send_goal /inspect_poses pose_to_inspect_interfaces/action/InspectPoses \
  "{min_pitch: -0.1, max_pitch: 0.5, min_yaw: -0.2, max_yaw: 0.2, max_pitch_step: 0.1}" --feedback
```
